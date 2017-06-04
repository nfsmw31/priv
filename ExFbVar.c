#include <string.h>
#include "cal_conf.h"
#include "co_util.h"
#include "co_type.h"
#include "co_usr.h"
#include "access.h"
#include "objects.h"
#include "drv.h"
#include "co_sdo.h"
#include "sdo.h"
#include "co_mcpy.h"
#include "co_stor.h"
#include "co_def.h"
#include "nmt.h"
#include "nmt_s.h"
#include "co_pdo.h"
#include "pdo.h"
#include "co_odidx.h"
#include "co_setcp.h"
#include "heartbt.h"


#include <co_emcy.h>

#include "402fsa.def"
#include "Init.def"
#include "ExFbVar.def"
#include "Exe_IO.def"
#include "Current.def"
#include "Err_Hndl.def"
#include "Homing.def"
#include "i2c.def"
#include "Design.def"
#include "Velocity.def"
#include "FPGA.def"
#include "Modbus_comm.def"
#include "Flash.def"
#include "Homing.def"
#include "PtpGenerator.def"
#include "Analog_If.def"
#include "CanErr.def"


#include "Ser_Comm.var"
#include "ModCntrl.var"
#include "Drive.var"
#include "Units.var"
#include "ExFbVar.var"
#include "Extrn_Asm.var"
#include "Motor.var"
#include "Velocity.var"
#include "Foldback.var"
#include "Homing.var"
#include "Init.var"
#include "Exe_IO.var"
#include "Position.var"
#include "Hiface.var"
#include "Position.var"
#include "i2c.var"
#include "User_Var.var"
#include "PtpGenerator.var"
#include "FltCntrl.var"
#include "Flash.var"
#include "Drive_Table.var"
#include "Zeroing.var"
#include "Display.var"
#include "PhaseFind.var"
#include "Runtime.var"
#include "Record.var"
#include "MotorSetup.var"
#include "PtpGenerator.def"
#include "Exe_Hndl.var"
#include "Modbus_Comm.var"
#include "AutoTune.var"
#include "402fsa.h"
#include "Lxm_profile.var"
#include "FlashHandle.var"
#include "Err_Hndl.var"

#include "Prototypes.pro"
#include "BiSS_C.def"
#include "BiSS_C.pro"
#include "Display.pro"
#include "AN_FLTR.VAR"



extern MB_STRACT MB_ST;
extern int s16_CAN_SDO_Channel_Number;
extern IDENTITY_T p301_identity;
extern  unsigned long long u64_Warning_Latched;
extern int s16_QuickStop_Handled;
extern UNSIGNED32 p301_hb_consumer[4];
extern UNSIGNED32 p301_pre_defined_error_field[11];
extern UNSIGNED8 lNodeId;


/* Local types */

typedef struct
{
   unsigned int u16_object_id_1;
   unsigned int u16_object_id_2;
}MultRefObjectsStruct;

typedef struct
{
   int s16_drive;    // key
   int s16_fieldbus;// value
} UnitIndexConvStruct;


/* Communication variables */
// CAN
int p_fieldbus_bg_rx_data[FIELDBUS_BG_ENTRY_SIZE] = {0,};
int p_fieldbus_bg_tx_data[FIELDBUS_BG_ENTRY_SIZE] = {0,};
int p_fieldbus_rt_rx_data[FIELDBUS_RT_ENTRY_SIZE] = {0,};
int p_fieldbus_rt_tx_data[FIELDBUS_RT_ENTRY_SIZE] = {0,};

CAN_MSG_T  CAN_Msg_BG_buffer[CAN_SDO_BUFFER_SIZE];
unsigned int CAN_Msg_BG_buffer_wr_index;
unsigned int CAN_Msg_BG_buffer_rd_index;
int p_s16_bg_rx_data[FIELDBUS_BG_ENTRY_SIZE] = {0,};
int p_s16_bg_tx_data[FIELDBUS_BG_ENTRY_SIZE] = {0,};

int* p_bg_rx_ctrl_buffer;
int* p_bg_tx_ctrl_buffer;
int* p_bg_rx_data_buffer;
int* p_bg_tx_data_buffer;

unsigned int* p_u16_bg_rx_ctrl_in_index;
unsigned int* p_u16_bg_rx_ctrl_out_index;
unsigned int* p_u16_bg_tx_ctrl_in_index;
unsigned int* p_u16_bg_tx_ctrl_out_index;
unsigned int* p_u16_bg_rx_data_in_index;
unsigned int* p_u16_bg_rx_data_out_index;
unsigned int* p_u16_bg_tx_data_in_index;
unsigned int* p_u16_bg_tx_data_out_index;
unsigned int* p_u16_bg_tx_emcy_register_index;

unsigned int* p_u16_tx_nmt_state;
unsigned int* p_u16_tx_station_address;


unsigned int u16_CAN_commands_register = 0;
unsigned int u16_Is_Pdo_After_Operation_Mode = 0;


//EtherCAT//
int* p_EC_rt_rx_pdo_buffer;
int* p_EC_rt_tx_pdo_buffer;



// State "EtherCAT cable disconnected" for initialization in order to support the logic that a 1-time link needs
// to be detected before monitoring the Fb3 fault (see function "FieldBusEtherCATCableDisconnectedFault").
unsigned int u16_EC_commands_register = EC_CWORD_CABLE_DISCONNECTED_MASK;


/* Local variables */
const ErrorCodeConvStruct  FB_ErrorCodesConv_Table[]        = FB_ERROR_CODES_CONV_TABLE;
//const MultRefObjectsStruct    FB_MultASCIIRefObject_Table[]       = FB_MULT_REF_OBJECTS_TABLE;
const UnitIndexConvStruct  FB_UnitIndex_Conversion_Table[]     = FB_UNIT_INDEX_CONV_TABLE;
unsigned int            BGVAR_DEFINE(u16_fb_curr_obj_id)    = INIT_SDO_LAST_OBJECT;



//**********************************************************
// Function Name: FalExecuteFieldBusBgRxObject
// Description:
//          This function is called in response to Field Bus RX request
//
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
void FalExecuteFieldBusBgRxObject(int s16_trd_context)
{
   int i, ret_val = FAL_SUCCESS, drive = 0;
   unsigned int u16_address = 0;  // the adrress in the data buffer
   int          s16_data_size = 0;
   unsigned int u16_id = 0;
   unsigned int u16_prop  = 0;
   unsigned int u16_operation = 0;

   unsigned int u16_local_bg_rx_ctrl_in_index = 0;
   unsigned int u16_local_bg_rx_ctrl_out_index = 0;
   unsigned int u16_local_bg_rx_data_out_index = 0;

   //init buffer
   *p_s16_bg_rx_data = 0;

   // Read the value in DPR address 0x1707 (TOMI memory space).
   // This is an offset for the "LP RX FIFO CTRL" register.
   SET_DPRAM_FPGA_MUX_REGISTER(2);
   u16_local_bg_rx_ctrl_out_index = *(int*)p_u16_bg_rx_ctrl_out_index;

   // Read the value in DPR address 0x0783 (MOTI memory space).
   // This is an offset for the "LP RX FIFO CTRL" register.
   SET_DPRAM_FPGA_MUX_REGISTER(0);
   u16_local_bg_rx_ctrl_in_index = *(int*)p_u16_bg_rx_ctrl_in_index;


   // handle SAL functions that takes more time than fieldbus timeout for EC (CAN handled by InitDownloadRequest)

   if(BGVAR(u16_Ec_Sal_Not_Finished_Flag) && IS_EC_DRIVE_AND_COMMODE_1)
   {
      u16_local_bg_rx_ctrl_out_index = BGVAR(u16_Ec_Bg_Rx_Ctrl_Out_Index_Sal_Not_Finished);
     u16_local_bg_rx_data_out_index = BGVAR(u16_Ec_Bg_Rx_Data_Out_Index_Sal_Not_Finished);

   }

   //if there is an object to handle in RX buffers
   if(u16_local_bg_rx_ctrl_in_index != u16_local_bg_rx_ctrl_out_index)
   {
      STORE_EXECUTION_PARAMS_0_15

     if(IS_EC_DRIVE_AND_COMMODE_1)
     {
        if(!BGVAR(u16_Ec_Sal_Not_Finished_Flag))
          {
             BGVAR(u16_Ec_Bg_Rx_Ctrl_Out_Index_Sal_Not_Finished) = u16_local_bg_rx_ctrl_out_index;
            BGVAR(u16_Ec_Bg_Rx_Data_Out_Index_Sal_Not_Finished) = u16_local_bg_rx_data_out_index;
          }

          BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 0;
     }

      //read "in" index again to defeat metastability
      u16_local_bg_rx_ctrl_in_index = *(int*)p_u16_bg_rx_ctrl_in_index;

      //get object id (entry number inside of the parameter table)
      u16_id = *(int*)(p_bg_rx_ctrl_buffer + u16_local_bg_rx_ctrl_out_index);

      //increment pointer
      u16_local_bg_rx_ctrl_out_index = ((u16_local_bg_rx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //get the address for the data of the object resides in the data buffer (LP RX FIFO DATA)
      u16_address = *(int*)(p_bg_rx_ctrl_buffer + u16_local_bg_rx_ctrl_out_index);

      //increment pointer
      u16_local_bg_rx_ctrl_out_index = ((u16_local_bg_rx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //get properties word
      u16_prop = *(int*)(p_bg_rx_ctrl_buffer + u16_local_bg_rx_ctrl_out_index);

      //increment pointer
      u16_local_bg_rx_ctrl_out_index = ((u16_local_bg_rx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //increment pointer to disregard the reserved word
      u16_local_bg_rx_ctrl_out_index = ((u16_local_bg_rx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //init data index to begining of message
      u16_local_bg_rx_data_out_index = u16_address;

      //if write operation (for write only)
      if((u16_prop & 0x0F00)  == INIT_DOWNLOAD_REQUEST_MSG)
      {
         u16_operation = 1;

         //data information header must not be written on a buffer rollover - jump to begining of the buffer if so
         if(u16_local_bg_rx_data_out_index > (FIELDBUS_BG_DATA_BUFFER_SIZE - 3))
            u16_local_bg_rx_data_out_index = 0;

         //increament pointer to disregard complete size
         u16_local_bg_rx_data_out_index = ((u16_local_bg_rx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

         //increament pointer to disregard complete size
         u16_local_bg_rx_data_out_index = ((u16_local_bg_rx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

         // get the data size out of the LP RX FIFO DATA buffer
         SET_DPRAM_FPGA_MUX_REGISTER(1);
         s16_data_size = *(int*)(p_bg_rx_data_buffer + u16_local_bg_rx_data_out_index);

         //increment pointer in order to show on the first data value
         u16_local_bg_rx_data_out_index = ((u16_local_bg_rx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

         // Here load the SDO out of the Buffer
         for(i=0;i<s16_data_size;i++)
         {
            p_s16_bg_rx_data[i] = *(int*)(p_bg_rx_data_buffer + u16_local_bg_rx_data_out_index);

         //for 8 bits SIGNED objects make sure only 8 bits are passed
         if(FB_objects_array[u16_id].u16_attr & FAL_8_BITS_SIGNED_OBJ_MASK)
         {
            //move from 16 bits to 8 bits
            p_s16_bg_rx_data[0] <<= 8;
               p_s16_bg_rx_data[0] >>= 8;
         }

         //for 8 bits UNSIGNED objects make sure only 8 bits are passed
         if(FB_objects_array[u16_id].u16_attr & FAL_8_BITS_UNSIGNED_OBJ_MASK)
         {
               p_s16_bg_rx_data[0] &= 0x00FF;
         }

            //increment pointer to next data value
            u16_local_bg_rx_data_out_index = ((u16_local_bg_rx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
         }
       if (u16_id == SDO5FFDS1)
         FB_objects_size_array[SDO5FFDS1] = s16_data_size;


      }
      else//reading operation
      {
         // get the data size
         SET_DPRAM_FPGA_MUX_REGISTER(1);
         s16_data_size = *(int*)(p_bg_rx_data_buffer + u16_local_bg_rx_data_out_index);

         //increament pointer
         u16_local_bg_rx_data_out_index = ((u16_local_bg_rx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
      }

      //upadte out pointers
      SET_DPRAM_FPGA_MUX_REGISTER(2);

      // Update the value in DPR address 0x1707 (TOMI memory space).
      // This is an offset for the "LP RX FIFO CTRL" register.
      *p_u16_bg_rx_ctrl_out_index = u16_local_bg_rx_ctrl_out_index;
      // Update the value in DPR address 0x1708 (TOMI memory space).
      // This is an offset for the "LP RX FIFO DATA" register. Tell uBlaze
      // Where the TI is currently with the data pointer in order to not
      // overwrite information, which are not yet read by the TI.
      *p_u16_bg_rx_data_out_index = u16_local_bg_rx_data_out_index;

      /* Check the object validity */
      /* For the CAN bus drive the object has been already examined by the Port library flow,
         therefore no additional validation is required */
      if(IS_EC_DRIVE || IS_PN_DRIVE)
      {
         // Write internal CDHD-id to a global variable due to the "FalExecuteFieldBusCommandBg"
         // later in the code (called via function pointer "FB_objects_array[SDO_GENERIC_OBJECT].fp_bg".
         BGVAR(u16_fb_curr_obj_id) = u16_id;

         if((FB_objects_array[u16_id].u16_attr & FAL_PDO_MAP_ACT_OBJ_MASK) == 0)
         {
            ret_val = FalCheckObjectDefinition(s16_data_size, u16_operation, drive);
            if(ret_val == FAL_SUCCESS)
            {
               ret_val = FalCheckObjectValidity(u16_operation, (unsigned int)s16_data_size, p_s16_bg_rx_data, drive);
            }

            if(ret_val != FAL_SUCCESS)
            {
               // initiate abort on fail
               FalAbortRequest(s16_trd_context, (int)u16_id, (int)(u16_prop & 0x00FF), u16_operation, FalConvertErrorCode(ret_val));
               SET_DPRAM_FPGA_MUX_REGISTER(0);
               RESTORE_EXECUTION_PARAMS_0_15
               return;
            }
         }
      }

      //execute the matching function with the data recived  (for both read & write)
      // Call "FalExecuteFieldBusCommandBg" with data, CAN object-id and operation-variable.

      ret_val = FB_objects_array[SDO_GENERIC_OBJECT].fp_bg(p_s16_bg_rx_data,(u16_prop & 0x00FF), u16_operation);
      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         FalAbortRequest(s16_trd_context, (int)u16_id, (int)(u16_prop & 0x00FF), u16_operation, FalConvertErrorCode(ret_val));
         SET_DPRAM_FPGA_MUX_REGISTER(0);
         RESTORE_EXECUTION_PARAMS_0_15
         return;
      }
      else if(ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         if(u16_operation)//if can dont raise the flag
            {
            // handle SAL functions that takes more time than fieldbus timeout
            if (IS_CAN_DRIVE_AND_COMMODE_1)
            {
               InitDownloadRequest((int)BGVAR(u16_fb_curr_obj_id), INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, s16_data_size, p_s16_bg_rx_data);
            }
            else if (IS_EC_DRIVE_AND_COMMODE_1)
            {
               // For ECAT the next call for FAL function in case of SAL_NOT_FINISHED return value
               // will be executed only if "u16_Ec_Sal_Not_Finished_Flag" flag is set to "1"
               BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;
            }
         }
         SET_DPRAM_FPGA_MUX_REGISTER(0);
         RESTORE_EXECUTION_PARAMS_0_15
         return;
      }

      if(IS_EC_DRIVE || IS_PN_DRIVE)
      {
         if((u16_operation == 1) && ((FB_objects_array[u16_id].u16_attr & FAL_PDO_MAP_ACT_OBJ_MASK) == 0))
         {
            // Put the object into the object dictionary data base on write operation request
            ret_val = FalPutObject(s16_data_size, p_s16_bg_rx_data, drive);
            if(ret_val != FAL_SUCCESS)
            {
               // initiate abort on fail
               FalAbortRequest(s16_trd_context, (int)u16_id, (int)(u16_prop & 0x00FF), u16_operation, (unsigned long)ret_val);
               SET_DPRAM_FPGA_MUX_REGISTER(0);
               RESTORE_EXECUTION_PARAMS_0_15
               return;
            }
         }
      }
      SET_DPRAM_FPGA_MUX_REGISTER(0);
      RESTORE_EXECUTION_PARAMS_0_15
   }
}

//**********************************************************
// Function Name: FalExecuteFieldBusBgTxObject
// Description:
//          This function is called in response to MFB TX request
//
//
// Author: Lior
// Algorithm:
// Revisions:
// **********************************************************
int FalActualMotorPositionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long long s64_tmp = 0LL;
   unsigned int u16_temp_time;
   // AXIS_OFF;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO21A7S0, s16_msg_id);
   }
   else// read operation
   {     
         do {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_tmp = LLVAR(AX0_u32_Pfb_Internal_After_Mod_Lo); // This variable holds MFB
         } while (u16_temp_time != Cntr_3125);
     
     //convertion to counts!   (MFB is rellevant on dual-loop, otherwise use PFB)
     s64_tmp = s64_tmp/1000; //devided since value was originally multiplied by 1000
      manu_spec_Mfb = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Mfb & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Mfb >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO21A7S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalExecuteFieldBusBgTxObject
// Description:
//          This function is called in response to Field Bus TX request
//
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
void FalExecuteFieldBusBgTxObject()
{
/*   unsigned int u16_address = 0;  // the adrress in the data buffer
   int s16_data_size = 0;
   unsigned int u16_id = 0;
   unsigned int u16_properties = 0;
   unsigned long u32_complete_size = 0L;
   int i;

   //if there is an object to handle in TX buffers
   if(*p_u16_bg_tx_ctrl_in_index != *p_u16_bg_tx_ctrl_out_index)
   {
       //get object id (for both read & write)
       u16_id = *(int*)(p_bg_tx_ctrl_buffer + *p_u16_bg_tx_ctrl_out_index);

      //increament pointer
       *p_u16_bg_tx_ctrl_out_index = ((*p_u16_bg_tx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //get the address for the data of the object resides in the data buffer
         u16_address = *(int*)(p_bg_tx_ctrl_buffer + *p_u16_bg_tx_ctrl_out_index);

       //increament pointer
      *p_u16_bg_tx_ctrl_out_index = ((*p_u16_bg_tx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

       //get properties word - TBD for now
       u16_properties = *(int*)(p_bg_tx_ctrl_buffer + *p_u16_bg_tx_ctrl_out_index);

       //increament pointer
      *p_u16_bg_tx_ctrl_out_index = ((*p_u16_bg_tx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

      //increament pointer for reserved word
      *p_u16_bg_tx_ctrl_out_index = ((*p_u16_bg_tx_ctrl_out_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));


       //init data index to begining of message in data buffer
       *p_u16_bg_tx_data_out_index = u16_address;


       //if complete size is used
      if(u16_properties & 0x2000)
      {
          u32_complete_size = *(int*)(p_bg_tx_data_buffer + *p_u16_bg_tx_data_out_index);
         u32_complete_size += *(int*)(p_bg_tx_data_buffer + *p_u16_bg_tx_data_out_index + 1);
      }

      //increament pointer
      *p_u16_bg_tx_data_out_index = ((*p_u16_bg_tx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
      *p_u16_bg_tx_data_out_index = ((*p_u16_bg_tx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

       // get the data size is the first word
      s16_data_size = *(int*)(p_bg_tx_data_buffer + *p_u16_bg_tx_data_out_index);

       //increament pointer
      *p_u16_bg_tx_data_out_index = ((*p_u16_bg_tx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

      for(i=0;i<s16_data_size;i++)
      {
           p_s16_bg_tx_data[i] = *(int*)(p_bg_tx_data_buffer + *p_u16_bg_tx_data_out_index);

         //increament pointer
         *p_u16_bg_tx_data_out_index = ((*p_u16_bg_tx_data_out_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
      }
   }*/
}


void FalResetCanMsg() // reset a can message that was already handled
{
   int i;

   CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].cobId   = 0;
   CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].cobType = CO_COB_DISABLED;
   CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].length  = 0;

   for (i = 0; i < 8; i++)
      CAN_Msg_BG_buffer[CAN_Msg_BG_buffer_rd_index].pData[i] = 0;
}



//**********************************************************
// Function Name: FalExecuteFieldBusCommandBg
// Description:
//          This function is called in response to Field Bus command request
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalExecuteFieldBusCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int   drive = 0, ret_val = FAL_SUCCESS, s16_drive_units_index = 0, s16_fb_units_index = 0;
   int   s16_units_conv_req = 0, s16_arg_type = 0;
   unsigned int u16_drive_table_index = 0;
   LIST_ELEMENT_T* pCurObj;
   int s16_object_is_unsigned = 0;
   int s16_op_type;
   long s32_value;
   unsigned int u16_param;


   // for schnieder:
   // handle canopen objects which are P param.
   // these objects are handled seperatly in ExecutePParam(). so call the function directly from here
   // objects in range 0x4F00 to 0x4FFF are not p-params but general objects for schnieder: handle as cdhd objects

   // objects 0x4450 to 0x4454 are not mapped into p-params (alexander badura request 29/7/14), so dont use ExecutePParam() to handle them
   if (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_attr & FAL_P_PARAM_OBJ_MASK)
   {
      // find the object and check if signed or unsigned (for proper casting. e.g. writing 60000 in object 0x4134)
      pCurObj = searchObj(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index CO_COMMA_LINE_PARA_DECL);
      if (pCurObj == NULL)
      {
         return FAL_OBJECT_NOT_FOUND;
      }

      // data types for unsigned parameters (according to eds file):
      // data type u8  = 5
      // data type u16 = 6
      // data type u32 = 7
      if (pCurObj->pValDesc->varType >= CO_TYPEDESC_UNSIGNED8 && pCurObj->pValDesc->varType <= CO_TYPEDESC_UNSIGNED64) // fix bug 3737
      {
         s16_object_is_unsigned = 1;
      }

      u16_param = ConvertCanOpenIndexToPparam(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index);

      if (u16_operation == 1)// write operation
      {
         s16_op_type = OP_TYPE_WRITE;
         // all P 11 group is read only
         // return CO_E_NO_WRITE_PERM;

         // set value according to object's size
         if (FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)] == 1)
         {
            // 16 bit
            if (s16_object_is_unsigned)
               s32_value = (unsigned int)(p_data[0] & 0xffff);
            else
               s32_value = (int)(p_data[0] & 0xffff);
         }
         else if (FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)] == 2)
         {
            // 32 bit
            if (s16_object_is_unsigned)
            {
               s32_value = ((unsigned long)p_data[0] & 0xFFFF);                             //set 16 LSBits
               s32_value |= (unsigned long)p_data[1] << 16;   //set 16 MSBits
            }
            else
            {
               s32_value = (p_data[0] & 0xFFFF);                      //set 16 LSBits
               s32_value |= (long)(unsigned long)p_data[1] << 16;     //set 16 MSBits
            }
         }
         else
         {
            // TBD
            return FAL_INTERNAL_FW_FAULT;
         }
      }
      else // read operation
      {
         s16_op_type = OP_TYPE_READ;
      }

      /*
      now using ExecutePParam() instead of original code when accessing canopen object in the 0x4000 range:
      */

      ret_val = ExecutePParam(drive, u16_param, s16_op_type, &s32_value, LEX_CH_FIELDBUS);
      if (ret_val == SAL_NOT_FINISHED)
      {
         return ret_val;
      }

      if (ret_val != SAL_SUCCESS)
      {
         // special case for objects 0x4019 to 0x4020 (Parameter Mapping 1 to 8), p-params p0-25 to p0-32
         if (ret_val == NOT_PROGRAMMABLE && u16_param >= 25 && u16_param <= 32)
         {
            ret_val = FAL_CANNOT_TRANSFER_DATA;
         }

         return ret_val;
      }


      if (u16_operation == 1)// write operation
      {
         ret_val = FalInitDownloadRespond((int)BGVAR(u16_fb_curr_obj_id), s16_msg_id);
      }
      else
      {
         // set value according to object's size
         if (FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)] == 1)
         {
            // 16 bit
            *(int*)FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u32_addr = (int)s32_value;
            p_fieldbus_bg_tx_data[0] = s32_value & 0xffff;
         }
         else if (FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)] == 2)
         {
            // 32 bit
            *(long*)FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u32_addr = s32_value;
            p_fieldbus_bg_tx_data[0] = s32_value & 0xFFFF;        //set 16 LSBits
            p_fieldbus_bg_tx_data[1] = s32_value >> 16;           //set 16 MSBits
         }
         else
         {
            // TBD
            return FAL_INTERNAL_FW_FAULT;
         }

         ret_val = FalInitUploadRespond((int)BGVAR(u16_fb_curr_obj_id), s16_msg_id, (int)FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)], (int*)p_fieldbus_bg_tx_data);
      }

      return ret_val;
   }



   // execute on object specific handler if the object doesn't refer to the drive table
   // 0x6xxx objects are called directly via function pointer in the next if-condition.
   if(((FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_attr & (FAL_PDO_MAP_ACT_OBJ_MASK | FAL_ASCII_NON_EQUIV_OBJ_MASK | FAL_NON_SAL_USER_OBJ_MASK)) != 0) ||
      ((FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index & 0xF000) == 0x6000))
   {
      return(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].fp_bg(p_data, s16_msg_id, u16_operation));
   }


   /* Generic object handling */

   // get the corresponding drive table index
   ret_val = FalGetDriveTableIndex(BGVAR(u16_fb_curr_obj_id), &u16_drive_table_index);
   if(ret_val != FAL_SUCCESS)
      return ret_val;
   // get the argument type
   if ((MAX_WRITE_PARAMS(Commands_Table[u16_drive_table_index].u16_min_max_arg) > 1) &&
      (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index > 0))
   {
      s16_arg_type = M_PARAM_TYPE(Commands_Table[u16_drive_table_index].u32_argument_types, (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index - 1));
   }
   else
   {
      s16_arg_type = M_PARAM_TYPE(Commands_Table[u16_drive_table_index].u32_argument_types, 0);
      if(!s16_arg_type)
      {
            if(Commands_Table[u16_drive_table_index].u8_flags & 0x40)
                 s16_arg_type = 2; // decimal
            else
                 s16_arg_type = 1;// integer
      }

/*
       // if decimal value and a schneider objects (in the 0x4000 range)
       if ((s16_arg_type == 2) && (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index & 0xf000) == 0x4000)
       {
            // fix for p params which are float in commands table but integer in eds file
            // (these p params are multiplied by 1000 by the user instaed of being treated as float)
            LIST_ELEMENT_T* pCurObj = searchObj(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index CO_COMMA_LINE_PARA_DECL);
            unsigned int subIndex = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index;
            if(pCurObj == NULL)
                 return CO_E_NOT_EXIST;


            // object is a number (not string or domain).
            if (pCurObj->pValDesc[subIndex].attribute & CO_NUM_VAL)
           {
                 if (CO_READ_OD_DESC_U8(pCurObj, subIndex, varType) != CO_TYPEDESC_REAL32)
                    s16_arg_type = 1;// integer
            }
       }
*/

      // nitsan: for current units, set arg type to 1 (neeed to return int in mA units instead of decimal in A units).
      if (Commands_Table[u16_drive_table_index].str_units_name != NA &&
          Commands_Table[u16_drive_table_index].str_units_name[0] == 'A' &&
          Commands_Table[u16_drive_table_index].str_units_name[1] == '\0')
      {
          s16_arg_type = 1;// integer
      }
   }


   // determine if units conversion is required
   s16_units_conv_req = (int)(long)Commands_Table[u16_drive_table_index].str_units_var;
   if(s16_units_conv_req)
   {
      s16_drive_units_index = UnitsConversionIndex(Commands_Table[u16_drive_table_index].str_units_var, drive);
      if(s16_drive_units_index >= 0)
      {
         ret_val = FalConvertUnitIndex(s16_drive_units_index, &s16_fb_units_index);
         if(ret_val != FAL_SUCCESS)
            return ret_val;

         if(s16_fb_units_index == -1)// if actual units conversion is not available for Field Bus
            s16_units_conv_req = 0;
      }
      else
      {
         return FAL_INTERNAL_FW_FAULT;
      }
   }
   // execute the command
   if(u16_operation == 1)// write operation
   {
      ret_val = FalExecuteFBWriteCommandBg(u16_drive_table_index, s16_arg_type, s16_units_conv_req, s16_fb_units_index, p_data, drive);
      if(ret_val == FAL_SUCCESS)
      {
         if(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_attr & FAL_CONFIG_REQ_OBJ_MASK) // initiate config command
         {
            // after meeting at: 18/3/2013, it was decided that:
            // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
            // if config fail a fault will be generated
            ret_val = 0x0101;
            /*ret_val = */FalConfigCommandBg(&ret_val, s16_msg_id, 1);
            ret_val = FAL_SUCCESS;
         }

         if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)BGVAR(u16_fb_curr_obj_id), s16_msg_id);
      }
   }
   else// read operation
   {
      ret_val = FalExecuteFBReadCommandBg(u16_drive_table_index, s16_arg_type, s16_units_conv_req, s16_fb_units_index, p_fieldbus_bg_tx_data, drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitUploadRespond((int)BGVAR(u16_fb_curr_obj_id), s16_msg_id, (int)FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)], (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalExecuteFBWriteCommandBg
// Description:
//          This function is responsible to execute the write operation on the Field Bus object
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalExecuteFBWriteCommandBg(unsigned int u16_drive_table_index, int s16_arg_type, int s16_units_conv, int s16_fb_units_index, int* p_data, int drive)
{
   long long*     p_target_data;
   int         ret_val = FAL_SUCCESS, i = 0;
   unsigned int   u16_max_args_wr = 0, u16_obj_size = 0;
   unsigned int   u16_temp_idx = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_obj_size   = FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)];
   u16_max_args_wr = MAX_WRITE_PARAMS(Commands_Table[u16_drive_table_index].u16_min_max_arg);

   // get the index of the currently requested value for the objects' sub index
   u16_temp_idx = (unsigned int)((FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index - 1) && (u16_max_args_wr > 1));
   p_target_data  = &s64_Execution_Parameter[u16_temp_idx];

   // load the execution parameter if the argument is not a string or not a non-argument
   if(((s16_arg_type & 0x0003) != 3) && (s16_arg_type != 0))
   {
      if(UNSIGNED_PARAM(Commands_Table[u16_drive_table_index].u8_flags))
      {
         *p_target_data = (unsigned long long)p_data[(u16_obj_size - 1)];
      }
      else
      {
         *p_target_data = (long long)p_data[(u16_obj_size - 1)];
      }

      for(i = (u16_obj_size - 2); i >= 0; i--)
      {
         *p_target_data <<= 16;
         *p_target_data |= (unsigned long long)p_data[i] & 0xFFFF;
      }
   }

   // handle the data via argument type
   switch(s16_arg_type)
   {
      case 0:// no actual argument type is provided
         break;

      case 1:// integer type
      case 2:// decimal type
         if(s16_units_conv)
         {
            if((s16_fb_units_index == POSITION_CONVERSION_LIN_ROT) ||
               (s16_fb_units_index == VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM))
            {
               *p_target_data = (long long)((*(float*)p_target_data) * 1000.0);
            }

            // nitsan: current units for manufacturer specific objects should be mA (standard object are not handled here)
            if (s16_fb_units_index == FB_CAN_CURRENT_CONVERSION)
            {
                 // convert from micont(mA)/1000 to mA
                 *p_target_data = (*p_target_data * 1000LL) / (long long)BGVAR(s32_Motor_I_Cont);
            }

            // convert from user FB to drive internal units
            *p_target_data = MultS64ByFixS64ToS64(*p_target_data,
                           BGVAR(Unit_Conversion_Table[s16_fb_units_index]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[s16_fb_units_index]).u16_unit_conversion_to_internal_shr);


            // indicate the parameter is in internal units
            BGVAR(u8_Is_Internal_Units) = 1;

            // TBD: consider objects with double units conversion for special Field Bus pseudo float cases
         }
         else if(s16_arg_type == 2)
         {
            *p_target_data = (long long)((*(float*)p_target_data) * 1000.0);
         }
         break;

      case 3:// string type TBD !!!!!
         i = 0;
         do
         {
            u8_Execution_String[0][i] = p_data[i];
            i++;
         }
         while(i < u16_obj_size);
         // null terminate the internal variable
         u8_Execution_String[0][u16_obj_size] = '\0';
         break;

      default:
         break;

   }

   // execute on write command
   ret_val = ExecuteWriteCommand((int)u16_drive_table_index, drive, s64_Execution_Parameter);

   // reset internal units flag
   BGVAR(u8_Is_Internal_Units) = 0;

   return (ret_val);
}

//**********************************************************
// Function Name: FalExecuteFBReadCommandBg
// Description:
//          This function is responsible to execute the read operation on the Field Bus object
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalExecuteFBReadCommandBg(unsigned int u16_drive_table_index, int s16_arg_type, int s16_units_conv, int s16_fb_units_index, int* p_data, int drive)
{
   void*        p_fb_data_ptr;
   long long      s64_data_val = 0;
   unsigned int u16_max_args_wr = 0, u16_obj_size = 0;
   int            ret_val = FAL_SUCCESS, i = 0;
   int         s16_drive_units_index=0;


   u16_obj_size   = FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)];
   p_fb_data_ptr  = (void*)(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u32_addr);
   u16_max_args_wr = MAX_WRITE_PARAMS(Commands_Table[u16_drive_table_index].u16_min_max_arg);

   // execute on read command
   ret_val = ExecuteReadCommand((int)u16_drive_table_index, drive, &s64_data_val);


   if(ret_val == FAL_SUCCESS)
   {
      if((FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_attr & FAL_SERIAL_CONVERSION_ONLY_OBJ_MASK)) // meital: only serial conversion flag is set
      { // do a serial conversion
         s16_drive_units_index = UnitsConversionIndex(Commands_Table[u16_drive_table_index].str_units_var, drive);
         if(s16_drive_units_index >= 0)
         {
            s64_data_val = MultS64ByFixS64ToS64(s64_data_val,
                     BGVAR(Unit_Conversion_Table[s16_drive_units_index]).s64_unit_conversion_to_user_fix,
                     BGVAR(Unit_Conversion_Table[s16_drive_units_index]).u16_unit_conversion_to_user_shr);
         }
      }

      if(s16_units_conv)
      {
         s64_data_val = MultS64ByFixS64ToS64(s64_data_val,
                     BGVAR(Unit_Conversion_Table[s16_fb_units_index]).s64_unit_conversion_to_user_fix,
                     BGVAR(Unit_Conversion_Table[s16_fb_units_index]).u16_unit_conversion_to_user_shr);

         // nitsan: current units for manufacturer specific objects should be mA (standard object are not handled here)
         if (s16_fb_units_index == FB_CAN_CURRENT_CONVERSION)
         {
            // convert from micont(mA)/1000 to mA
            s64_data_val = (s64_data_val * (long long)BGVAR(s32_Motor_I_Cont)) / 1000LL;
         }
      }


      // handle the data via argument type
      switch(s16_arg_type)
      {
         case 0:// no actual argument type is provided
            //break; // TBD !!!
         case 1: // integer type
         case 2: // decimal type
            if (u16_max_args_wr <= 1 ||
               Commands_Table[u16_drive_table_index].u16_p_param_list_index) // schnieder can use list of params in one commands table line (e.g. P1-12)
            {
               memcpy_ram(p_fb_data_ptr, (void*)&s64_data_val, (size_t)u16_obj_size);

               if(s16_arg_type == 2)
               {
                  if(!s16_units_conv || ((s16_fb_units_index == POSITION_CONVERSION_LIN_ROT) ||
                                    (s16_fb_units_index == VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM)))
                     *(float*)p_fb_data_ptr = ((float)s64_data_val) * .001;
               }

            }
            else if(u16_max_args_wr > 1) // TBD !!!!
            {
               // TBD !!!
               // search for the start of the relevant string in the Serial Comm output buffer
               // Get access to relevant data in Serial Comm output buffer TBD!!!!!
            }
            break;

         case 3: // string type // TBD !!!
            i += 0;
            // TBD !!!
            // search for the start of the string in the Serial Comm output buffer
            // Get access to relevant data in Serial Comm output buffer TBD!!!!!
            // null terminate the object variable
            // update the length
            break;

         default:
            break;
      }
   }

   /* Copy the read data into the communication buffer */
   memcpy_ram((void*)p_data, (void*)p_fb_data_ptr, (size_t)u16_obj_size);

   return ret_val;
}


//**********************************************************
// Function Name: FalGoToResidentCommandBg
// Description:
//          This function is called in response object 0x2001 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalGoToResidentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      // special handling for both axises
      if(Enabled(0))
         return (DRIVE_ACTIVE);

      if((*(int*)p_data) != 0x55AA)
         return VALUE_IS_NOT_ALLOWED;

      // generate the response before executing the resident command
      manu_spec_Go_To_Resident_command = 0; // initialize the variable

      ret_val = FalInitDownloadRespond((int)SDO2001S0, s16_msg_id);
      if(ret_val == FAL_SUCCESS)
         ResidentCommand();
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_Go_To_Resident_command;
      ret_val = FalInitUploadRespond((int)SDO2001S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalConfigCommandBg
// Description:
//          This function is called in response object 0x2002 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalConfigCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;

   if(u16_operation == 1)// write operation
      {
      u8_tmp = (unsigned char)(*p_data & 0x00FF);
      if(u8_tmp != 0x01)
         return VALUE_IS_NOT_ALLOWED;

      //proceed to the configuration sequence:
      ret_val = SalConfigCommand(drive);

      manu_spec_Config_command = 0; // initialize the variable

      if(ret_val == FAL_SUCCESS && ((*p_data & 0xFF00) == 0))
         ret_val = FalInitDownloadRespond((int)SDO2002S0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (int)manu_spec_Config_command;
      ret_val = FalInitUploadRespond((SDO2002S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalAnInput1ZeroCommandBg
// Description:
//          This function is called in response object 0x20F8 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnInput1ZeroCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned char u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {

      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      s64_Execution_Parameter[0] = (long long)u16_tmp;
      ret_val = SalAnzero1(drive);



      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO20F8S0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);
       if (IS_EC_DRIVE_AND_COMMODE_1)
          BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }

      numOfCalls = 0;
      manu_spec_anin1zero = 0; // initialize the value
      ret_val = FalInitDownloadRespond((int)SDO20F8S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_anin1zero;
      ret_val = FalInitUploadRespond((int)SDO20F8S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalMotorSetupCommandBg
// Description:
//          This function is called in response object 0x2041 (Motor Setup) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalMotorSetupCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;

   if(u16_operation == 1)// write operation
   {
       u8_tmp = (unsigned char)(*p_data & 0x00FF);
       if(u8_tmp == 0x01)
       {
            s16_Number_Of_Parameters = 0;  // set num of parameters to zero to indicate start of motor setup process.
       }
       else if(u8_tmp == 0x00)
       {
            s16_Number_Of_Parameters = 1;  // set num of parameters to 1 to indicate abort/stop of motor setup process.
            s64_Execution_Parameter[0] = 0LL;
       }
       else
       {
            return VALUE_IS_NOT_ALLOWED;
       }

       //proceed to the motor setup sequence:
       ret_val = MotorSetupCommand(drive);

       manu_spec_Motor_Setup = 0; // initialize the variable

       if(ret_val == FAL_SUCCESS && ((*p_data & 0xFF00) == 0))
            ret_val = FalInitDownloadRespond((int)SDO2041S0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (int)manu_spec_Motor_Setup;
       ret_val = FalInitUploadRespond((SDO2041S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalDriveNameCommandBg
// Description:
//          This function is called in response object 0x2015 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalDriveNameCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, i = 0, ret_val = FAL_SUCCESS, co_num = 0;
   LIST_ELEMENT_T* pCurObj;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(IS_EC_DRIVE)
      co_num = 1;

   if(u16_operation == 1)// write operation
   {
      CO_UNPACK_MEMCPY((UNSIGNED8*)BGVAR(u8_DriveName), (UNSIGNED8*)p_data, FB_objects_size_array[SDO2015S0], co_num);

      // null terminate the internal variable
      BGVAR(u8_DriveName)[FB_objects_size_array[SDO2015S0]] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO2015S0, s16_msg_id);
   }
   else// read operation
   {
      pCurObj = searchObj(0x2015 CO_COMMA_LINE_PARA_DECL);
      do
      {
         manu_spec_Drive_Name_command[i] = BGVAR(u8_DriveName)[i];
         p_fieldbus_bg_tx_data[i] = manu_spec_Drive_Name_command[i];
         i++;
      }
      while (BGVAR(u8_DriveName)[i] != 0);
      if(i < (int)getObjSize(pCurObj, 0)) // null terminate the object variable
         manu_spec_Drive_Name_command[i] = 0;

      // update the length
      FB_objects_size_array[SDO2015S0] = i;

      ret_val = FalInitUploadRespond((int)SDO2015S0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalSetMotorNameCommandBg
// Description:
//          This function is called in response object SDO203FS0 from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************/
int FalMotorNameCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, i = 0, ret_val = FAL_SUCCESS, co_num = 0;
   LIST_ELEMENT_T* pCurObj;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(IS_EC_DRIVE)
      co_num = 1;

   if(u16_operation == 1)// write operation
   {
      CO_UNPACK_MEMCPY((UNSIGNED8*)BGVAR(u8_MotorName), (UNSIGNED8*)p_data, FB_objects_size_array[SDO203FS0], co_num);

      // null terminate the internal variable
      BGVAR(u8_MotorName)[FB_objects_size_array[SDO203FS0]] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO203FS0, s16_msg_id);
   }
   else// read operation
   {
      pCurObj = searchObj(0x203F CO_COMMA_LINE_PARA_DECL);
      do
      {
         manu_spec_Motor_Name_command[i] = BGVAR(u8_MotorName)[i];
         p_fieldbus_bg_tx_data[i] = manu_spec_Motor_Name_command[i];
         i++;
      }
      while (BGVAR(u8_MotorName)[i] != 0);
      if(i < (int)getObjSize(pCurObj, 0)) // null terminate the object variable
         manu_spec_Motor_Name_command[i] = 0;

      // update the length
      FB_objects_size_array[SDO203FS0] = i;

      ret_val = FalInitUploadRespond((int)SDO203FS0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalMotorSetupStCommandBg
// Description:
//          This function is called in response object 0x2042 (Motor Setup Status) from Fieldbus.
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************/
int FalMotorSetupStCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;    // avoid remark

   if(u16_operation != 1)// read operation
   {
       if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_FAILED)
       {
            // return value for failure
            manu_spec_Motor_Setup_St = 0x2000;
       }
       else if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_SUCCEEDED)
       {
            // return value for success
            manu_spec_Motor_Setup_St = 0x1000;
       }
       else
       {
            // return the step
            manu_spec_Motor_Setup_St = BGVAR(u16_Motor_Setup_State);
       }

       p_fieldbus_bg_tx_data[0] = (manu_spec_Motor_Setup_St & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Motor_Setup_St >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO2042S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDriveControlTempCommandBg
// Description:
//          This function is called in response object 0x2044 index 1 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalDriveControlTempCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2044S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Drive_Temp_command[1] = BGVAR(s16_Control_Temperature_Deg);

      p_fieldbus_bg_tx_data[0] = manu_spec_Drive_Temp_command[1];

      ret_val = FalInitUploadRespond((int)SDO2044S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDrivePowerTempCommandBg
// Description:
//          This function is called in response object 0x2044 index 2 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalDrivePowerTempCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2044S2, s16_msg_id);
   }
   else// read operation
   {
     manu_spec_Drive_Temp_command[2] = BGVAR(s16_Power_Temperature_Deg);

      p_fieldbus_bg_tx_data[0] = manu_spec_Drive_Temp_command[2];

      ret_val = FalInitUploadRespond((int)SDO2044S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDriveModuleTempCommandBg
// Description:
//          This function is called in response object 0x2044 index 3 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalDriveModuleTempCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   // AXIS_OFF;
   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2044S3, s16_msg_id);
   }
   else// read operation
   {
      if (VAR(AX0_u16_IPM_OT_DC_Enable) != 1)
     {
         BGVAR(u16_IPM_Temperature)=0;
         manu_spec_Drive_Temp_command[3] = 0;
        return NOT_SUPPORTED_ON_HW;
     }

      manu_spec_Drive_Temp_command[3] = BGVAR(u16_IPM_Temperature);
      p_fieldbus_bg_tx_data[0] = manu_spec_Drive_Temp_command[3];
      ret_val = FalInitUploadRespond((int)SDO2044S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalFactoryRestoreCommandBg
// Description:
//          This function is called in response object 0x204C from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalFactoryRestoreCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned char u8_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      if(numOfCalls == 0)
      {
         if(u8_tmp != 0x01)
            return VALUE_IS_NOT_ALLOWED;
      }

      ret_val = SalRestoreFactorySettingsCommand(drive);


      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the factory restore command execution has not been finished yet
         //request to handle the command again:
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO204CS0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u8_tmp);
       if (IS_EC_DRIVE_AND_COMMODE_1)
          BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }

      numOfCalls = 0;
      manu_spec_Factory_Restore_command = 0; // initialize the value
      ret_val = FalInitDownloadRespond((int)SDO204CS0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Factory_Restore_command;
      ret_val = FalInitUploadRespond((int)SDO204CS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHall1StateCommandBg
// Description:
//          This function is called in response object 0x2056 index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHallUStateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, param = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      // this is read-only command
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2056S1, s16_msg_id);
   }
   else// read operation
   {
        ReadHalls(&param, drive);
      manu_spec_Halls_State_command[1] = (param & 0x0004) >> 2;

      p_fieldbus_bg_tx_data[0] = manu_spec_Halls_State_command[1];

      ret_val = FalInitUploadRespond(SDO2056S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalHall2StateCommandBg
// Description:
//          This function is called in response object 0x2056 index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHallVStateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, param = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      // this is read-only command
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2056S2, s16_msg_id);
   }
   else// read operation
   {
        ReadHalls(&param, drive);
      manu_spec_Halls_State_command[2] = (param & 0x0002) >> 1;

      p_fieldbus_bg_tx_data[0] = manu_spec_Halls_State_command[2];

      ret_val = FalInitUploadRespond(SDO2056S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalHall3StateCommandBg
// Description:
//          This function is called in response object 0x2056 index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHallWStateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, param = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      // this is read-only command
      p_data += 0; //For Remark Avoidance
      ret_val = FalInitDownloadRespond((int)SDO2056S3, s16_msg_id);
   }
   else// read operation
   {
        ReadHalls(&param, drive);
      manu_spec_Halls_State_command[3] = param & 0x0001;

      p_fieldbus_bg_tx_data[0] = manu_spec_Halls_State_command[3];

      ret_val = FalInitUploadRespond(SDO2056S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalInvertHallSignalsCommandBg
// Description:
//          This function is called in response object 0x2057 index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInvertHallUSignalCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      VAR(AX0_s16_Halls_Inv) = (VAR(AX0_s16_Halls_Inv) & 0x0006) | (u8_tmp);

      ret_val = FalInitDownloadRespond((int)SDO2057S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Invert_Hall_Signals_command[1] = (unsigned char)(VAR(AX0_s16_Halls_Inv) & 0x0001);

      p_fieldbus_bg_tx_data[0] = manu_spec_Invert_Hall_Signals_command[1];

      ret_val = FalInitUploadRespond(SDO2057S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalInvertHallSignalsCommandBg
// Description:
//          This function is called in response object 0x2057 index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInvertHallVSignalCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      VAR(AX0_s16_Halls_Inv) = (VAR(AX0_s16_Halls_Inv) & 0x0005) | (u8_tmp << 1);

      ret_val = FalInitDownloadRespond((int)SDO2057S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Invert_Hall_Signals_command[2] = (unsigned char)((VAR(AX0_s16_Halls_Inv) & 0x0002) >> 1);

      p_fieldbus_bg_tx_data[0] = manu_spec_Invert_Hall_Signals_command[2];

      ret_val = FalInitUploadRespond(SDO2057S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalInvertHallSignalsCommandBg
// Description:
//          This function is called in response object 0x2057 index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInvertHallWSignalCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      VAR(AX0_s16_Halls_Inv) = (VAR(AX0_s16_Halls_Inv) & 0x0003) | (u8_tmp << 2);

      ret_val = FalInitDownloadRespond((int)SDO2057S3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Invert_Hall_Signals_command[3] = (unsigned char)((VAR(AX0_s16_Halls_Inv) & 0x0004) >> 2);

      p_fieldbus_bg_tx_data[0] = manu_spec_Invert_Hall_Signals_command[3];

      ret_val = FalInitUploadRespond(SDO2057S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar1Ind1CommandBg
// Description:
//          This function is called in response object 0x205A index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar1Ind1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0;


   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      manu_spec_Fdbk_Harmonic_Correction_Par1_command[0] = 0;// initialize the variable

      //if(u32_tmp != 1UL)
        // return VALUE_IS_NOT_ALLOWED;

      //ret_val = HarmonicCorrectionDesign(drive);
      //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO205AS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Fdbk_Harmonic_Correction_Par1_command[0];
      ret_val = FalInitUploadRespond(SDO205AS1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   ++drive;
   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar1Ind2CommandBg
// Description:
//          This function is called in response object 0x205A index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar1Ind2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0UL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      //BGVAR(s32_HcVact1)[0] = s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205AS2, s16_msg_id);
   }
   else// read operation
   {
      //manu_spec_Fdbk_Harmonic_Correction_Par1_command[1] = BGVAR(s32_HcVact1)[0];

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[1] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[1] >> 16);     // set 16 MSBits
      ret_val = FalInitUploadRespond(SDO205AS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar1Ind3CommandBg
// Description:
//          This function is called in response object 0x205A index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar1Ind3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0UL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      //BGVAR(s32_HcVact1)[1] = s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205AS3, s16_msg_id);
   }
   else// read operation
   {
      //manu_spec_Fdbk_Harmonic_Correction_Par1_command[2] = BGVAR(s32_HcVact1)[1];

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[2] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[2] >> 16);     // set 16 MSBits
      ret_val = FalInitUploadRespond(SDO205AS3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar1Ind4CommandBg
// Description:
//          This function is called in response object 0x205A index 4 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar1Ind4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long long s64_temp_pos = 0LL;
   unsigned long u32_tmp = 0UL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s64_temp_pos = MultS64ByFixS64ToS64((long long)u32_tmp,
                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // perform variable limits validation (depends on Field Bus scaling parameters)
      if((s64_temp_pos < 0LL) || (s64_temp_pos > 0x7FFFFFFFLL))
         return(VALUE_OUT_OF_RANGE);

      // assign the new value
      //BGVAR(s32_HcVact1)[2] = (long)s64_temp_pos;

      ret_val = FalInitDownloadRespond((int)SDO205AS4, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Fdbk_Harmonic_Correction_Par1_command[3] = (unsigned long)MultS64ByFixS64ToS64(0/*(long long)BGVAR(s32_HcVact1)[2]*/,
                                             BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                             BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[3] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par1_command[3] >> 16);     // set 16 MSBits
      ret_val = FalInitUploadRespond(SDO205AS4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar2Ind1CommandBg
// Description:
//          This function is called in response object 0x205B index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar2Ind1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u32_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      manu_spec_Fdbk_Harmonic_Correction_Par2_command[0] = 0;// initialize the variable

      if(u32_tmp != 1UL)
         return VALUE_IS_NOT_ALLOWED;

      //ret_val = HarmonicCorrectionDesign(drive);
      //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO205BS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Fdbk_Harmonic_Correction_Par2_command[0];

      ret_val = FalInitUploadRespond(SDO205BS1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   drive++;
   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar2Ind2CommandBg
// Description:
//          This function is called in response object 0x205B index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar2Ind2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      //BGVAR(s32_HcVact2)[0] = s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205BS2, s16_msg_id);
   }
   else// read operation
   {
      //manu_spec_Fdbk_Harmonic_Correction_Par2_command[1] = BGVAR(s32_HcVact2)[0];

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[1] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[1] >> 16);     // set 16 MSBits

      ret_val = FalInitUploadRespond(SDO205BS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar2Ind3CommandBg
// Description:
//          This function is called in response object 0x205B index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar2Ind3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      //BGVAR(s32_HcVact2)[1] = s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205BS3, s16_msg_id);
   }
   else// read operation
   {
      //manu_spec_Fdbk_Harmonic_Correction_Par2_command[2] = BGVAR(s32_HcVact2)[1];

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[2] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[2] >> 16);     // set 16 MSBits

      ret_val = FalInitUploadRespond(SDO205BS3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   ++drive;
   return(ret_val);
}

//**********************************************************
// Function Name: FalFdbkHarmonicCorrectionPar2Ind4CommandBg
// Description:
//          This function is called in response object 0x205B index 4 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFdbkHarmonicCorrectionPar2Ind4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long long s64_temp_pos = 0LL;
   unsigned long u32_tmp = 0UL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s64_temp_pos = MultS64ByFixS64ToS64((long long)u32_tmp,
                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // perform variable limits validation (depends on Field Bus scaling parameters)
      if((s64_temp_pos < 0LL) || (s64_temp_pos > 0x7FFFFFFFLL))
         return(VALUE_OUT_OF_RANGE);

      // assign the new value
      //BGVAR(s32_HcVact2)[2] = (long)s64_temp_pos;

      ret_val = FalInitDownloadRespond((int)SDO205BS4, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Fdbk_Harmonic_Correction_Par2_command[3] = (unsigned long)MultS64ByFixS64ToS64(0/*(long long)BGVAR(s32_HcVact2)[2]*/,
                                             BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                             BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[3] & 0xFFFF);  // set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (int)(manu_spec_Fdbk_Harmonic_Correction_Par2_command[3] >> 16);     // set 16 MSBits
      ret_val = FalInitUploadRespond(SDO205BS4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar1Ind1CommandBg
// Description:
//          This function is called in response object 0x205D index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar1Ind1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)(p_data[0] & 0x00FF);

      manu_spec_Harmonic_Curr_ICMD_Par1_command.Config = 0; // initialize the variable

      if(u8_tmp != 0x01)
         return VALUE_IS_NOT_ALLOWED;

      //ret_val = HarmonicCorrectionDesign(drive);
      //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO205DS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Harmonic_Curr_ICMD_Par1_command.Config;
      ret_val = FalInitUploadRespond(SDO205DS1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   ++drive;
   return(ret_val);
}



//**********************************************************
// Function Name: FalHarmonicCurrICMDPar1Ind2CommandBg
// Description:
//          This function is called in response object 0x205D index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar1Ind2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_tmp = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
         s16_tmp = (int)p_data[0] & 0xFFFF;

         VAR(AX0_s16_Icmd_Harmonic_Amp_1) = s16_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205DS2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_1 = VAR(AX0_s16_Icmd_Harmonic_Amp_1);

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_1;
      ret_val = FalInitUploadRespond(SDO205DS2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar1Ind3CommandBg
// Description:
//          This function is called in response object 0x205D index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar1Ind3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_tmp = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s16_tmp = (int)p_data[0] & 0xFFFF;

      VAR(AX0_s16_Icmd_Harmonic_Phase_1) = s16_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205DS3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_2 =VAR(AX0_s16_Icmd_Harmonic_Phase_1);

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_2;
      ret_val = FalInitUploadRespond(SDO205DS3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar1Ind4CommandBg
// Description:
//          This function is called in response object 0x205D index 4 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar1Ind4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, shr_val, s16_icmd;
   long long half_for_rounding = 0LL;
   long s32_tmp = 0L;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;
      s32_tmp = (long)(*(float*)(&s32_tmp) * 1000.0);

      // perform variable limits validation (depends on Field Bus scaling parameters)
      shr_val = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
      if(shr_val > 0)
         half_for_rounding = 1LL << ((long long)shr_val - 1);
      s16_icmd = (int)((s32_tmp * (long long)BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + \
                                       half_for_rounding) >> (long long)shr_val);

      if((s16_icmd < 0) || (s16_icmd > 0x6666))
         return (VALUE_OUT_OF_RANGE);

      // assign the new value
      VAR(AX0_s16_Icmd_Harmonic_Phase_1) = (int)s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205DS4, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_3 = (float)(VAR(AX0_s16_Icmd_Harmonic_Phase_1)) * .001;

      p_fieldbus_bg_tx_data[0] = (*(long*)&manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_3) & 0xFFFF;
      p_fieldbus_bg_tx_data[1] = (*(long*)&manu_spec_Harmonic_Curr_ICMD_Par1_command.Argument_3) >> 16;
      ret_val = FalInitUploadRespond(SDO205DS4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar2Ind1CommandBg
// Description:
//          This function is called in response object 0x205E index 1 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar2Ind1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)(p_data[0] & 0x00FF);

      manu_spec_Harmonic_Curr_ICMD_Par2_command.Config = 0; // initialize the variable

      if(u8_tmp != 0x01)
         return VALUE_IS_NOT_ALLOWED;

      //ret_val = HarmonicCorrectionDesign(drive);
      //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO205ES1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Harmonic_Curr_ICMD_Par2_command.Config;
      ret_val = FalInitUploadRespond(SDO205ES1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar2Ind2CommandBg
// Description:
//          This function is called in response object 0x205E index 2 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar2Ind2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int u16_tmp = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (int)p_data[0] & 0xFFFF;

      VAR(AX0_s16_Icmd_Harmonic_Phase_2) = u16_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205ES2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_1 = VAR(AX0_s16_Icmd_Harmonic_Phase_2);

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_1 & 0xFFFF);
      ret_val = FalInitUploadRespond(SDO205ES2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar2Ind3CommandBg
// Description:
//          This function is called in response object 0x205E index 3 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar2Ind3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int u16_tmp = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (int)p_data[0] & 0xFFFF;

      VAR(AX0_s16_Icmd_Harmonic_Phase_2) = u16_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205ES3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_2 = VAR(AX0_s16_Icmd_Harmonic_Phase_2);

      p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_2 & 0xFFFF);
      ret_val = FalInitUploadRespond(SDO205ES3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHarmonicCurrICMDPar2Ind4CommandBg
// Description:
//          This function is called in response object 0x205E index 4 from Fieldbus.
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHarmonicCurrICMDPar2Ind4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, shr_val, s16_icmd;
   long long half_for_rounding = 0LL;
   long s32_tmp = 0L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;
      s32_tmp = (long)(*(float*)(&s32_tmp) * 1000.0);

      // perform variable limits validation (depends on Field Bus scaling parameters)
      shr_val = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
      if(shr_val > 0)
         half_for_rounding = 1LL << ((long long)shr_val - 1);
      s16_icmd = (int)((s32_tmp * (long long)BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + \
                                       half_for_rounding) >> (long long)shr_val);

      if((s16_icmd < 0) || (s16_icmd > 0x6666))
         return (VALUE_OUT_OF_RANGE);

      // assign the new value
      //BGVAR(s16_HcIcmd2)[2] = (int)s32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO205ES4, s16_msg_id);
   }
   else// read operation
   {
      //manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_3 = (float)(BGVAR(s16_HcIcmd2)[2]) * .001;

      p_fieldbus_bg_tx_data[0] = (*(long*)(&manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_3)) & 0xFFFF;
      p_fieldbus_bg_tx_data[1] = (*(long*)(&manu_spec_Harmonic_Curr_ICMD_Par2_command.Argument_3)) >> 16;
      ret_val = FalInitUploadRespond(SDO205ES4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalHoldPositionCommandBg
// Description:
//          This function is called in response object 0x2063 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalHoldPositionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned int u16_tmp = 0;
   int ret_val = FAL_SUCCESS, drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;
      if(u16_tmp == 0x0001)
      {
         p402_controlword |= HALT_MOVE;
      }
      else
      {
         if(numOfCalls < 2)
         {
            if(p402_controlword & HALT_MOVE)
               p402_controlword &= ~HALT_MOVE;
            else
               p402_controlword &= ~0x10;

            if (IS_CAN_DRIVE_AND_COMMODE_1)
               InitDownloadRequest((int)SDO2063S0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);
          if (IS_EC_DRIVE_AND_COMMODE_1)
             BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

            numOfCalls++;
            return(FAL_SUCCESS);
         }
         else
            p402_controlword |= 0x10;
      }
      numOfCalls = 0;
      ret_val = FalInitDownloadRespond((int)SDO2063S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Hold_Position_command = BGVAR(u16_Hold);
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Hold_Position_command;

      ret_val = FalInitUploadRespond(SDO2063S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalIndexFindCommandBg
// Description:
//          This function is called in response object 0x206D from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalIndexFindCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, drive = 0;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
   }
   else// read operation
   {
      ret_val = SalIndexFindCommand(drive);
      if(ret_val != FAL_SUCCESS)
      {
         manu_spec_Index_Find_command = 0;
      }
     else
     {
         manu_spec_Index_Find_command = 1;
     }
      p_fieldbus_bg_tx_data[0] = manu_spec_Index_Find_command;
      ret_val = FalInitUploadRespond(SDO206DS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalInputInversionIndexCommandBg
// Description:
//          This function is called in response object 0x2070 index 1 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInputInversionIndexCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
    unsigned int u16_mask, u16_tmp;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = p_data[0] & 0xFFFF;

      u16_mask = (1 << (int)(u16_tmp - 1));

      if (!(u16_Supported_Dig_Inputs_Mask & u16_mask))
         return (I_O_NOT_SUPPORTED);

      ret_val = FalInitDownloadRespond((int)SDO2070S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Input_Inversion_command[1];

      ret_val = FalInitUploadRespond(SDO2070S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalInputInversionValueCommandBg
// Description:
//          This function is called in response object 0x2070 index 2 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInputInversionValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_mask, u16_tmp;
   unsigned int u16_value;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u16_tmp = p_data[0] & 0xFFFF;

      u16_mask = (1 << (int)(manu_spec_Input_Inversion_command[1] - 1));

      if (u16_tmp == 0)
         BGVAR(u16_Dig_In_Polar) &= ~u16_mask;
      else
         BGVAR(u16_Dig_In_Polar) |= u16_mask;

      // This is to make sure both axes has the same value, it is defined as BGVAR for "SAVE" purposes
      u16_value = BGVAR(u16_Dig_In_Polar);
      drive = 1;
      BGVAR(u16_Dig_In_Polar) = u16_value;
      drive = 0;

      WriteInputPolarityToFPGA();

      ret_val = FalInitDownloadRespond((int)SDO2070S2, s16_msg_id);
   }
   else// read operation
   {
      u16_mask = (1 << (int)(manu_spec_Input_Inversion_command[1] - 1));

      if ((BGVAR(u16_Dig_In_Polar) & u16_mask) != 0)
         manu_spec_Input_Inversion_command[2] = 1;
      else
         manu_spec_Input_Inversion_command[2] = 0;

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Input_Inversion_command[2];

      ret_val = FalInitUploadRespond(SDO2070S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalNlTuneStartCommandBg
// Description:
//          This function is called in response object 0x2093 sub index 0 from Fieldbus.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalNlTuneStartCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   int s16_tmp;

   if(u16_operation == 1)// write operation
   {
       // check that fieldbus has access right to start auto tune
       ret_val = CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_AUTO_TUNING, LEX_CH_FIELDBUS);
       if (ret_val == SAL_SUCCESS)
       {
            // we need to take data int by int to prevent bad data due to odd address
            s16_tmp = (long)p_data[0] & 0xFFFF;
            ret_val = SalNLTuneCommand(s16_tmp, drive);
            if (ret_val == SAL_SUCCESS)
            {
                 ret_val = FalInitDownloadRespond((int)SDO2093S0, s16_msg_id);
            }
       }
   }
   else// read operation
   {
       manu_spec_NL_Tune_Status_command = VAR(AX0_u16_NL_Tune_Status);
       ret_val = FalInitUploadRespond(SDO2093S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalNLTuneStSelectCommandBg
// Description:
//          This function is called in response object 0x20ED sub index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalNLTuneStSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   p_data += 0;

   u16_operation += 0;

   if(u16_operation == 1)// write operation
   {
      // write operation
      if ((p_data[0]) > 4)
         ret_val = VALUE_TOO_HIGH;
      else
         manu_spec_NLTune_St_command.StatusSelect = p_data[0];

      if (ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20EDS1, s16_msg_id);
   }
   else// read operation
   {
      ret_val = DomainHandler(drive,(void *)&manu_spec_NLTune_St_command,0x20ED,NLTuneStatusCommand,PACKETS_PARAM_TRANSFER_ACTIVATE);
      p_fieldbus_bg_tx_data[0] = manu_spec_NLTune_St_command.StatusSelect;
      ret_val = FalInitUploadRespond((int)SDO20EDS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalNLTuneStSelectCommandBg
// Description:
//          This function is called in response object 0x20ED sub index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalNLTuneStGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   p_data += 0;
   u16_operation += 0;

   DomainHandler(drive,(void *)&manu_spec_NLTune_St_command,0x20ED,NULL,DOMAIN_DATA_UPLOAD);

   if (IS_EC_DRIVE)
      p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_NLTune_St_command.Domain & 0xffff);
   ret_val = FalInitUploadRespond((int)SDO20EDS2, s16_msg_id, u16_Domain_Data_Length, (int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}

//**********************************************************************
//  Function Name: DomainDataUploadHandler
//  Description:
//  handles the compression of the array data
//  and setting the pointer and data size to PORT_CAN domain functions
//
//  DOMAIN_DATA_UPLOAD
//  At uploading data ,rearanging the array content so each byte is used,instead of sending extra 0 content bytes
//
//  PACKETS_PARAM_TRANSFER_ACTIVATE /   PACKETS_TRANSFER_ACTIVATE
//  function activation
//  in case of parameterized function,the user will have to send its parameter value each time by writing to the sdo.
//  and then read the sdo for activating the function.
//  in case of unparameterized function,the user will only have to read the sdo.
//
// Author: Gil
// Algorithm:
// Revisions:
//*********************************************************************
int DomainHandler(int drive,void * obj_ptr ,int u16_index,int (*f_ptr)(int ,int ),int action)
{
   unsigned int i;
   int u16_inner_ret_val,ret_val=SAL_SUCCESS;

   // rearanging the array content
   if (action == DOMAIN_DATA_UPLOAD)
   {
      if (u16_General_Domain[0] == 0)
      {
         strcpy((char*)&u16_General_Domain[0],"  ");
      }
      setDomainAddr(u16_index, 2, (UNSIGNED8 *) &u16_General_Domain[0] CO_COMMA_LINE_PARA_DECL);
      u16_Domain_Data_Length = strlen((char *)&u16_General_Domain[0]);

      for (i=1;i<u16_Domain_Data_Length;i++)
      {
         if (!(i%2))
            *((char *)&u16_General_Domain[0]+i/2)=*((char *)&u16_General_Domain[0]+i);
         else
            *((char *)&u16_General_Domain[0]+i/2)|=*((char *)&u16_General_Domain[0]+i)<<8;
      }
      setDomainSize(u16_index,2, u16_Domain_Data_Length CO_COMMA_LINE_PARA_DECL);
   }
   // function activation
   else if (action != DOMAIN_DATA_UPLOAD)
   {
      STORE_EXECUTION_PARAMS_0_1
      if (action == PACKETS_PARAM_TRANSFER_ACTIVATE)
      {
         s64_Execution_Parameter[0]= ((NLTUNE_DOMAIN *)obj_ptr)->StatusSelect;
         s16_Number_Of_Parameters=1;
      }
      u16_inner_ret_val = f_ptr(drive,FB_IN_USE);

      if (u16_inner_ret_val == SAL_SUCCESS)
         ret_val = PACKETS_TRANSFER_COMPLETED;
      else if (u16_inner_ret_val != SAL_NOT_FINISHED)
         ret_val = PACKETS_TRANSFER_FAILED;
      else
      {
         if (u16_General_Domain[0]==0)
            ret_val = PACKETS_NOT_READY;
         else
            ret_val = PACKETS_PENDING;
      }

      ((NLTUNE_DOMAIN *)obj_ptr)->StatusSelect = ret_val;
      RESTORE_EXECUTION_PARAMS_0_1
   }
   return ret_val;
}

//**********************************************************
// Function Name: FalPfbOffCommandBg
// Description:
//          This function is called in response object 0x2095 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:FalPfbOffCommandBg
//**********************************************************
int FalPfbOffCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   long s32_tmp=0;
   long long s64_tmp = 0LL;
   unsigned int u16_temp_time = 0;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
          // if this object is saved in MTP (motor name plate), it cannot be modified.
          // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
         if (BGVAR(u16_MTP_Mode) == 1)
       {
          return MTP_USED;
       }

         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

       do {
          u16_temp_time = Cntr_3125;
            LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) = (unsigned long)(s64_tmp & 0x00000000FFFFFFFF);
            LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) = (long)(s64_tmp >> 32LL);
            } while (u16_temp_time != Cntr_3125);

         ret_val = FalInitDownloadRespond((int)SDO2095S0, s16_msg_id);
   }
   else// read operation
   {
      do {
        u16_temp_time = Cntr_3125;
         s64_tmp = ((long long)LVAR(AX0_s32_Pos_Fdbk_Offset_Hi)) << 32LL;
         s64_tmp |= (((long long)LVAR(AX0_u32_Pos_Fdbk_Offset_Lo)) & 0x00000000FFFFFFFF);
        } while (u16_temp_time != Cntr_3125);


       manu_spec_Pos_Fdbk_Offset_command = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
       p_fieldbus_bg_tx_data[0] = (manu_spec_Pos_Fdbk_Offset_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Pos_Fdbk_Offset_command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2095S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalControlHwVerCommand
// Description:
//          This function is called in response object 0x217D sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:FalControlHwVerCommand
//**********************************************************
int FalControlHwVerCommand(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
   }
   else// read operation
   {
      
      manu_spec_u16_Hw_Ver_Command[1] = u16_Control_Board_Rev;
      p_fieldbus_bg_tx_data[0] = manu_spec_u16_Hw_Ver_Command[1];
      ret_val = FalInitUploadRespond(SDO217DS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalPowerHwVerCommand
// Description:
//          This function is called in response object 0x217D sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:FalPowerHwVerCommand
//**********************************************************
int FalPowerHwVerCommand(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
   }
   else// read operation
   {
      manu_spec_u16_Hw_Ver_Command[2] = u16_Power_Board_Rev;
      p_fieldbus_bg_tx_data[0] = manu_spec_u16_Hw_Ver_Command[2];
      ret_val = FalInitUploadRespond(SDO217DS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalRefOffsetValueCommandBg
// Description:
//          This function is called in response object 0x217C from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:FalPfbOffCommandBg
//**********************************************************
int FalRefOffsetValueCommand(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   unsigned int u16_temp_time = 0;
   long s32_tmp=0;
   long long s64_tmp = 0LL;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

     do{
        u16_temp_time = Cntr_3125; // the same RT Interrupt
         LVAR(AX0_u32_Home_Offset_User_Lo) = (unsigned long)(s64_tmp & 0x00000000FFFFFFFF);
         LVAR(AX0_s32_Home_Offset_User_Hi) = (long)(s64_tmp >> 32LL);
      }while (u16_temp_time != Cntr_3125);

      ret_val = FalInitDownloadRespond((int)SDO217CS0, s16_msg_id);
   }
   else// read operation
   {
      do {
         u16_temp_time = Cntr_3125; // the same RT Interrupt
         s64_tmp = ((long long)LVAR(AX0_s32_Home_Offset_User_Hi)) << 32LL;
         s64_tmp |= (((long long)LVAR(AX0_u32_Home_Offset_User_Lo)) & 0x00000000FFFFFFFF);
         } while (u16_temp_time != Cntr_3125);

      manu_spec_s32_Ref_Offset_Val_Command = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
      p_fieldbus_bg_tx_data[0] = (manu_spec_s32_Ref_Offset_Val_Command & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_s32_Ref_Offset_Val_Command >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO217CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalOutPolar2CommandBg
// Description:
//          This function is called in response object 0x209B index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalOutputInversionIndexCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0, u16_mask;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      u16_mask = (1 << (int)(u16_tmp - 1));

      if(!(u16_Supported_Dig_Outputs_Mask & u16_mask))
         return (I_O_NOT_SUPPORTED);

      ret_val = FalInitDownloadRespond((int)SDO209BS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Polar_command[1];
      ret_val = FalInitUploadRespond((int)SDO209BS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalOutPolar2CommandBg
// Description:
//          This function is called in response object 0x209B index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalOutputInversionValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0, u16_mask;
   unsigned int u16_value;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      u16_mask = (1 << (int)(manu_spec_Polar_command[1] - 1));

      if (u16_tmp == 0)
         BGVAR(u16_Dig_Out_Polar) &= ~u16_mask;
      else
         BGVAR(u16_Dig_Out_Polar) |= u16_mask;

      // This is to ensure both axes has the same value, it is defined as BGVAR for "SAVE" purposes
      u16_value = BGVAR(u16_Dig_Out_Polar);
      drive = 1;
      BGVAR(u16_Dig_Out_Polar) = u16_value;
      drive = 0;

      FastOutInv(drive);
      ret_val = FalInitDownloadRespond((int)SDO209BS2, s16_msg_id);
   }
   else// read operation
   {
      u16_mask = (1 << (int)(manu_spec_Polar_command[1] - 1));

      if ((BGVAR(u16_Dig_Out_Polar) & u16_mask) != 0)
         manu_spec_Polar_command[2] = 1;
      else
         manu_spec_Polar_command[2] = 0;

      p_fieldbus_bg_tx_data[0] = manu_spec_Polar_command[2];
      ret_val = FalInitUploadRespond((int)SDO209BS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalInModeConfigCommandBg
// Description:
//          This function is called in response object 0x209C index 1 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalOutModeOutputIndexCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      if (!(u16_Supported_Dig_Outputs_Mask & (1 << (int)(u16_tmp - 1))))
         return (I_O_NOT_SUPPORTED);

      ret_val = FalInitDownloadRespond((int)SDO209CS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Out_Mode_command[1];

      ret_val = FalInitUploadRespond(SDO209CS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalReadSwenCommandBg
// Description:
//          This function is called in response object 0x20A2 index 0 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalReadSwenCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive =0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   u16_operation += 0;
   manu_spec_SW_En_Status_command = (BGVAR(s16_DisableInputs) & SW_EN_MASK)?0:1;
   p_fieldbus_bg_tx_data[0] = (int)manu_spec_SW_En_Status_command;
   ret_val = FalInitUploadRespond(SDO20A2S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return(ret_val);
}


//**********************************************************
// Function Name: FalOutModeOutputFunctionCommandBg
// Description:
//          This function is called in response object 0x209C index 2 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalOutModeOutputFunctionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, temp_state = 0;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       // Set the same outmode --> do nothing
       if (u16_Out_Mode[manu_spec_Out_Mode_command[1] - 1] == (int)u16_tmp)
       {
            ret_val = FalInitDownloadRespond((int)SDO209CS2, s16_msg_id);
            return ret_val;
       }

       if (u16_tmp == NO_FUNC_OUT) // If no functionality set then init out value to current state
       {
            // Support for CDHD HW is needed
            temp_state = OxValue((int)(manu_spec_Out_Mode_command[1] - 1));
            u16_Out_State[manu_spec_Out_Mode_command[1] - 1] = temp_state;
       }

       //accept outmode
       u16_Out_Mode[manu_spec_Out_Mode_command[1] - 1] = u16_tmp;

       FastOutInv(drive);
       ret_val = FalInitDownloadRespond((int)SDO209CS2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Out_Mode_command[2] = u16_Out_Mode[manu_spec_Out_Mode_command[1] - 1];

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Out_Mode_command[2];

      ret_val = FalInitUploadRespond(SDO209CS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalServoSenseReadCommandBg
// Description:
//          This function is called in response object 0x217E from Fieldbus.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalServoSenseReadCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0UL;
   static int numOfCalls = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      manu_spec_s32_Servo_Sense_Read_Command = (long)u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO217ES0, s16_msg_id);
   }
   else// read operation
   {
     s64_Execution_Parameter[0] = manu_spec_s32_Servo_Sense_Read_Command;     
     s16_Number_Of_Parameters = 1;

     ret_val = SalSrvSnsReadAddr(drive);

     if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
     {
        return(ret_val);
     }

     else if (ret_val == SAL_NOT_FINISHED)
     {
       //the command execution has not been finished yet
       //request to handle the command again:
       if (IS_CAN_DRIVE_AND_COMMODE_1)
          InitUploadRequest((int)SDO217ES0, 2, INIT_UPLOAD_REQUEST_MSG);
       if (IS_EC_DRIVE_AND_COMMODE_1)
          BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

       numOfCalls++;
       return(FAL_SUCCESS);
     }

     numOfCalls = 0;
     manu_spec_s32_Servo_Sense_Read_Command = s32_Servo_Sense_Read_Value; // initialize the value

     p_fieldbus_bg_tx_data[0] = (manu_spec_s32_Servo_Sense_Read_Command & 0xFFFF); //set 16 LSBits
     p_fieldbus_bg_tx_data[1] = (manu_spec_s32_Servo_Sense_Read_Command >> 16);    //set 16 MSBits
     ret_val = FalInitUploadRespond((int)SDO217ES0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalServoSenseWriteAddressCommandBg
// Description:
//          This function is called in response object 0x217F sub index 1 from Fieldbus
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalServoSenseWriteAddressCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      manu_spec_s32_Servo_Sense_Write_Command[1] = u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO217FS1, s16_msg_id);
   }   
   else// read operation
   {
      p_fieldbus_bg_tx_data[0]   = (manu_spec_s32_Servo_Sense_Write_Command[1] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (manu_spec_s32_Servo_Sense_Write_Command[1] >> 16);       //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO217FS1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalServoSenseWriteValueCommandBg
// Description:
//          This function is called in response object 0x217F sub index 2from Fieldbus
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalServoSenseWriteValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   long s32_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {       
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s64_Execution_Parameter[0] = (long long)manu_spec_s32_Servo_Sense_Write_Command[1];     
      s64_Execution_Parameter[1] = (long long)s32_tmp;
      s16_Number_Of_Parameters = 2;

      ret_val = SalSrvSnsWriteAddr(drive);
      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO217FS2, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&s32_tmp);
         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }
      numOfCalls = 0;
      ret_val = FalInitDownloadRespond((int)SDO217FS2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_s32_Servo_Sense_Write_Command[2];
      ret_val = FalInitUploadRespond((int)SDO217FS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalServoSenseSendCmdCommandBg
// Description:
//          This function is called in response object 0x2180 sub index 2from Fieldbus
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalServoSenseSendCmdCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {       
      u16_tmp = (p_data[0] & 0xFFFF);
      s64_Execution_Parameter[0] = (long long)u16_tmp;
      s16_Number_Of_Parameters = 1;
      ret_val = SalSrvSnsSendCmd(drive);

      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO2180S0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);
         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }
      numOfCalls = 0;
      ret_val = FalInitDownloadRespond((int)SDO2180S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_s32_Servo_Sense_Write_Command[2];
      ret_val = FalInitUploadRespond((int)SDO2180S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalPhaseFindCommandBg ?
// Description:
//          This function is called in response object 0x20A4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPhaseFindCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      if((p_data[0] & 0x00FF) != 1)
      {
         return VALUE_IS_NOT_ALLOWED;
      }

      //ITAI - remove blocking of phase find in the sync modes
     //do not allow phasefind in the sync modes
/*     if((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
        (p402_modes_of_operation == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
        (p402_modes_of_operation == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
      {
         return INVALID_OPMODE;
      }
*/

      ret_val = PhaseFindCommand(drive);

      ret_val = FalInitDownloadRespond((int)SDO20A4S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Phase_Find_command = 0;     // initialize the value
      p_fieldbus_bg_tx_data[0] = manu_spec_Phase_Find_command;
      ret_val = FalInitUploadRespond((int)SDO20A4S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name:FalSfbStartCalibrationCommandBg
// Description:
//          This function is called in response object 0x2151 from Fieldbus.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbStartCalibrationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      if(p_data[0] != 1)
      {
         return VALUE_IS_NOT_ALLOWED;
      }

      ret_val = SalSFBStartCalibrationProcess(drive);

      ret_val = FalInitDownloadRespond((int)SDO2151S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Sfb_Start_Calibration;
      ret_val = FalInitUploadRespond((int)SDO2151S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPhaseFindCurrentCommandBg
// Description:
//          This function is called in response object 0x20A7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPhaseFindCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
// long long s64_tmp = 0LL:
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s32_tmp = (long)(*(float*)(&s32_tmp) * 1000.0);

/*    //convert to internal FB current units
      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      //convert to user current units
      s32_tmp = (long)MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);*/

      ret_val = PhaseFindCurrentCommand((long long)s32_tmp, drive);
      if(ret_val != FAL_SUCCESS)
         return ret_val;

      ret_val = FalInitDownloadRespond((int)SDO20A7S0, s16_msg_id);
   }
   else// read operation
   {
/*    // convert to internal units // TBD!!!
      s64_tmp = MultS64ByFixS64ToS64((long long)BGVAR(s32_PhaseFind_Current),
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      //convert to user FB current units
      manu_spec_Phase_Find_Current_command = (long)MultS64ByFixS64ToS64((long long)s64_tmp,
                                 BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                 BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Phase_Find_Current_command & 0xFFFF);                               //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Phase_Find_Current_command >> 16);                                  //set 16 MSBits*/

      manu_spec_Phase_Find_Current_command = (float)BGVAR(s32_PhaseFind_Current) * 0.001;
      p_fieldbus_bg_tx_data[0] = ((*(long*)&manu_spec_Phase_Find_Current_command) & 0xFFFF);                  //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = ((*(long*)&manu_spec_Phase_Find_Current_command) >> 16);                     //set 16 MSBits
      ret_val = FalInitUploadRespond((int)SDO20A7S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPRBTypeCommandBg
// Description:
//          This function is called in response object 0x20AF index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPRBTypeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      VAR(AX0_u16_PRB_Type)=(unsigned int)(*p_data);
      ret_val = FalInitDownloadRespond((int)SDO20AFS1, s16_msg_id);

   }
   else// read operation
   {
      manu_spec_Prb_Param_command.PRB_Type = VAR(AX0_u16_PRB_Type);
      p_fieldbus_bg_tx_data[0] = manu_spec_Prb_Param_command.PRB_Type;
      ret_val = FalInitUploadRespond((int)SDO20AFS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPrbIAMPCommandBg
// Description:
//          This function is called in response object 0x20AF index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPrbIAMPCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      //convert to internal FB current units
      s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      //convert to user current units
      s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

      //Loading the Current noise amplitude
      if (!((s32_tmp <= BGVAR(s32_Ilim_User)) && ((long)s32_tmp >= 0)))
      {
         return (VALUE_OUT_OF_RANGE);
      }
      s64_tmp = MultS64ByFixS64ToS64((long)s32_tmp,
                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      VAR(AX0_s16_I_AMP) = s64_tmp & 0xffff;

      ret_val = FalInitDownloadRespond((int)SDO20AFS2, s16_msg_id);
   }
   else// read operation
   {
      s32_tmp = (long)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_I_AMP),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

      manu_spec_Prb_Param_command.PRB_I_AMP = s32_tmp;

      p_fieldbus_bg_tx_data[0] = manu_spec_Prb_Param_command.PRB_I_AMP & 0xFFFF;             //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = manu_spec_Prb_Param_command.PRB_I_AMP >> 16;                //set 16 MSBits

      ret_val = FalInitUploadRespond((int)SDO20AFS2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPRBVAmpCommandBg
// Description:
//          This function is called in response object 0x20AF index 3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPRBVAmpCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      //convert to internal FB velocity out of loop units
      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      //Loading the Velocity noise amplitude
      if (s64_tmp > (long long)BGVAR(s32_V_Lim_Design))
         return (VALUE_TOO_HIGH);

      if (s64_tmp < 0)
         return (VALUE_TOO_LOW);

      BGVAR(u32_PRB_V_Amp) = (unsigned long)s64_tmp;

      //convert to internal FB velocity in of loop units
      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      VAR(AX0_s16_V_AMP)=(int)s64_tmp & 0xffff;

      ret_val = FalInitDownloadRespond((int)SDO20AFS3, s16_msg_id);

   }
   else// read operation
   {
      //convert to user FB velocity out of loop units
      s32_tmp = (long)MultS64ByFixS64ToS64((long long)BGVAR(u32_PRB_V_Amp),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      manu_spec_Prb_Param_command.PRB_V_Amp = s32_tmp;

      p_fieldbus_bg_tx_data[0] = manu_spec_Prb_Param_command.PRB_V_Amp & 0xFFFF;        //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = manu_spec_Prb_Param_command.PRB_V_Amp >> 16;            //set 16 MSBits

      ret_val = FalInitUploadRespond((int)SDO20AFS3, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPrbParamWriteCommandBg
// Description:
//          This function is called in response object 0x20AF index 4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPRBCounterPeriodCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      //Loading the time interval= counter*31.25u[sec]
        VAR(AX0_u16_PRB_Counter_Period) = (unsigned int)(*p_data);

      ret_val = FalInitDownloadRespond((int)SDO20AFS4, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Prb_Param_command.PRB_Counter_Period = VAR(AX0_u16_PRB_Counter_Period);
      p_fieldbus_bg_tx_data[0] = manu_spec_Prb_Param_command.PRB_Counter_Period;
      ret_val = FalInitUploadRespond((int)SDO20AFS4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPRBConfigCommandBg
// Description:
//          This function is called in response object 0x20AF index 5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPRBConfigCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      if (p_data[0] != 1)
      {
         return VALUE_IS_NOT_ALLOWED;
      }

      manu_spec_Prb_Param_command.PRB_Config = 0; // initialize the variable

      ret_val= SalPRBFreqCommand((long long)BGVAR(u32_PRB_Freq), drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20AFS5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Prb_Param_command.PRB_Config;
      ret_val = FalInitUploadRespond((int)SDO20AFS5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalRelayCommandBg
// Description:
//          This function is called in response object 0x20B8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRelayCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
     p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20B8S0, s16_msg_id);
   }
   else// read operation
   {
      ret_val = SalRelayCommand((long long *)&s64_tmp);
     if(ret_val != FAL_SUCCESS)
     {
        manu_spec_Relay_command = 0;
        return ret_val;
     }

     manu_spec_Relay_command = (unsigned int)s64_tmp;
      p_fieldbus_bg_tx_data[0] = manu_spec_Relay_command;
      ret_val = FalInitUploadRespond((int)SDO20B8S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalSaveToFlashCommandBg
// Description:
//          This function is called in response object 0x1010 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions: nitsan:save using standard 0x1010 (instaed of 0x20BD)
//            make SDO non blocking to prevent sdo timeout (consider add manufacturer specific object for save/load status)
//**********************************************************
int FalSaveToFlashCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
/*
   static int numOfCalls = 0;
   unsigned char u8_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      if(numOfCalls == 0)
      {
         if(u8_tmp != 0x01)
            return VALUE_IS_NOT_ALLOWED;
      }

      ret_val = SalSaveToFlash(drive);
      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the save command execution has not been finished yet
         //request to handle the command again:
         InitDownloadRequest((int)SDO20BDS0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u8_tmp);
         numOfCalls++;
         return(FAL_SUCCESS);
      }
      numOfCalls = 0;
      manu_spec_Save_To_Flash_command = 0; // initialize the value
      ret_val = FalInitDownloadRespond((int)SDO20BDS0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Save_To_Flash_command;
      ret_val = FalInitUploadRespond((int)SDO20BDS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);

   */

   unsigned long s32_tmp = 0;
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       s32_tmp <<= 16;
       s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;


       if(s32_tmp != SAVE_SIGNATURE)
       {
            manu_spec_nonvolatile_command_stat = VALUE_IS_NOT_ALLOWED;
            return VALUE_IS_NOT_ALLOWED;
       }

       if (s16_CAN_EEprom_Operation_Type)
       {
            manu_spec_nonvolatile_command_stat = OTHER_PROCEDURE_RUNNING;
            return OTHER_PROCEDURE_RUNNING;
       }

       // set flag to allow operation run in background and update status to not finished
       manu_spec_nonvolatile_command_stat = SAL_NOT_FINISHED;
       s16_CAN_EEprom_Operation_Type = 1; // 1 for save, 2 for load
       ret_val = FalInitDownloadRespond((int)SDO1010S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = 0;
       p_fieldbus_bg_tx_data[1] = 0;
       ret_val = FalInitUploadRespond((int)SDO1010S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}



//**********************************************************
// Function Name: FalLoadFromFlashCommandBg
// Description:
//          This function is called in response object 0x20BD from Fieldbus (save/load status).
//          since save/load are time comsuming operations, SDOs are non blocking (not standard) and executed in background,
//          this object holds the status of the operation
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalEepromOperationStatus(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation != 1)  // read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_nonvolatile_command_stat;
       ret_val = FalInitUploadRespond(SDO20BDS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxm28VelocityWindowCommandBg
// Description:
//          This function is called in response object 0x4328 from Fieldbus.
//          It is the same as ds402 velocity window, but only u32 instead of u16.
//          No p-param is required to this canopen object according to Alexander Badura request, regarding IPR 791
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28VelocityWindowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      // we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(s32_Velocity_Window)  = (long)MultS64ByFixS64ToS64((long long)u32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO4328S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (u32_Lxm28_Velocity_Window & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (u32_Lxm28_Velocity_Window >> 16);        //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4328S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxm28VelocityThresholdCommandBg
// Description:
//          This function is called in response object 0x4329 from Fieldbus.
//          It is the same as ds402 velocity window, but only u32 instead of u16.
//          No p-param is required to this canopen object according to Alexander Badura request, regarding IPR 791
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28VelocityThresholdCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      // we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(s32_Velocity_Threshold)  = (long)MultS64ByFixS64ToS64((long long)u32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO4329S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (u32_Lxm28_Velocity_Threshold & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (u32_Lxm28_Velocity_Threshold >> 16);        //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4329S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxm28SdoAbortCodeCommandBg
// Description:
//          This function is called in response object 0x4FA6 from Fieldbus.
//          It holds the internal drive error code for an SDO request that returned general error (0x08000000).
//          The reason for this object is that no manufacturer specific codes are allowed as SDO abort code
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28SdoAbortCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;   // for compilation warning

   if(u16_operation == 1)// write operation
   {
      ret_val = NOT_PROGRAMMABLE;
   }
   else  // read operation
   {
      // for lxm28 the error codes should be OR with 0x1000 (as defined in error list SE_errcodes_1_30_4.ini).
      if (manu_spec_u32_Drive_Error_Code != 0UL)
         manu_spec_u32_Drive_Error_Code |= 0x1000UL;

      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_Drive_Error_Code & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_Drive_Error_Code >> 16);        //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4FA6S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxm28SdoAbortCodeCommandBg
// Description:
//          This function is called in response object 0x216C from Fieldbus.
//          It holds the internal drive error code for an SDO request that returned general error (0x08000000).
//          The reason for this object is that no manufacturer specific codes are allowed as SDO abort code
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalSdoAbortCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;   // for compilation warning

   if(u16_operation != 1)// read operation
   {
      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_Drive_Error_Code & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_Drive_Error_Code >> 16);        //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO216CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalLxmAccessLockCommandBg
// Description:
//          This function is called in response object 0x4A25 from Fieldbus (access lock).
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmAccessLockCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      // we need to take data int by int to prevent bad data due to odd address
      u16_tmp = (unsigned int)p_data[0];

      if (u16_tmp > 1) return VALUE_TOO_HIGH;

      BGVAR(u8_Lexium_Access_Rights_Locked) = u16_tmp;
      ret_val = FalInitDownloadRespond((int)SDO4A25S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = BGVAR(u8_Lexium_Access_Rights_Locked);
      ret_val = FalInitUploadRespond(SDO4A25S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxmEepromOperationStatus
// Description:
//          This function is called in response object 0x4FA3 from Fieldbus (save/load status).
//          since save/load are time comsuming operations, SDOs are non blocking (not standard) and executed in background,
//          this object holds the status of the operation. this is for lxm drive
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmEepromOperationStatus(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation != 1)  // read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_nonvolatile_command_stat;
       ret_val = FalInitUploadRespond(SDO4FA3S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxmCommandedVelocityCommandBg
// Description:
//          This function is called in response object 0x4FA4 from Fieldbus (Commanded velocity).
//          Schnider requested an object that will return vcmd for all opmodes.
//          this object will return VCMD or PTPVCMD according to actual opmode
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmCommandedVelocityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_velocity_command = 0;

   // AXIS_OFF;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation != 1)  // read operation
   {
      if (VAR(AX0_s16_Opmode) == 0 || VAR(AX0_s16_Opmode) == 1)
      {
         // velocity mode (serial or analog) - take vcmd
         s32_velocity_command = (long)MultS64ByFixS64ToS64((long long)(VAR(AX0_s16_Vel_Var_Ref_0)),
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);
      }
      else if (VAR(AX0_s16_Opmode) == 8 || VAR(AX0_s16_Opmode) == 4)
      {
         // position mode or gear mode - take ptpvcmd
         s32_velocity_command = (long)MultS64ByFixS64ToS64((long long)VAR(AX0_s32_Pos_Vcmd),
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);
      }


      manu_spec_s32_lxm_vcmd = s32_velocity_command;
      p_fieldbus_bg_tx_data[0] = (manu_spec_s32_lxm_vcmd & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_s32_lxm_vcmd >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4FA4S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxmVThresholdObjectCommandBg
// Description:
//          This function is called in response object 0x4FA0 sub 6 from Fieldbus (velocity threshold for drive profile lexium info bit).
//          this object is in canopen velocity units
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmVThresholdObjectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp, u32_velocity_outLoop;
   int drive = 0;

   // read only object
   if(u16_operation == 1)  // write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      u32_velocity_outLoop = (unsigned long)MultS64ByFixS64ToS64((long long)(u32_tmp),
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // bug 3662: limit velocity according to vmax (object 607f)
      if (u32_velocity_outLoop > (unsigned long)BGVAR(s32_V_Lim_Design))
      {
         return VALUE_TOO_HIGH;
      }

      // convert into out_loop vel units
      BGVAR(u32_Lxm_Vel_Threshold_Can_Obj) = (unsigned long)MultS64ByFixS64ToS64((long long)u32_tmp,
                       BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                       BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      CalcVelThresholdInLoop(drive);

      ret_val = FalInitDownloadRespond((int)SDO4FA0S6, s16_msg_id);
   }
   else  // read operation
   {
      p_fieldbus_bg_tx_data[0] = (manu_spec_Lxm_DPL_Ctrl.MON_V_Threshold & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Lxm_DPL_Ctrl.MON_V_Threshold >> 16);        //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4FA0S6, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalLxmIThresholdObjectCommandBg
// Description:
//          This function is called in response object 0x4FA0 sub 7 from Fieldbus (current threshold for drive profile lexium info bit).
//          this object is in canopen velocity units
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmIThresholdObjectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_tmp_torque = 0;

   if(u16_operation == 1)// write operation
   {

      s32_tmp_torque = (long)MultS64ByFixS64ToS64((long long)(*(int*)p_data),
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      BGVAR(u32_Lxm_Current_Threshold_Can_Obj) = s32_tmp_torque;
      ret_val = FalInitDownloadRespond((int)SDO4FA0S7, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Lxm_DPL_Ctrl.MON_I_Threshold;
      ret_val = FalInitUploadRespond(SDO4FA0S7, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }


   return (ret_val);
}

//**********************************************************
// Function Name: FalLxm28JogMethod
// Description:
//          This function is called in response object 0x4453 sub0 from Fieldbus (jog mode method).
//          range is 0: continious, 1: step
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28JogMethod(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation == 1)  // write operation
   {

      u16_tmp = (unsigned int)p_data[0];

      if (u16_tmp > 1) return VALUE_TOO_HIGH;

      BGVAR(u16_P4_83_Jog_Method) = u16_tmp;

      //BGVAR(s32_Fb_Gear_In) = s32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO4453S0, s16_msg_id);
   }
   else  // read operation
   {
      manu_spec_u16_P4_83_Jog_Method = BGVAR(u16_P4_83_Jog_Method);
      p_fieldbus_bg_tx_data[0] = (manu_spec_u16_P4_83_Jog_Method); //set 16 LSBits

      ret_val = FalInitUploadRespond(SDO4453S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}




//**********************************************************
// Function Name: FalLxm28JogFast
// Description:
//          This function is called in response object 0x4450 sub0 from Fieldbus (jog mode fast speed).
//          units are can velocity units.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28JogFast(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation == 1)  // write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop) = (long)MultS64ByFixS64ToS64((long long)u32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO4450S0, s16_msg_id);
   }
   else  // read operation
   {
      // fix bug 3866 (conversion to user units was missing)
      manu_spec_u32_P4_80_Jog_Fast =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_P4_80_Jog_Fast & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_P4_80_Jog_Fast >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4450S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalLxm28JogSlow
// Description:
//          This function is called in response object 0x4454 sub0 from Fieldbus (jog mode slow speed).
//          units are can velocity units.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxm28JogSlow(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp;

   p_data += 0; //For Remark Avoidance

   // read only object
   if(u16_operation == 1)  // write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(s32_P4_84_Jog_SpeedSlow_Out_Loop) = (long)MultS64ByFixS64ToS64((long long)u32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO4454S0, s16_msg_id);
   }
   else  // read operation
   {
      // fix bug 3866 (conversion to user units was missing)
      manu_spec_u32_P4_84_Jog_Slow =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(s32_P4_84_Jog_SpeedSlow_Out_Loop)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_P4_84_Jog_Slow & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_P4_84_Jog_Slow >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4454S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalLxmGearInCommandBg
// Description:
//          This function is called in response object 0x4FA5 sub1 from Fieldbus (gear in).
//          This object is for changing gearin without changing PUU.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmGearInCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int ret_val = FAL_SUCCESS;
   long s32_tmp;

   p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)  // write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      if (s32_tmp > 536870911) return VALUE_TOO_HIGH;
      if (s32_tmp < 1) return VALUE_TOO_LOW;

      ret_val = SalGearInCommand(s32_tmp, drive);
      if (ret_val != SAL_SUCCESS)
      {
         return ret_val;
      }

      BGVAR(s32_Fb_Gear_In) = s32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO4FA5S1, s16_msg_id);
   }
   else  // read operation
   {
      manu_spec_s32_Lxm_Gear_Arr[1] = BGVAR(s32_Fb_Gear_In);
      p_fieldbus_bg_tx_data[0] = (BGVAR(s32_Fb_Gear_In) & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (BGVAR(s32_Fb_Gear_In) >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4FA5S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//***********************************************************
// Function Name: FalLxmGearInCommandRt
// Description:
//           This function is called in response object 0x4FA5 sub 1 (gear in) from Fieldbus RPDO.
//           actual gear change is signaled here and done in background
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalLxmGearInCommandRt, "ramcan_2");
int  FalLxmGearInCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long s32_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   if (s32_tmp > 536870911) return VALUE_TOO_HIGH;
   if (s32_tmp < 1) return VALUE_TOO_LOW;

   BGVAR(s32_CAN_Rpdo_GearIn) = s32_tmp;
   BGVAR(s16_CAN_Flags) |= CANOPEN_GEAR_IN_CHANGED_MASK;
   return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalLxmGearOutCommandBg
// Description:
//          This function is called in response object 0x4FA5 sub2 from Fieldbus (gear out).
//          This object is for changing gearin without changing PUU.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLxmGearOutCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int ret_val = FAL_SUCCESS;
   long s32_tmp;

   p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)  // write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      if (s32_tmp > 2147483647) return VALUE_TOO_HIGH;
      if (s32_tmp < 1) return VALUE_TOO_LOW;

      ret_val = SalGearOutCommand(s32_tmp, drive);
      if (ret_val != SAL_SUCCESS)
      {
         return ret_val;
      }

      BGVAR(s32_Fb_Gear_Out) = s32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO4FA5S2, s16_msg_id);
   }
   else  // read operation
   {
      manu_spec_s32_Lxm_Gear_Arr[2] = BGVAR(s32_Fb_Gear_Out);
      p_fieldbus_bg_tx_data[0] = (BGVAR(s32_Fb_Gear_Out) & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (BGVAR(s32_Fb_Gear_Out) >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO4FA5S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//***********************************************************
// Function Name: FalLxmGearOutCommandRt
// Description:
//           This function is called in response object 0x4FA5 sub 2 (gear outn) from Fieldbus RPDO.
//           actual gear change is signaled here and done in background
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalLxmGearOutCommandRt, "ramcan_2");
int  FalLxmGearOutCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long s32_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   if (s32_tmp > 2147483647) return VALUE_TOO_HIGH;
   if (s32_tmp < 1) return VALUE_TOO_LOW;

   BGVAR(s32_CAN_Rpdo_GearOut) = s32_tmp;
   BGVAR(s16_CAN_Flags) |= CANOPEN_GEAR_OUT_CHANGED_MASK;
   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalSrvSnsInfoSelectCommandBg
// Description:
//          This function is called in response object 0x214C sub_index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalSrvSnsInfoSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   u16_operation += 0;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO214CS1, s16_msg_id);
   }
   else// read operation
   {
      DomainHandler(drive,(void *)&manu_spec_Srv_Sns_Info_Command,0x214C,((int (*)(int ,int ))SrvSnsInfoCommand),PACKETS_TRANSFER_ACTIVATE);
      p_fieldbus_bg_tx_data[0] = manu_spec_Srv_Sns_Info_Command.StatusSelect;
      ret_val = FalInitUploadRespond((int)SDO214CS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//
//**********************************************************
// Function Name: FalSrvSnsInfoGrabCommandBg
// Description:
//          This function is called in response object 0x214C sub index 2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalSrvSnsInfoGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS,drive = 0;
   p_data += 0;
   u16_operation += 0;

   DomainHandler(drive,(void *)&manu_spec_Srv_Sns_Info_Command,0x214C,NULL,DOMAIN_DATA_UPLOAD);
   if (IS_EC_DRIVE)
      p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_Srv_Sns_Info_Command.Domain & 0xffff);

   ret_val = FalInitUploadRespond((int)(SDO214CS2), s16_msg_id, u16_Domain_Data_Length, (int*)p_fieldbus_bg_tx_data); //The length needs to be in Bytes
   return (ret_val);
}

//**********************************************************
// Function Name: FalLoadFromFlashCommandBg
// Description:
//          This function is called in response object 0x1011 from Fieldbus.
//          make SDO non blocking to prevent sdo timeout (consider add manufacturer specific object for save/load status)
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalLoadFromFlashCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long s32_tmp = 0;
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (long)p_data[1] & 0xFFFF;
       s32_tmp <<= 16;
       s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       // prevent load if drive is enabled
       if (Enabled(DRIVE_PARAM))
       {
            manu_spec_nonvolatile_command_stat = DRIVE_ACTIVE;
            return DRIVE_ACTIVE;
       }

       if(s32_tmp != LOAD_SIGNATURE)
       {
            manu_spec_nonvolatile_command_stat = VALUE_IS_NOT_ALLOWED;
            return VALUE_IS_NOT_ALLOWED;
       }

       if (s16_CAN_EEprom_Operation_Type)
       {
            manu_spec_nonvolatile_command_stat = OTHER_PROCEDURE_RUNNING;
            return OTHER_PROCEDURE_RUNNING;
       }

       // set flag to allow operation run in background and status to not finished
       manu_spec_nonvolatile_command_stat = SAL_NOT_FINISHED;
       s16_CAN_EEprom_Operation_Type = 2; // 1 for save, 2 for load
       ret_val = FalInitDownloadRespond((int)SDO1011S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = 0;
       p_fieldbus_bg_tx_data[1] = 0;
       ret_val = FalInitUploadRespond((int)SDO1011S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}


//**********************************************************
// Function Name: FalSininitCommandBg
// Description:
//          This function is called in response object 0x20BE from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalSininitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0];

      if(u16_tmp != 1)
         return VALUE_IS_NOT_ALLOWED;

      ret_val = SininitCommand(drive);
      if(ret_val == FAL_SUCCESS)
      {
         manu_spec_Sin_init_command = 0;// initialize the variable
         ret_val = FalInitDownloadRespond((int)SDO20BES0, s16_msg_id);
      }
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Sin_init_command;
      ret_val = FalInitUploadRespond(SDO20BES0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalSinparamCommandBg
// Description:
//          This function is called in response object 0x20C1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalSinparamCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, i=0;
   // AXIS_OFF;
   p_data += 0;
   if(u16_operation == 1)// write operation
      ret_val = FalInitDownloadRespond((int)SDO20C1S0, s16_msg_id);
   else// read operation
   {
      strcpy((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Sine_Offset), 5));
      strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);

      strncat((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Cosine_Offset), 4), 5);
      strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);

      strncat((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Sine_Gain_Fix), 4), 5);
      strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);

      strncat((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Sine_Gain_Shr), 4), 5);
      strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);

       if (LVAR(AX0_s32_Feedback_Ptr) == &SW_RESOLVER_FEEDBACK)
      {
           strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);
           strncat((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Swr2d_Fix), 4), 5);

           strncat((char*)&manu_spec_Sin_param_command[0], " ", 1);
           strncat((char*)&manu_spec_Sin_param_command[0], DecToAsciiHex((unsigned long)VAR(AX0_s16_Swr2d_Shr), 4), 5);
      }

      i = strlen((char*)&manu_spec_Sin_param_command[0]);

      if(i < FB_objects_size_array[SDO20C1S0]) // null terminate the object variable
         manu_spec_Sin_param_command[i] = 0;

      ret_val = FalInitUploadRespond((int)SDO20C1S0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalReadThermCommandBg
// Description:
//          This function is called in response object 0x20C7 from Fieldbus.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalReadThermCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   
   int drive = 0, ret_val = FAL_SUCCESS;
   long long s64_temperature;
   p_data+=0;

   if(u16_operation == 0)// read operation
   {
      ret_val = ReadThermCommand(&s64_temperature, drive);
      manu_spec_Read_Therm_command = (long)s64_temperature;
      p_fieldbus_bg_tx_data[0] = (manu_spec_Read_Therm_command & 0xFFFF);
      p_fieldbus_bg_tx_data[1] = (manu_spec_Read_Therm_command >> 16);
      ret_val = FalInitUploadRespond(SDO20C7S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}




//**********************************************************
// Function Name: FalTmTurnResetCommandBg
// Description:
//          This function is called in response object 0x20CB from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTmTurnResetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned char u8_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       u8_tmp = p_data[0] & 0x00FF;

       if(numOfCalls == 0)
       {
          if(u8_tmp != 1)
              return VALUE_IS_NOT_ALLOWED;
       }

      ret_val = SalTmTurnResetCommand(drive);
      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the Tamagawa Multi-Turn Reset command execution has not been finished yet
         //request to handle the command again:

         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO20CBS0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u8_tmp);
         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }
      numOfCalls = 0;
        manu_spec_Tm_Turn_Reset_command = 0; // initialize the variable
        ret_val = FalInitDownloadRespond((int)SDO20CBS0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Tm_Turn_Reset_command;
      ret_val = FalInitUploadRespond((int)SDO20CBS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalSkTurnResetCommandBg
// Description:
//          This function is called in response object 0x215A from Fieldbus.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int FalSkTurnResetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       u16_tmp = p_data[0] & 0xFFFF;

       if(numOfCalls == 0)
       {
          if(u16_tmp != 1)
              return VALUE_IS_NOT_ALLOWED;
       }

      ret_val = SalSkTurnResetCommand(drive);
      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the Sankyo Multi-Turn Reset command execution has not been finished yet
         //request to handle the command again:

         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO215AS0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);
         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }
      numOfCalls = 0;
        manu_spec_Tm_Turn_Reset_command = 0; // initialize the variable
        ret_val = FalInitDownloadRespond((int)SDO215AS0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Tm_Turn_Reset_command;
      ret_val = FalInitUploadRespond((int)SDO215AS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalProbe1CounterReadCommandBg
// Description:
//          This function is called in response object 0x2131 from Fieldbus.
//          This object provides how many events were captured by probe #1
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalProbe1CounterReadCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data+=0;
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2131S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Probe_Cntr = VAR(AX0_u16_TProbe_Cntr_Arr[0]) + VAR(AX0_u16_TProbe_Cntr_Arr[1]) ;
       p_fieldbus_bg_tx_data[0] = manu_spec_Probe_Cntr;
       ret_val = FalInitUploadRespond(SDO2131S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalProbe2CounterReadCommandBg
// Description:
//          This function is called in response object 0x2183 from Fieldbus.
//          This object provides how many events were captured by probe #2
//          The function is the equivalent of the "FalProbe1CounterReadCommandBg" function,
//          implemented by Gil.
//
// Athor: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalProbe2CounterReadCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data+=0;
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2183S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Probe_Cntr_2 = VAR(AX0_u16_TProbe_Cntr_Arr[2]) + VAR(AX0_u16_TProbe_Cntr_Arr[3]) ;
       p_fieldbus_bg_tx_data[0] = manu_spec_Probe_Cntr_2;
       ret_val = FalInitUploadRespond(SDO2183S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTProbe2LevelPeriodBg
// Description:
//          This function is called in response object 0x216B from Fieldbus.
//          This object handles touch probe 2 period for stabilization
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTProbe2LevelPeriodBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      BGVAR(u16_Probe_Lvl_Prd[1])= p_data[0] & 0xFFFF;
      *(int *)FPGA_CAPTURE3_FILTER_REG_ADD = BGVAR(u16_Probe_Lvl_Prd[1])*1875;
      ret_val = FalInitDownloadRespond((int)SDO216BS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Probe_2_Lvl_Prd = BGVAR(u16_Probe_Lvl_Prd[1]);
       p_fieldbus_bg_tx_data[0] = manu_spec_Probe_2_Lvl_Prd;

       ret_val = FalInitUploadRespond(SDO216BS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalTProbe1PosEdgeCounterBg
// Description:
//          This function is called in response object 0x60D5 from Fieldbus.
//          This object shall provide the probe #1 positive edge counter
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTProbe1PosEdgeCounterBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO60D5S0, s16_msg_id);
   }
   else// read operation
   {
       p402_touch_probe_1_positive_edge_counter = BGVAR(Probe_1_Rise_Record).u16_edge_counter;
       p_fieldbus_bg_tx_data[0] = p402_touch_probe_1_positive_edge_counter;

       ret_val = FalInitUploadRespond(SDO60D5S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//***********************************************************
// Function Name: FalTProbe1PosEdgeCounterRt
//          This function is called in response to PDO object 0x60D5 from Fieldbus.
//          This object shall provide the probe #1 positive edge counter
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTProbe1PosEdgeCounterRt , "ramcan_2");
int  FalTProbe1PosEdgeCounterRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_touch_probe_1_positive_edge_counter = p_data[0];
   return (FAL_SUCCESS);
}
//**********************************************************
// Function Name: FalTProbe1NegEdgeCounterBg
// Description:
//          This function is called in response object 0x60D6 from Fieldbus.
//          This object shall provide the probe #1 negative edge counter
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTProbe1NegEdgeCounterBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO60D6S0, s16_msg_id);
   }
   else// read operation
   {
       p402_touch_probe_1_negative_edge_counter = BGVAR(Probe_1_Fall_Record).u16_edge_counter;
       p_fieldbus_bg_tx_data[0] = p402_touch_probe_1_negative_edge_counter;
       ret_val = FalInitUploadRespond(SDO60D6S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//***********************************************************
// Function Name: FalTProbe1NegEdgeCounterRt
//          This function is called in response to PDO object 0x60D6 from Fieldbus.
//          This object shall provide the probe #1 negative edge counter
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTProbe1NegEdgeCounterRt , "ramcan_2");
int  FalTProbe1NegEdgeCounterRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_touch_probe_1_negative_edge_counter = p_data[0];
   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalTProbe2PosEdgeCounterBg
// Description:
//          This function is called in response object 0x60D7 from Fieldbus.
//          This object shall provide the probe #2 positive edge counter
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTProbe2PosEdgeCounterBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO60D7S0, s16_msg_id);
   }
   else// read operation
   {
       p402_touch_probe_2_positive_edge_counter = BGVAR(Probe_2_Rise_Record).u16_edge_counter;
       p_fieldbus_bg_tx_data[0] = p402_touch_probe_2_positive_edge_counter;
       ret_val = FalInitUploadRespond(SDO60D7S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//***********************************************************
// Function Name: FalTProbe2PosEdgeCounterRt
//          This function is called in response to PDO object 0x60D7 from Fieldbus.
//          This object shall provide the probe #2 positive edge counter
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTProbe2PosEdgeCounterRt , "ramcan_2");
int  FalTProbe2PosEdgeCounterRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_touch_probe_2_positive_edge_counter = p_data[0];
   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalTProbe2NegEdgeCounterBg
// Description:
//          This function is called in response object 0x60D8 from Fieldbus.
//          This object shall provide the probe #2 negative edge counter
//
// Athor: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTProbe2NegEdgeCounterBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO60D8S0, s16_msg_id);
   }
   else// read operation
   {
       p402_touch_probe_2_negative_edge_counter = BGVAR(Probe_2_Fall_Record).u16_edge_counter;
       p_fieldbus_bg_tx_data[0] = p402_touch_probe_2_negative_edge_counter;
       ret_val = FalInitUploadRespond(SDO60D8S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//***********************************************************
// Function Name: FalTProbe2NegEdgeCounterRt
//          This function is called in response to PDO object 0x60D8 from Fieldbus.
//          This object shall provide the probe #2 negative edge counter
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTProbe2NegEdgeCounterRt , "ramcan_2");
int  FalTProbe2NegEdgeCounterRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_touch_probe_2_negative_edge_counter = p_data[0];
   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalTrunCommandBg
// Description:
//          This function is called in response object 0x20CC from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTrunCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20CCS0, s16_msg_id);
   }
   else// read operation
   {
      strcpy((char*)&manu_spec_Trun_command[0], UnsignedIntegerToAscii(u16_RunTimeHours));
      strncat((char*)&manu_spec_Trun_command[0], ":", 1);

      if (u16_RunTimeMins < 10)
         strncat((char*)&manu_spec_Trun_command[0], "0",1);
      strncat((char*)&manu_spec_Trun_command[0], UnsignedIntegerToAscii(u16_RunTimeMins), strlen(UnsignedIntegerToAscii(u16_RunTimeMins)));
      strncat((char*)&manu_spec_Trun_command[0], ":",1);

      strncat((char*)&manu_spec_Trun_command[0], UnsignedIntegerToAscii(u16_RunTimeSecs), strlen(UnsignedIntegerToAscii(u16_RunTimeSecs)));
      if (u16_RunTimeSecs < 10)
         strncat((char*)&manu_spec_Trun_command[0], "0",1);

      ret_val = FalInitUploadRespond((int)SDO20CCS0, s16_msg_id, (int)strlen((char*)&manu_spec_Trun_command[0]), (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}



//**********************************************************
// Function Name: FalHeartbeatToleranceCommandBg
// Description:
//          This function is called in response object 0x2159 from Fieldbus.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHeartbeatToleranceCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      if (u16_tmp > 100) return VALUE_TOO_HIGH;

     manu_spec_Heartbeat_Tolerance_command = p_data[0];
     setHeartBeatConsumerTime((int)0, (unsigned long)p301_hb_consumer[1], (unsigned int)1, CO_TRUE CO_COMMA_LINE_PARA);
      ret_val = FalInitDownloadRespond((int)SDO2159S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Heartbeat_Tolerance_command;
      ret_val = FalInitUploadRespond((int)SDO2159S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVelDesignCommandBg
// Description:
//          This function is called in response object 0x from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVelDesignCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   LIST_ELEMENT_T* pCurObj;
   int i, objSize, len;


   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20D5S0, s16_msg_id);
   }
   else// read operation
   {
       //update string
       ret_val = VelDesignCommand(0);
      if(ret_val == SAL_NOT_FINISHED)
         return ret_val;

       // Get object
       pCurObj = searchObj(0x20D5 CO_COMMA_LINE_PARA_DECL);
       objSize = getObjSize(pCurObj, 0);

       // prepare string to send via canopen
       len = strlen((char*)manu_spec_Vel_Design_command);
       for (i=0; i<len && i<objSize; i++)
       {
            p_fieldbus_bg_tx_data[i] = manu_spec_Vel_Design_command[i];
       }

       // null terminate the object variable if shorter than size of manu_spec_Ver_command
       if (i < objSize)
            manu_spec_Vel_Design_command[i] = 0;

       // update the length
       FB_objects_size_array[SDO20D5S0] = i;

       ret_val = FalInitUploadRespond((int)SDO20D5S0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVerCommandBg
// Description:
//          This function is called in response object 0x20D7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVerCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
    LIST_ELEMENT_T* pCurObj;
       char buffFpga[128];
       char buffResident[128];
       int i, objSize, len;

   if(u16_operation == 1)// write operation
   {
     p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20D7S0, s16_msg_id);
   }
   else// read operation
   {
      // for now return firmware, fpga and resident only
       pCurObj = searchObj(0x20D7 CO_COMMA_LINE_PARA_DECL);
       objSize = getObjSize(pCurObj, 0);

       PrintFpgaVersion(buffFpga);
       GetResidentVersion(buffResident);

       // trim resident string (it has many spaces at the end).
       len = strlen(buffResident);
       for (i=len-1; i>=0; i--)
       {
            if (buffResident[i] == ' ')
            {
                 buffResident[i] = 0;
            }
            else
            {
                 break;
            }
       }

       // prepare string
       strcpy((char*)manu_spec_Ver_command, "FW: ");
       strcat((char*)manu_spec_Ver_command, p_s8_CDHD_Drive_Version);
       strcat((char*)manu_spec_Ver_command, "\r\nFPGA: ");
       strcat((char*)manu_spec_Ver_command, buffFpga);
       strcat((char*)manu_spec_Ver_command, "\r\nResident: ");
       strcat((char*)manu_spec_Ver_command, buffResident);


       // prepare string to send via canopen
       len = strlen((char*)manu_spec_Ver_command);
       for (i=0; i<len && i<objSize; i++)
       {
            p_fieldbus_bg_tx_data[i] = manu_spec_Ver_command[i];
       }

       // null terminate the object variable if shorter than size of manu_spec_Ver_command
       if (i < objSize)
            manu_spec_Motor_Name_command[i] = 0;

       // update the length
       FB_objects_size_array[SDO20D7S0] = i;

       ret_val = FalInitUploadRespond((int)SDO20D7S0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVf1CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=0, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val = VALUE_TOO_LOW;
      if ((long)p_data[0] > 32767LL) ret_val = VALUE_TOO_HIGH;

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D8S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[1];
      ret_val = FalInitUploadRespond((int)SDO20D8S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVf2CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
/***********************************************************/
int FalVf2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i = 1, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20D8S2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[2];
      ret_val = FalInitUploadRespond((int)SDO20D8S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVf3CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=2, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val = VALUE_TOO_LOW;
      if ((long)p_data[0] > 32767LL) ret_val = VALUE_TOO_HIGH;

      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20D8S3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[3];
      ret_val = FalInitUploadRespond((int)SDO20D8S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVf4CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=3, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D8S4, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[4];
      ret_val = FalInitUploadRespond((int)SDO20D8S4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVf5CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf5CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=4, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D8S5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[5];
      ret_val = FalInitUploadRespond((int)SDO20D8S5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVf6CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf6CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=5, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D8S6, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[6];
      ret_val = FalInitUploadRespond((int)SDO20D8S6, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVf7CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf7CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=6, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20D8S7, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vf_command[7];
      ret_val = FalInitUploadRespond((int)SDO20D8S7, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVf8CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D8S8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVf8CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
//**********************************************************
{
   unsigned int u16_temp[7];
   int drive = 0, ret_val = FAL_SUCCESS, s16_i;
// Configure the velocity loop - and if failed, restore previous parameters
   /* Accept new params */
   if(u16_operation == 1)// write operation
   {
      if (p_data[0] !=0)
     {
         for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
         {
            u16_temp[s16_i] = BGVAR(s16_Vf)[s16_i];
            BGVAR(s16_Vf)[s16_i] = (int)manu_spec_Vf_command[s16_i+1];
         }
         if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
         {
            for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
               BGVAR(s16_Vf)[s16_i] = u16_temp[s16_i];
            VelocityConfig(DRIVE_PARAM);
            ret_val =(VELOCITY_CONFIG_FAIL);
       }

          if(ret_val != FAL_SUCCESS)
            return ret_val;
     }
      ret_val = FalInitDownloadRespond((int)SDO20D8S8, s16_msg_id);
   }
   else// read operation
      ret_val = FalInitUploadRespond((int)SDO20D8S8, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (ret_val);

}
//**********************************************************
// Function Name: FalVfi1CommandBg
// Description:
//          This function is responsible to validate access to configure the object 20D9S1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=0, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[1];
      ret_val = FalInitUploadRespond((int)SDO20D9S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVfi2CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
/***********************************************************/
int FalVfi2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i = 1, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[2];
      ret_val = FalInitUploadRespond((int)SDO20D9S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVfi3CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=2, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[3];
      ret_val = FalInitUploadRespond((int)SDO20D9S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVfi4CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=3, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S4, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[4];
      ret_val = FalInitUploadRespond((int)SDO20D9S4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVfi5CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi5CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=4, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[5];
      ret_val = FalInitUploadRespond((int)SDO20D9S5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVfi6CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi6CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=5, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S6, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[6];
      ret_val = FalInitUploadRespond((int)SDO20D9S6, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVfi7CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi7CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_i=6, s16_l_limit;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20D9S7, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vfi_command[7];
      ret_val = FalInitUploadRespond((int)SDO20D9S7, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVfi8CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20D9S8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVfi8CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
//**********************************************************
{
   unsigned int u16_temp[7];
   int drive = 0, ret_val = FAL_SUCCESS, s16_i;
// Configure the velocity loop - and if failed, restore previous parameters
   /* Accept new params */
   if(u16_operation == 1)// write operation
   {
      if (p_data[0] !=0)
     {
         for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
         {
            u16_temp[s16_i] = BGVAR(s16_Vfi)[s16_i];
            BGVAR(s16_Vfi)[s16_i] = (int)manu_spec_Vfi_command[s16_i+1];
         }
         if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
         {
            for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
               BGVAR(s16_Vfi)[s16_i] = u16_temp[s16_i];
            VelocityConfig(DRIVE_PARAM);
            ret_val =(VELOCITY_CONFIG_FAIL);
       }
          if(ret_val != FAL_SUCCESS)
            return ret_val;
     }
      ret_val = FalInitDownloadRespond((int)SDO20D9S8, s16_msg_id);
   }
   else// read operation
      ret_val = FalInitUploadRespond((int)SDO20D9S8, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (ret_val);

}
//**********************************************************
// Function Name: FalVh1CommandBg 20D9
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************/

int FalVh1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;

   int ret_val = FAL_SUCCESS;
   int s16_i=0;
   //unsigned int u16_temp[12];
   //int drive = 0;
   if(u16_operation == 1)// write operation
   {

      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);
      
      //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[1];
      ret_val = FalInitUploadRespond((int)SDO20DAS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVh2CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
/***********************************************************/
int FalVh2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i = 1;
   

   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] =manu_spec_Vh_command[2];
      ret_val = FalInitUploadRespond((int)SDO20DAS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVh3CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=2;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[3];
      ret_val = FalInitUploadRespond((int)SDO20DAS3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalVh4CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=3;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS4, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[4];
      ret_val = FalInitUploadRespond((int)SDO20DAS4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh5CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh5CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=4;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[5];
      ret_val = FalInitUploadRespond((int)SDO20DAS5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh6CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh6CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=5;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS6, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[6];
      ret_val = FalInitUploadRespond((int)SDO20DAS6, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh7CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh7CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=6;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS7, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[7];
      ret_val = FalInitUploadRespond((int)SDO20DAS7, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh8CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh8CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=7;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS8, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[8];
      ret_val = FalInitUploadRespond((int)SDO20DAS8, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh9CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS9 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh9CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;

   int ret_val = FAL_SUCCESS;
   int s16_i=8;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DAS9, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[9];
      ret_val = FalInitUploadRespond((int)SDO20DAS9, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh10CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DASA from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh10CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=9;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DASA, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[10];
      ret_val = FalInitUploadRespond((int)SDO20DASA, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh11CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DASB from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh11CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i = 10;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DASB, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[11];
      ret_val = FalInitUploadRespond((int)SDO20DASB, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh12CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS12 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh12CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i = 11;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DASC, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vh_command[12];
      ret_val = FalInitUploadRespond((int)SDO20DASC, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVh13CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DAS13 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVh13CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_temp[12];
   int drive = 0, ret_val = FAL_SUCCESS, s16_i;
// Configure the velocity loop - and if failed, restore previous parameters
   /* Accept new params */
   if(u16_operation == 1)// write operation
   {
      if (p_data[0] !=0)
     {
         for (s16_i=0;s16_i<=11;++s16_i)
         {
            u16_temp[s16_i] = BGVAR(s16_Vh)[s16_i];
            BGVAR(s16_Vh)[s16_i] = (int)manu_spec_Vh_command[s16_i+1];
         }
         if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
         {
            for (s16_i=0; s16_i<=11; ++s16_i)
               BGVAR(s16_Vh)[s16_i] = u16_temp[s16_i];
            VelocityConfig(DRIVE_PARAM);
            ret_val =(VELOCITY_CONFIG_FAIL);
       }
             if(ret_val != FAL_SUCCESS)
               return ret_val;
    }
         ret_val = FalInitDownloadRespond((int)SDO20DASD, s16_msg_id);
   }
   else// read operation
      ret_val = FalInitUploadRespond((int)SDO20DASD, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}
//**********************************************************
// Function Name: FalVr1CommandBg 20D9
// Description:
//          This function validates access to configure the object 0x20DBS1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;

   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_i=0;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;
      
      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
      {
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
//         /*ret_val =*/ SalConfigCommand(drive);
         // Using direct call to DesignRoutine to avoid Feedback re-configuration when not needed
         DesignRoutine(0, drive);
         //if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO20DBS1, s16_msg_id);
     }
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[1];
      ret_val = FalInitUploadRespond((int)SDO20DBS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalVr2CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
/***********************************************************/
int FalVr2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i = 1;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[2];
      ret_val = FalInitUploadRespond((int)SDO20DBS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalVr3CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=2;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[3];
      ret_val = FalInitUploadRespond((int)SDO20DBS3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalVr4CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=3;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS4, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[4];
      ret_val = FalInitUploadRespond((int)SDO20DBS4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr5CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr5CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=4;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[5];
      ret_val = FalInitUploadRespond((int)SDO20DBS5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr6CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr6CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=5;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS6, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[6];
      ret_val = FalInitUploadRespond((int)SDO20DBS6, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr7CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr7CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;
   int ret_val = FAL_SUCCESS;
   int s16_i=6;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS7, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[7];
      ret_val = FalInitUploadRespond((int)SDO20DBS7, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr8CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr8CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=7;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20DBS8, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[8];
      ret_val = FalInitUploadRespond((int)SDO20DBS8, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr9CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS9 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr9CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = -32768;

   int ret_val = FAL_SUCCESS;
   int s16_i=8;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO20DBS9, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[9];
      ret_val = FalInitUploadRespond((int)SDO20DBS9, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr10CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS10 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr10CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_l_limit = 0;
   int ret_val = FAL_SUCCESS;
   int s16_i=9;
   if(u16_operation == 1)// write operation
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if ((long)p_data[0] < (long long)s16_l_limit) ret_val =(VALUE_TOO_LOW);
      if ((long)p_data[0] > 32767LL) ret_val =(VALUE_TOO_HIGH);

      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20DBSA, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Vr_command[10];
      ret_val = FalInitUploadRespond((int)SDO20DBSA, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalVr11CommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DBS11 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalVr11CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_temp[10];
   int drive = 0, ret_val = FAL_SUCCESS, s16_i;
// Configure the velocity loop - and if failed, restore previous parameters
   /* Accept new params */
   if(u16_operation == 1)// write operation
   {
      if (p_data[0] !=0)
     {
         for (s16_i=0; s16_i<=9; ++s16_i)
         {
            u16_temp[s16_i] = BGVAR(s16_Vr)[s16_i];
            BGVAR(s16_Vr)[s16_i] = (int)manu_spec_Vr_command[s16_i+1];
         }
         if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
         {
            for (s16_i=0; s16_i<=9; ++s16_i)
               BGVAR(s16_Vr)[s16_i] = u16_temp[s16_i];
            VelocityConfig(DRIVE_PARAM);
            ret_val =(VELOCITY_CONFIG_FAIL);
       }
             if(ret_val != FAL_SUCCESS)
               return ret_val;
    }
         ret_val = FalInitDownloadRespond((int)SDO20DBSB, s16_msg_id);
   }
   else// read operation
      ret_val = FalInitUploadRespond((int)SDO20DBSB, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}

//**********************************************************
// Function Name: FalWNSErrorCommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DC from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalWNSErrorCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   LIST_ELEMENT_T* pCurObj;
   int i, objSize, len;


   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20DCS0, s16_msg_id);
   }
   else// read operation
   {
       //update string
       WNSErrorCommand(0);

       // Get object
       pCurObj = searchObj(0x20DC CO_COMMA_LINE_PARA_DECL);
       objSize = getObjSize(pCurObj, 0);

       // prepare string to send via canopen
       len = strlen((char*)manu_spec_WNS_Error_command);
       for (i=0; i<len && i<objSize; i++)
       {
            p_fieldbus_bg_tx_data[i] = manu_spec_WNS_Error_command[i];
       }

       // null terminate the object variable if shorter than size of manu_spec_Ver_command
       if (i < objSize)
            manu_spec_WNS_Error_command[i] = 0;

       // update the length
       FB_objects_size_array[SDO20DCS0] = i;

       ret_val = FalInitUploadRespond((int)SDO20DCS0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalPrintCurrentWarningsCommandBg
// Description:
//          This function is responsible to validate access to configure the object 0x20DD from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPrintCurrentWarningsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   LIST_ELEMENT_T* pCurObj;
   int i, objSize, len;


   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO20DDS0, s16_msg_id);
   }
   else// read operation
   {
       //update string
      s64_Execution_Parameter[0] = 1;
       ret_val = PrintCurrentWarnings(0);
      if(ret_val == SAL_NOT_FINISHED)
         return ret_val;

       // Get object
       pCurObj = searchObj(0x20DD CO_COMMA_LINE_PARA_DECL);
       objSize = getObjSize(pCurObj, 0);

       // prepare string to send via canopen
       len = strlen((char*)manu_spec_Print_Current_Warnings_command);
       for (i=0; i<len && i<objSize; i++)
       {
            p_fieldbus_bg_tx_data[i] = manu_spec_Print_Current_Warnings_command[i];
       }

       // null terminate the object variable if shorter than size of manu_spec_Ver_command
       if (i < objSize)
            manu_spec_Print_Current_Warnings_command[i] = 0;

       // update the length
       FB_objects_size_array[SDO20DDS0] = i;

       ret_val = FalInitUploadRespond((int)SDO20DDS0, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalInModeInputIndexCommandBg
// Description:
//          This function is called in response object 0x20E0 index 1 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInModeInputIndexCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      if (!(u16_Supported_Dig_Inputs_Mask & (1 << (int)(u16_tmp - 1))))
         return (I_O_NOT_SUPPORTED);

      ret_val = FalInitDownloadRespond((int)SDO20E0S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Input_Mode_command[1];

      ret_val = FalInitUploadRespond(SDO20E0S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalInModeInputFunctionCommandBg
// Description:
//          This function is called in response object 0x20E0 index 2 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalInModeInputFunctionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, drive = 0;
   unsigned int i = 0, u16_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      if(u16_tmp == RESERVED_INP)
         return VALUE_IS_NOT_ALLOWED;

      if ((u16_tmp == HIGH_SPD_PUSLE_INP) && (manu_spec_Input_Mode_command[1] != 5))
         return FUNC_NOT_SUPPORTED_ON_IO;

      if ((u16_tmp == HIGH_SPD_DIR_INP) && (manu_spec_Input_Mode_command[1] != 6))
         return FUNC_NOT_SUPPORTED_ON_IO;

      // Set the same inmode --> do nothing
      if (VAR(AX0_u16_Input_Mode_Arr[manu_spec_Input_Mode_command[1]]) == u16_tmp)
      {
         ret_val = FalInitDownloadRespond((int)SDO20E0S2, s16_msg_id);
         return (ret_val);
      }

      // Check that function is not occupied (unless emergency stop or no_func)
      if (((u16_tmp != NO_FUNC_INP) && (u16_tmp != EMERGENCY_STOP_INP)) && (IsInFunctionalityConfigured(u16_tmp, drive)))
         return (FUNCTIONALITY_OCCUPIED);

      // Accept inmode
      i = VAR(AX0_u16_Input_Mode_Arr[manu_spec_Input_Mode_command[1]]); // Store prev function

      UpdateIndexFunc(drive,(int)u16_tmp, manu_spec_Input_Mode_command[1]);

      if (VAR(AX0_u16_Input_Mode_Arr[manu_spec_Input_Mode_command[1]]) == RESET_INP)
         s16_Clr_Flt_In = (int)(manu_spec_Input_Mode_command[1]); // this input associated with clr fault function

      if (i == RESET_INP)
         s16_Clr_Flt_In = -1; // no input is associated with clr fault function

      // Set P&D Mux on FPGA depending on I/O 5,6 inmodes
      PulseandDirectionFpgaMUX(drive);

      ret_val = FalInitDownloadRespond((int)SDO20E0S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Input_Mode_command[2] = VAR(AX0_u16_Input_Mode_Arr[manu_spec_Input_Mode_command[1]]);

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Input_Mode_command[2];

      ret_val = FalInitUploadRespond(SDO20E0S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalAddrCommandBg
// Description:
//          This function is called in response object 0x20E1 from Fieldbus.
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
int FalAddrCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
   }
   else// read operation
   {
      if (IS_ROTARY_SWITCH)
       {
        if (IS_EC_DRIVE)
       {
          if(((Rotary_Switch & 0x00F0) >> 4) < 10)  //digits: 0-9
            manu_spec_addr_command[0] = (('0' + ((Rotary_Switch & 0x00F0) >> 4)) <<8);
            else //letters: A-F
            manu_spec_addr_command[0] = (('A' + (((Rotary_Switch & 0x00F0) >> 4) - 10)) <<8);

            if((Rotary_Switch & 0x000F) < 10) //digits: 0-9
               manu_spec_addr_command[0] += ('0' + (Rotary_Switch & 0x000F));
            else //letters: A-F
            manu_spec_addr_command[0] += ('A' + ((Rotary_Switch & 0x000F) - 10));

       }
       else
       {
          if(((Rotary_Switch & 0x00F0) >> 4) < 10)  //digits: 0-9
            manu_spec_addr_command[0] = ('0' + ((Rotary_Switch & 0x00F0) >> 4));
            else //letters: A-F
            manu_spec_addr_command[0] = ('A' + (((Rotary_Switch & 0x00F0) >> 4) - 10));

            if((Rotary_Switch & 0x000F) < 10) //digits: 0-9
               manu_spec_addr_command[1] = ('0' + (Rotary_Switch & 0x000F));
            else //letters: A-F
            manu_spec_addr_command[1] = ('A' + ((Rotary_Switch & 0x000F) - 10));

        }
       }
       else//rotary display not assembled
      {
         manu_spec_addr_command[0] = 0x0;
      }

      p_fieldbus_bg_tx_data[0] = manu_spec_addr_command[0];
      p_fieldbus_bg_tx_data[1] = manu_spec_addr_command[1];
      ret_val = FalInitUploadRespond(SDO20E1S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalDisplayTestCommandBg
// Description:
//          This function is called in response object 0x20E2 from Fieldbus.
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
int FalDisplayTestCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
        u16_tmp = (unsigned int)p_data[0] & 0xFFFF;
      if(u16_tmp != 1)
         return VALUE_IS_NOT_ALLOWED;

      manu_spec_display_test_command = 0; // initialize the variable

      SalTestLedCommand();
         ret_val = FalInitDownloadRespond((int)SDO20E2S0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_display_test_command;
      ret_val = FalInitUploadRespond((int)SDO20E2S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalEncOutModeCommandBg
// Description:
//          This function is called in response object 0x20E3 from Fieldbus.
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
int FalEncOutModeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
     u16_tmp = (unsigned int)p_data[0] & 0xFFFF;
     ret_val = SalEncoderSimMode((long long)u16_tmp, 0);

     if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO20E3S0, s16_msg_id);
   }
   else// read operation
   {
      manu_enc_out_mode_command = BGVAR(u8_EncoutMode);
       p_fieldbus_bg_tx_data[0] = manu_enc_out_mode_command;
       ret_val = FalInitUploadRespond(SDO20E3S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalEncoderSimResCommandBg
// Description:
//          This function is called in response object 0x20E4 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalEncoderSimResCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, drive = 0, s16_tmp = 0;
   long s32_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      ret_val = SalEncoderSimResCommand((long long)s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         s16_tmp = 0x0101;
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         //if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO20E4S0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_Enc_Sim_Res_command = BGVAR(s32_Enc_Sim_Res);
      p_fieldbus_bg_tx_data[0] = (manu_spec_Enc_Sim_Res_command & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Enc_Sim_Res_command >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO20E4S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}




//**********************************************************
// Function Name: SalEncSimTestCommand
// Description:
//    Writes Encoder simulation Testing parameters
//
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int SalEncSimTestCommand(void)
{
    unsigned int u16_return_value, u16_enc_sim_test_number_of_cycles_temp;
    int s16_enc_sim_test_Counts_temp;
    unsigned long u32_enc_sim_test_frequency_temp;

    u16_return_value = SAL_SUCCESS;


   if (s16_Number_Of_Parameters == 0)// Query last executed command's parameters
   {
      PrintUnsignedInteger(u32_Enc_Sim_Test_Frequency);
      PrintChar(SPACE);
      PrintUnsignedInteger(u16_Enc_Sim_Test_Counts);
      PrintChar(SPACE);
      PrintUnsignedInteger(u16_Enc_Sim_Test_Number_Of_Cycles);
      PrintCrLf();
      return SAL_SUCCESS;
   }

   // input checking
   if (s64_Execution_Parameter[0] > 16000000LL) return VALUE_TOO_HIGH; // Max freq by HW is 16M counts/sec
   if ((s64_Execution_Parameter[0] < 32000LL) && (s64_Execution_Parameter[0] != 0LL)) return VALUE_TOO_LOW;
   if (s64_Execution_Parameter[1] > 65535LL) return VALUE_TOO_HIGH;
   if (s64_Execution_Parameter[2] > 65535LL) return VALUE_TOO_HIGH;

   u32_enc_sim_test_frequency_temp = (unsigned long)s64_Execution_Parameter[0]; // frequency
   s16_enc_sim_test_Counts_temp = (int)s64_Execution_Parameter[1]; // counts
   u16_enc_sim_test_number_of_cycles_temp = (unsigned int)s64_Execution_Parameter[2]; // number of cycle, managed in background task


   if(!BGVAR(u32_enc_sim_test_frequency_temp) && (!BGVAR(s16_enc_sim_test_Counts_temp)) && (!BGVAR(u16_enc_sim_test_number_of_cycles_temp)))// if all parameters are set to zero
   {
      BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_DISABLE; // Disable user request, move to regular encoder mode
      return (u16_return_value);
   }


   if((BGVAR(u8_Enc_Sim_Test_Request) == ENC_SIM_TEST_DISABLE) || (BGVAR(u8_Enc_Sim_Test_Request) == ENC_SIM_TEST_IDLE)) // state is idle or disable
   {
      BGVAR(u32_Enc_Sim_Test_Frequency) = u32_enc_sim_test_frequency_temp;
      if (s16_enc_sim_test_Counts_temp < 0) // Set the FPGA direction bit according to the sign of # counts
      {
         s16_enc_sim_test_Counts_temp = -s16_enc_sim_test_Counts_temp;
         *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 1;
      }
      else
         *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 0;

      BGVAR(u16_Enc_Sim_Test_Counts) = s16_enc_sim_test_Counts_temp;
      BGVAR(u16_Enc_Sim_Test_Number_Of_Cycles) = u16_enc_sim_test_number_of_cycles_temp;
      BGVAR(u8_Enc_Sim_Test_Request) = ENC_SIM_TEST_ENABLE; //we've got a user request
   }
   else  // enc sim test during simulation, not available (ENC_SIM_TEST_ENABLE or ENC_SIM_TEST_BUSY or ENC_SIM_TEST_WAIT_BETWEEN_CYCLES)
   {
      return NOT_AVAILABLE;
   }

    return (u16_return_value);
}



//**********************************************************
// Function Name: FalEncoderSimIndPosCommandBg
// Description:
//          This function is called in response object 0x20E5 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalEncoderSimIndPosCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      LVAR(AX0_u32_Encsim_ZPos) = u32_tmp;

      ret_val = FalInitDownloadRespond((int)SDO20E5S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Enc_Sim_Index_Pos_command = LVAR(AX0_u32_Encsim_ZPos);
      p_fieldbus_bg_tx_data[0] = (manu_spec_Enc_Sim_Index_Pos_command & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Enc_Sim_Index_Pos_command >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO20E5S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalRecordDoneIndCommandBg
// Description:
//          This function is called in response object 0x20E6 from Fieldbus.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordDoneIndCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long long s64_tmp = 0LL;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
   }
   else// read operation
   {
      SalRecdoneCommand(&s64_tmp);
      manu_spec_Rec_Done_command = (unsigned char)s64_tmp;

      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Rec_Done_command;
      ret_val = FalInitUploadRespond(SDO20E6S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalFaultCodeCommandBg
// Description:
//          This function is called in response object 0x603F from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalFaultCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   u16_operation += 0;
   p_data += 0; //For Remark Avoidance
   p_fieldbus_bg_tx_data[0] = p402_error_code;
   ret_val = FalInitUploadRespond(SDO603FS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);

   return (ret_val);
}


//**********************************************************
// Function Name: FalRecordGrabPacketSelectCommandBg
// Description:
//          This function is called in response object 0x20E7  index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordGrabPacketSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
   {
         manu_spec_Rec_Grab_command.DataStatus=0; //clear manu_spec_Rec_Grab_command.DataStatus
         retValue = FalInitDownloadRespond((int)SDO20E7S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Rec_Grab_command.PacketSelect;
      retValue = FalInitUploadRespond((int)SDO20E7S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(retValue);
}

//**********************************************************
// Function Name: FalRecordGrabCommandBg
// Description:
//          This function is called in response object 0x20E7  index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//*********************************************************
int FalRecordGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   int s16_Delta_Len;
   unsigned int u16_data_len=0;
   p_data += 0;
   u16_operation += 0;

    switch (manu_spec_Rec_Grab_command.PacketSelect)
   {
      case 0:
         retValue = LoadUnitsPacket();
         break;
      case 1:
          BinaryDataHeader();
         break;
   }
   if(retValue != FAL_SUCCESS)
      return retValue;

   if (manu_spec_Rec_Grab_command.PacketSelect<2)
   {
      setDomainAddr(0x20E7, 2, (UNSIGNED8 *) &u16_General_Domain[0] CO_COMMA_LINE_PARA_DECL);
      u16_data_len = u16_General_Domain[264];//size in words

   }
   else
   {
       switch (manu_spec_Rec_Grab_command.PacketSelect)
      {
           case 2: //packet start location s16_Debug_Ram[0][0]

               setDomainAddr(0x20E7, 2, (UNSIGNED8 *) &(s16_Debug_Ram[manu_spec_Rec_Grab_command.NumOfChn][u16_Record_Start]) CO_COMMA_LINE_PARA_DECL);
              s16_Delta_Len=u16_Record_End -u16_Record_Start;
                 s16_Delta_Len=(s16_Delta_Len<0)?(u16_Ram_Rec_Length_Avaliable-u16_Record_Start):s16_Delta_Len;
                 u16_data_len=(unsigned int)s16_Delta_Len;
           break;
         case 3://in case the packet end was shifted to the start of the array
               setDomainAddr(0x20E7, 2, (UNSIGNED8 *) &s16_Debug_Ram[manu_spec_Rec_Grab_command.NumOfChn][0] CO_COMMA_LINE_PARA_DECL);
                 u16_data_len = u16_Record_End;
           break;
      }
   }
    setDomainSize(0x20E7,2, u16_data_len*2 CO_COMMA_LINE_PARA_DECL);

   if (IS_EC_DRIVE)
   {
       p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_Rec_Grab_command.Domain & 0xffff);
   }
    retValue = FalInitUploadRespond((int)SDO20E7S2, s16_msg_id, u16_data_len, (int*)p_fieldbus_bg_tx_data);
   return(retValue);
}

//**********************************************************
// Function Name: FalRecordGrabSampleLenLocBg
// Description:
//          This function is called in response object 0x20E7  index 3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordGrabSampleLenBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;

    if (manu_spec_Rec_Grab_command.PacketSelect>1)
       manu_spec_Rec_Grab_command.DataLength = s16_Recording_Length;
   else
      manu_spec_Rec_Grab_command.DataLength =0;
    p_fieldbus_bg_tx_data[0] = manu_spec_Rec_Grab_command.DataLength;
    retValue = FalInitUploadRespond((int)SDO20E7S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return(retValue);
}


//**********************************************************
// Function Name: FalRecordGrabDataStatusBg
// Description:
//          This function is called in response object 0x20E7  index 4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordGrabDataStatusBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;

   if ((manu_spec_Rec_Grab_command.PacketSelect>1) && !(s16_Record_Flags & DATA_AVAILABLE_MASK))  manu_spec_Rec_Grab_command.DataStatus=1;
    retValue = FalInitUploadRespond((int)SDO20E7S4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
    return(retValue);
}

//**********************************************************
// Function Name: FalRecordNumOfChnBg
// Description:
//          This function is called in response object 0x20E7  index 5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordRTDataAckBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;
/*
    used for RT recording handling
   if(u16_operation == 1)// write operation
   {
      if (manu_spec_Rec_Grab_command.RTDataAck==1 && (manu_spec_Rec_Grab_command.PacketSelect>1) &&  (manu_spec_Record.NumPoints==0)) //RT recording
      {
          s16_Record_Flags &= ~PRINT_RT_RECORD;
          s16_Record_Flags |= RT_UPDATE_DATA_PTR;
      }

       manu_spec_Rec_Grab_command.RTDataAck=0;
       retValue = FalInitDownloadRespond((int)SDO20E7S5, s16_msg_id);

    }
*/
       manu_spec_Rec_Grab_command.RTDataAck=0;
       p_fieldbus_bg_tx_data[0] = manu_spec_Rec_Grab_command.RTDataAck;
       retValue = FalInitUploadRespond((int)SDO20E7S5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);

    return(retValue);
}


//**********************************************************
// Function Name: FalRecordNumOfChnBg
// Description:
//          This function is called in response object 0x20E7  index 6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordNumOfChnBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO20E7S6, s16_msg_id);
   else// read operation
   {
       manu_spec_Rec_Grab_command.NumOfChn=s16_Record_Channel_Number;
       p_fieldbus_bg_tx_data[0] = manu_spec_Rec_Grab_command.NumOfChn;
       retValue = FalInitUploadRespond((int)SDO20E7S6, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
    }
    return(retValue);
}


//**********************************************************
// Function Name: FalRectrigVarBg
// Description:
//          This function is called in response object 0x20E8  index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRectrigVarBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20E8S1];
      while(i < u16_len)
       {
           if (manu_spec_Record_Trig.Var[i] >= 'a' && manu_spec_Record_Trig.Var[i] <= 'z')
         {
              manu_spec_Record_Trig.Var[i]-=0x20;
            p_data[i]-=0x20;
         }
         i++;
      }
      ret_val = FalInitDownloadRespond((int)SDO20E8S1, s16_msg_id);
    }
   else// read operation
   {
      do
      {
            manu_spec_Record_Trig.Var[i]-=(manu_spec_Record_Trig.Var[i] >= 'a' && manu_spec_Record_Trig.Var[i] <= 'z')?0x20:0;// Convert to upper case
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record_Trig.Var[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20E8S1]);
      manu_spec_Record_Trig.Var[i] = 0;
      ret_val = FalInitUploadRespond((int)SDO20E8S1, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalRectrigVarBg
// Description:
//          This function is called in response object 0x20E8  index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRectrigThrshLvlBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO20E8S2, s16_msg_id);
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = ((*(long*)&manu_spec_Record_Trig.ThrsLvl) & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = ((*(long*)&manu_spec_Record_Trig.ThrsLvl) >> 16);        //set 16 MSBits
      retValue = FalInitUploadRespond((int)SDO20E8S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);

}
//**********************************************************
// Function Name: FalRectrigPreTrigBg
// Description:
//          This function is called in response object 0x20E8  index 3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRectrigPreTrigBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO20E8S3, s16_msg_id);
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record_Trig.PreTrg;       //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20E8S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);

}
//**********************************************************
// Function Name: FalRectrigVarBg
// Description:
//          This function is called in response object 0x20E8  index 4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRectrigEdgePolarBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO20E8S4, s16_msg_id);
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record_Trig.EdgePlr;      //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20E8S4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);

}
//**********************************************************
// Function Name: FalRectrigActivateBg
// Description:
//          This function is called in response object 0x20E8  index 5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRectrigActivateBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, i = 0, retValue = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      if (p_data[0]!=0)
      {
         SalGetModeCommand(3,drive); //use of getmode 3 for binary data transfer
         u16_len = FB_objects_size_array[SDO20E8S1];
           while(i < u16_len)
          {
             u8_Execution_String[0][i] = (char)manu_spec_Record_Trig.Var[i];
            i++;
         }
         // null terminate the internal variable
         u8_Execution_String[0][u16_len] = '\0';
           //////////////////////////////////////
         s64_Execution_Parameter[1] = (long)(((*(long*)&manu_spec_Record_Trig.ThrsLvl) >> 16)) & 0xFFFF;
         s64_Execution_Parameter[1] <<= 16;
         s64_Execution_Parameter[1] |= (unsigned long)(((*(long*)&manu_spec_Record_Trig.ThrsLvl) & 0xFFFF)) & 0xFFFF;
         s64_Execution_Parameter[1] = (long)(*(float*)(&s64_Execution_Parameter) * 1000.0);
           ///////////////////////////////////
           s64_Execution_Parameter[2] = manu_spec_Record_Trig.PreTrg ;
           ///////////////////////////////////
             s64_Execution_Parameter[3] = (long)manu_spec_Record_Trig.EdgePlr & 0xFFFF;
          s16_Number_Of_Parameters=3;
           ///////////////////////////////////
         retValue = RecTrigCommand();
         if(retValue != FAL_SUCCESS)
               return retValue;

      }
      retValue = FalInitDownloadRespond((int)SDO20E8S5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record_Trig.Activate;     //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20E8S5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalRecordOffBg
// Description:
//          This function is called in response object 0x20E9  index 0 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************

int FalRecordOffBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
   {

      retValue = RecordOffCommand();
      if(retValue != FAL_SUCCESS)
      {
            return retValue;

      }
       retValue = FalInitDownloadRespond((int)SDO20E9S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record_Off;      //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20E9S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalInfoSelectCommandBg
// Description:
//          This function is called in response object 0x212A sub index 1 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalInfoSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val,drive=0 ;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO212AS1, s16_msg_id);
   }
   else
   {
      DomainHandler(drive,(void *)&manu_spec_Info_Command,0x212A,InfoCommand,PACKETS_TRANSFER_ACTIVATE);

      p_fieldbus_bg_tx_data[0] = manu_spec_Info_Command.StatusSelect;
      ret_val = FalInitUploadRespond((SDO212AS1), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalInfoGrabCommandBg
// Description:
//
//   This function is called in response object 0x212A sub index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalInfoGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;

   DomainHandler(drive,(void *)&manu_spec_Info_Command,0x212A,NULL,DOMAIN_DATA_UPLOAD);

   if (IS_EC_DRIVE)
      p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_Info_Command.Domain & 0xffff);
   ret_val = FalInitUploadRespond((int)(SDO212AS2), s16_msg_id, u16_Domain_Data_Length, (int*)p_fieldbus_bg_tx_data); //The length needs to be in Bytes
   return (ret_val);
}


//**********************************************************
// Function Name: FalTorqueWindowCommandBg
// Description:
//          This function is called in response object 0x212B sub index 0 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTorqueWindowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
  int ret_val ;
   p_data += 0;

   if(u16_operation == 1)// write operation
      ret_val = FalInitDownloadRespond((int)SDO212BS0, s16_msg_id);
   else
   {
         p_fieldbus_bg_tx_data[0] = (manu_spec_Torque_Window_Command & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1] = (manu_spec_Torque_Window_Command >> 16);    //set 16 MSBits
         ret_val = FalInitUploadRespond((SDO212BS0), s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);

}

//**********************************************************
// Function Name: FalBlockControlWordCommandBg
// Description:
//          This function is called in response object 0x212C sub index 0 from Fieldbus
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBlockControlWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
      ret_val = FalInitDownloadRespond((int)SDO212CS0, s16_msg_id);
   else
   {
         p_fieldbus_bg_tx_data[0] = manu_spec_Block_Control_Word_Command;
         ret_val = FalInitUploadRespond((SDO212CS0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);

}

//**********************************************************
// Function Name: FalRecordSampleTimeBg
// Description:
//          This function is called in response object 0x20EA  index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordSampleTimeBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int retValue = FAL_SUCCESS,index1=0,index2;
    if(u16_operation == 1)// write operation
   {
       for (index2=0;index2<6;index2++)
         for (index1=0;index1<15;index1++)
            u8_Execution_String[index2][index1]=0;
      manu_spec_Record.Var1[0]=0;
      manu_spec_Record.Var2[0]=0;
      manu_spec_Record.Var3[0]=0;
      manu_spec_Record.Var4[0]=0;
      manu_spec_Record.Var5[0]=0;
      manu_spec_Record.Var6[0]=0;

      s64_Execution_Parameter[0]= p_data[0];
       retValue = FalInitDownloadRespond((int)SDO20EAS1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record.SampleTime;     //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20EAS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}
//**********************************************************
// Function Name: FalRecordNumPointsBg
// Description:
//          This function is called in response object 0x20EA  index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordNumPointsBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int retValue = FAL_SUCCESS;
   p_data += 0;
    if(u16_operation == 1)// write operation
   {
       if (manu_spec_Record.NumPoints==0 )
            return VALUE_IS_NOT_ALLOWED;
       retValue = FalInitDownloadRespond((int)SDO20EAS2, s16_msg_id);
    }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record.NumPoints;      //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20EAS2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalRecordVar1Bg
// Description:
//          This function is called in response object 0x20EA  index 3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar1Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS3];
        while(i < u16_len)
       {
          // Convert to upper case
         if (manu_spec_Record.Var1[i] >= 'a' && manu_spec_Record.Var1[i] <= 'z')
         {
              manu_spec_Record.Var1[i]-=0x20;
            p_data[i]-=0x20;
         }
         u8_Execution_String[0][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[0][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS3, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var1[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS3]);

      manu_spec_Record.Var1[i] = 0;

      ret_val = FalInitUploadRespond((int)SDO20EAS3, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalRecordVar2Bg
// Description:
//          This function is called in response object 0x20EA  index 4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar2Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS4];
        while(i < u16_len)
       {
         if (manu_spec_Record.Var2[i] >= 'a' && manu_spec_Record.Var2[i] <= 'z')
         {
              manu_spec_Record.Var2[i]-=0x20;
            p_data[i]-=0x20;
         }

         u8_Execution_String[1][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[1][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS4, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var2[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS4]);

      manu_spec_Record.Var2[i] = 0;

      ret_val = FalInitUploadRespond((int)SDO20EAS4, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}
//**********************************************************
// Function Name: FalRecordVar3Bg
// Description:
//          This function is called in response object 0x20EA  index 5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar3Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS5];
        while(i < u16_len)
       {
         if (manu_spec_Record.Var3[i] >= 'a' && manu_spec_Record.Var3[i] <= 'z')
         {
              manu_spec_Record.Var3[i]-=0x20;
            p_data[i]-=0x20;
         }

         u8_Execution_String[2][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[2][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS5, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var3[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS5]);
      manu_spec_Record.Var3[i] = 0;
      ret_val = FalInitUploadRespond((int)SDO20EAS5, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}
//**********************************************************
// Function Name: FalRecordVar4Bg
// Description:
//          This function is called in response object 0x20EA  index 6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar4Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS6];
        while(i < u16_len)
       {
         if (manu_spec_Record.Var4[i] >= 'a' && manu_spec_Record.Var4[i] <= 'z')
         {
              manu_spec_Record.Var4[i]-=0x20;
            p_data[i]-=0x20;
         }

         u8_Execution_String[3][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[3][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS6, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var1[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS6]);
      manu_spec_Record.Var4[i] = 0;
      ret_val = FalInitUploadRespond((int)SDO20EAS6, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}
//**********************************************************
// Function Name: FalRecordVar5Bg
// Description:
//          This function is called in response object 0x20EA  index 7 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar5Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS7];
        while(i < u16_len)
       {
         if (manu_spec_Record.Var5[i] >= 'a' && manu_spec_Record.Var5[i] <= 'z')
         {
              manu_spec_Record.Var5[i]-=0x20;
            p_data[i]-=0x20;
         }

         u8_Execution_String[4][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[4][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS7, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var5[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS7]);
      manu_spec_Record.Var5[i] = 0;
      ret_val = FalInitUploadRespond((int)SDO20EAS7, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}
//**********************************************************
// Function Name: FalRecordVar6Bg
// Description:
//          This function is called in response object 0x20EA  index 8 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordVar6Bg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0, ret_val = FAL_SUCCESS;
   unsigned int u16_len;

   if(u16_operation == 1)// write operation
   {
      u16_len = FB_objects_size_array[SDO20EAS8];
        while(i < u16_len)
       {
         if (manu_spec_Record.Var6[i] >= 'a' && manu_spec_Record.Var6[i] <= 'z')
         {
              manu_spec_Record.Var6[i]-=0x20;
            p_data[i]-=0x20;
         }

         u8_Execution_String[5][i] = (char)p_data[i];
         i++;
      }
      // null terminate the internal variable
      u8_Execution_String[5][u16_len] = '\0';
      ret_val = FalInitDownloadRespond((int)SDO20EAS8, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         p_fieldbus_bg_tx_data[i] =  manu_spec_Record.Var6[i];
         i++;
      }
      while (i < FB_objects_size_array[SDO20EAS8]);
      manu_spec_Record.Var6[i] = 0;
      ret_val = FalInitUploadRespond((int)SDO20EAS8, s16_msg_id, i, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}

//**********************************************************
// Function Name: FalRecordActivateBg
// Description:
//          This function is called in response object 0x20EA  index 9 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordActivateBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int retValue = FAL_SUCCESS;
    if(u16_operation == 1)// write operation
   {

      if (p_data[0]!=0)
      {
          if (manu_spec_Record.NumPoints==0 )
             return VALUE_IS_NOT_ALLOWED;
        else
        {
            s16_Number_Of_Parameters=2 + (manu_spec_Record.Var1[0]!=0?1:0) + (manu_spec_Record.Var2[0]!=0?1:0)+ (manu_spec_Record.Var3[0]!=0?1:0)+ (manu_spec_Record.Var4[0]!=0?1:0)+ (manu_spec_Record.Var5[0]!=0?1:0)+ (manu_spec_Record.Var6[0]!=0?1:0);
           s64_Execution_Parameter[0] = manu_spec_Record.SampleTime;
           s64_Execution_Parameter[1] = manu_spec_Record.NumPoints;
             retValue = RecordCommand();
        }
         if(retValue != FAL_SUCCESS)
           return retValue;
      }
       retValue = FalInitDownloadRespond((int)SDO20EAS9, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Record.Activate;       //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO20EAS9, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}


//**********************************************************
// Function Name: FalRecordActivateBg
// Description:
//          This function is called in response object 0x20EB  index 0 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordStatusBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int retValue = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;
    SalRecordingStatusCommand((long long *)&manu_spec_Record_Status);
    p_fieldbus_bg_tx_data[0] = manu_spec_Record_Status;     //set 16 LSBits
   retValue = FalInitUploadRespond((int)SDO20EBS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (retValue);
}

//**********************************************************
// Function Name: FalRecordActivateBg
// Description:
//          This function is called in response object 0x20EC  index 0 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalRecordReadyBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int retValue = FAL_SUCCESS;
    p_data += 0;
   u16_operation += 0;
    SalRecordingReadyCommand((long long *)&manu_spec_Record_Ready);
    p_fieldbus_bg_tx_data[0] = manu_spec_Record_Ready;      //set 16 LSBits
   retValue = FalInitUploadRespond((int)SDO20ECS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   return (retValue);
}


//**********************************************************
// Function Name: FalControlWordCommandBg
// Description:
//          This function is called in response object 0x6040 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalControlWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {

     if(manu_spec_Block_Control_Word_Command != 0x1234)
     {
         if(IS_EC_DRIVE_AND_COMMODE_1)
           SET_DPRAM_FPGA_MUX_REGISTER(0);

         //do not allow to change enable bit in control word if fieldbus state is not operational
         //nitsan: request from schnider to allow enabling lxm28 drive in pre_operational state
         if((IS_EC_DRIVE_AND_COMMODE_1 && (*(unsigned int*)p_u16_tx_nmt_state != EC_NMTSTATE_OP) && (*((int*)p_data) & 0x0008)) ||
            (IS_CAN_DRIVE_AND_COMMODE_1 && (u16_Product != SHNDR && u16_Product != SHNDR_HW) && (GL_ARRAY(co_Node).eState != OPERATIONAL) && (*((int*)p_data) & 0x0008)))
         {
       //     FalInitDownloadRespond((int)SDO6040S0, s16_msg_id);
         p402_controlword &= ~0x0008;
            return FAL_WRONG_NMT_STATE;
         }
     }
     p402_controlword = *((int*)p_data);
     ret_val = FalInitDownloadRespond((int)SDO6040S0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_controlword;
       ret_val = FalInitUploadRespond(SDO6040S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalStatusWordCommandBg
// Description:
//          This function is called in response object 0x6041 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalStatusWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   u16_operation += 0;
   p_data += 0; //For Remark Avoidance
   p_fieldbus_bg_tx_data[0] = p402_statusword;
   ret_val = FalInitUploadRespond(SDO6041S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);

   return (ret_val);
}

//***********************************************************
// Function Name: FalOpmodeCommandRt
// Description:
//           This function is called in response object 0x6060 from Fieldbus RPDO.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************

#pragma CODE_SECTION(FalOpmodeCommandRt, "ramcan");
int  FalOpmodeCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int s16_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // opmode is signed 8 bit
   s16_tmp = p_data[0] & 0xff;

   //move from 8 bits to 16 bits with sign extension
   s16_tmp <<= 8;
   s16_tmp >>= 8;

   //data to set is the same as current data
   if ((s16_tmp == 0) || (s16_tmp == p402_modes_of_operation_display))//ITAI - Bug 4688: CDHD CAN - Unable to change drive opmode via PDO 0x6060
      return FAL_SUCCESS;

   if (s16_tmp > 0x0a) return VALUE_TOO_HIGH;

   // for ipr 615 set new high limit for 0x6060, prevent using modes: sync torque and sync vel
   // port post build tool is updated accordingly
   //if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   //{
   //   if (s16_tmp > 0x08) return VALUE_TOO_HIGH;
   //}

   if (s16_tmp < (int)CAN_MIN_OPMODE) return VALUE_TOO_LOW;
   // BZ#3143 - Notify the user about not supported can opmodes
   if ((s16_tmp == 2) || (s16_tmp == 5)) return VALUE_IS_NOT_ALLOWED;    
   if((u16_Modulo_Mode) && (s16_tmp >= 7)) return FB_NOT_ALLOWED_IN_MODULO_MODE;

   // if opmode == zero -> no change in opmode
   if (s16_tmp && (s16_tmp != p402_modes_of_operation_display))
   {
      // If no one else blocks the OPMODE change
      if(BGVAR(u16_Block_Opmode_Change) == 0)
      {
         BGVAR(s16_CAN_Opmode_Temp) = s16_tmp;
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
      }
      else
      {
         return OTHER_PROCEDURE_RUNNING;
      }
   }

   return FAL_SUCCESS;
}

#pragma CODE_SECTION(FalControlWordCommandRt, "ramcan");
int  FalControlWordCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // opmode is signed 8 bit
   p402_controlword = p_data[0];

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalOpmodeCommandBg
// Description:
//          This function is called in response object 0x6060 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalOpmodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int canopenOpmode = 0;

   if(u16_operation == 1)// write operation
   {
      if (BGVAR(s8_BurninParam) != 0) return (BURNIN_ACTIVE); // if in burnin mode return

      // If someone else blocks the OPMODE change
      if(BGVAR(u16_Block_Opmode_Change) != 0)
      {
         return (OTHER_PROCEDURE_RUNNING); // Another procedure is running that blocks the OPMODE change
      }

      canopenOpmode = *((int*)p_data);

      //move from 16 bits to 8 bits
      canopenOpmode <<= 8;
      canopenOpmode >>= 8;


      //if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      //{
         // for ipr 615 set new high limit for 0x6060, prevent using modes: sync torque and sync vel
         // port post build tool is updated accordingly
         //if (canopenOpmode > 0x08) return VALUE_TOO_HIGH;
      //}

      if (canopenOpmode > 0x0a) return VALUE_TOO_HIGH;
      if (canopenOpmode < (int)CAN_MIN_OPMODE) return VALUE_TOO_LOW;
      // BZ#3143 - Notify the user about not supported can opmodes
      if ((canopenOpmode == 2) || (canopenOpmode == 5)) return VALUE_IS_NOT_ALLOWED; 
      if((u16_Modulo_Mode) && (canopenOpmode >= 7)) return FB_NOT_ALLOWED_IN_MODULO_MODE;

      // if opmode value is zero or the same, then there is no change to the current opmode (just update canopen param).
      if ((canopenOpmode == 0) || (canopenOpmode == p402_modes_of_operation_display))
      {
         p402_modes_of_operation = canopenOpmode;
      }
      else
      {
         // allow opmode change on the fly in schnieder drive
         //16.01.2016 - Itai - open opmode change on the fly for all opmodes
         //if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
         //{
         //   if((Enabled(DRIVE_PARAM) && ((BGVAR(s16_CAN_Opmode) != 1) && (BGVAR(s16_CAN_Opmode) != 6) && (BGVAR(s16_CAN_Opmode) != 8))) || // check if in mode is 1 or 6 OR 8 and if the new mode is 1 OR 6 OR 8
         //      (Enabled(DRIVE_PARAM) &&((canopenOpmode != 1) && (canopenOpmode != 6) && (canopenOpmode != 8))))                           // and that the new and old are not equal
         //        return (DRIVE_ACTIVE);
         //}

         // change opmode using CheckCanOpmode() to handle corect opmode change from homing to sync position
         // (bug 3795: BYang00000396:Uncontrolled movement of the axis after switching the op-mode from "6 = homimg" to "8 = cyclic synchronous position")
         BGVAR(s16_CAN_Opmode_Temp) = canopenOpmode;
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         CheckCanOpmode(drive);
      }

      if (ret_val == FAL_SUCCESS)
      {
        ret_val = FalInitDownloadRespond((int)SDO6060S0, s16_msg_id);
      }
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_modes_of_operation;
     ret_val = FalInitUploadRespond(SDO6060S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}



//**********************************************************
// Function Name: ConvertCanOpenOpmode
// Description:
//          This function is converts canopen opmode to internal opmode
//          opmodeIn: CANopen opmode
//          opmodeOut: internal opmode
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int ConvertCanOpenOpmode(int drive, int opmodeIn, int* opmodeOut)
{
   int ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(opmodeIn)
   {
      case PROFILE_POSITION_MODE:            //profile postion mode
      case HOMING_MODE:                      //profile torque mode
      case INTRPOLATED_POSITION_MODE:        //interpolated position mode
      case CYCLIC_SYNCHRONOUS_POSITION_MODE: //cyclic synchronous position
     case JOG_MODE:                         //schneider jog mode. using ptp generator.
     case SFBVCMOVE_MODE:                   //mode for Frencken to allow only SFBVCMOVE SDO
         *opmodeOut = 8;
         break;

      case PROFILE_VELOCITY_MODE:            //profile velocity mode
      case CYCLIC_SYNCHRONOUS_VELOCITY_MODE: //cyclic synchronous velocity
         *opmodeOut = 0;
         break;

      case PROFILE_TORQUE_MODE:              //profile torque mode
      case CYCLIC_SYNCHRONOUS_TORQUE_MODE:   //cyclic synchronous torque
         *opmodeOut = 2;
         break;

      case GEAR_MODE:   //Position control, using gearing input
         *opmodeOut = 4;
         break;

      case ANALOG_VEL_MODE:      // analog velocity mode
         *opmodeOut = 1;
         break;

      case ANALOG_TORQUE_MODE:   // analog torque mode
         *opmodeOut = 3;
         break;


      default:
        ret_val = VALUE_OUT_OF_RANGE;
       break;
   }

   if(ret_val != FAL_SUCCESS)
   {
      if ( (*opmodeOut == 2) && (BGVAR(s16_ControllerType) == VOLTAGE_CONTROL) )
         *opmodeOut = 0L;
   }

   return ret_val;
}

//**********************************************************
// Function Name: FalSetFBSyncOpmode
// Description:
//          This function is responsible to set Field Bus synchronious mode settings if needed
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalSetFBSyncOpmode(int drive, int opmode)
{
   // AXIS_OFF;

   //exit if not fieldbus
   EXIT_IF_NOT_FIELDBUS1

   switch(opmode)
   {
     case CYCLIC_SYNCHRONOUS_POSITION_MODE: //for cyclic synchronous position mode
     case INTRPOLATED_POSITION_MODE:
         //init EtherCAT cyclic position mode
         FieldBusInterpolationInit(drive,POSITION_INTERPOLATION); //** add MAIN_CONTROLLER assigment to all AX0_s16_Pos_Loop_Vcmnd_Ptr **
         //POS_LOOP_VCMD_PTR_ASSIGN((int)((long)&LVAR(AX0_s32_Pos_Vcmd) & 0xffff));

         // fix bug IPR 396: Uncontrolled movement of the axis after switching the op-mode from "6 = homimg" to "8 = cyclic synchronous position"
         // in case of changing opmodes while hold is active, need to update the back up copy of target pos (fieldbus or internal PTP)
         // e.g. while homing in progress, limit-switch occurs, next, master changes opmode to 8 (sync position) and then clears limit-switch alarm.
         // in this case the tagrget positon pointer should be updated back to fieldbus only after limit-switch is cleared
         if (VAR(u16_Hold) == 0)
         {
            VAR(AX0_s16_Pre_Pos_Input_Ptr) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);
         }
         else
         {
            VAR(AX0_s16_Pre_Pos_Input_Ptr_Store) = (int)((long)&LVAR(AX0_u32_Pos_Cmd_Ptp_Lo) & 0xffff);
         }
      break;

      case CYCLIC_SYNCHRONOUS_VELOCITY_MODE: //for cyclic synchronous velocity mode
         //init EtherCAT cyclic velocity mode
         FieldBusInterpolationInit(drive,VELOCITY_INTERPOLATION);
         VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&LVAR(AX0_u32_FieldBus_Int_Lo) & 0x0000ffff);
      break;

      case CYCLIC_SYNCHRONOUS_TORQUE_MODE://for cyclic synchronous torque mode
         //init EtherCAT cyclic torque mode
         FieldBusInterpolationInit(drive,TORQUE_INTERPOLATION);
         //current pointer is set by SalOpmodeCommand
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = (int*)((long)&LVAR(AX0_u32_FieldBus_Int_Lo));
      break;

      case PROFILE_POSITION_MODE:// This case was created MOVESMOOTH command to detect fieldbus mode
         VAR(AX0_u16_FieldBus_Int_Type) = 10;
      break;

      case PROFILE_VELOCITY_MODE: // For all none SYNC modes mark that no interpolation needed so that interpolation mechanism will not run
      case PROFILE_TORQUE_MODE:
      case HOMING_MODE:
      case GEAR_MODE:
      case ANALOG_VEL_MODE:      // analog velocity mode
      case ANALOG_TORQUE_MODE:   // analog torque mode
      case SFBVCMOVE_MODE:
         VAR(AX0_u16_FieldBus_Int_Type) = 0;
         break;

      default:// more modes TBD!!!
         break;
   }

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalDisplayOpmodeCommandBg
// Description:
//          This function is called in response object 0x6061 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalDisplayOpmodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance

   if(u16_operation != 1)
   {
       p402_modes_of_operation_display = BGVAR(s16_CAN_Opmode);
       p_fieldbus_bg_tx_data[0] = p402_modes_of_operation_display;
       ret_val = FalInitUploadRespond(SDO6061S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

    return (ret_val);
}




//***********************************************************
// Function Name: FalDCLinkVoltageCommandRt
// Description:
//           This function is called in response object 0x6079 from Fieldbus TPDO.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDCLinkVoltageCommandRt, "ramcan");
int  FalDCLinkVoltageCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_DC_link_circuit_voltage = (*(unsigned int*)p_data);
   return FAL_SUCCESS;
}

//***********************************************************
// Function Name: FalDisplayOpmodeCommandRt
// Description:
//           This function is called in response object 0x6061 from Fieldbus TPDO.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDisplayOpmodeCommandRt, "ramcan");
int  FalDisplayOpmodeCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

    //take data, only 1 byte needed for opmode
//  p402_modes_of_operation_display = (*(int*)p_data) & 0xff;

   // negative opmoes require all 16 bits. otherwise opmode handling of negative opmodes wont work well
   // (comparison to JOG_MODE is corrupted)

   p402_modes_of_operation_display =((*(int*)p_data)<<8)>>8;  //sign extension from 8bit to 16bit
   return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalActualPositionCommandBg
// Description:
//          This function is called in response object 0x6064 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//           nitsan 3/2/14 - if dual mode, return SFB
//**********************************************************
int FalActualPositionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   long long s64_tmp = 0LL;
   unsigned int u16_temp_time;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO6064S0, s16_msg_id);
   }
   else// read operation
   {
      //ITAI - Removed on dual feedback branch
      //if (BGVAR(s16_SFBMode) != 1)
      //{  //get PFB (secondary feedback is disabled)
         do {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_tmp = LLVAR(AX0_u32_PFB_DF_Lo); // This variable holds MFB if dual loop is disabled, and SFB if enabled
         } while (u16_temp_time != Cntr_3125);
      //}
      /*else
      {  //get SFB
         do {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_tmp = LLVAR(AX0_u32_Pext_Fdbk_User_Lo);
         } while (u16_temp_time != Cntr_3125);
      }
      */
      p402_position_actual_value = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_position_actual_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_position_actual_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6064S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalActualPositionInternalCommandBg
// Description:
//          This function is called in response object 0x6063 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions: nitsan 25/1/2015: return lower word of pfb as position actual intenal value without scaling.
//                   so the object wil return the position inside a revolution where 32 bit is full revolution
//                   (32bit / 2) is half revolution
//**********************************************************
int FalActualPositionInternalCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0; //For Remark Avoidance

   if(u16_operation == 0)
   {
      p402_position_actual_internal_value = LVAR(AX0_u32_Pos_Fdbk_User_Lo);
      p_fieldbus_bg_tx_data[0] = (p402_position_actual_internal_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_position_actual_internal_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6063S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalActualPositionInternalCommandRt
// Description:
//          This function is called in response object 0x6063 from Fieldbus via PDO.
//          return lower word of pfb as position actual intenal value without scaling.
//          so the object wil return the position inside a revolution where 32 bit is full revolution
//          (32bit / 2) is half revolution
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalActualPositionInternalCommandRt, "ramcan");
int FalActualPositionInternalCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark


   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= ((unsigned long)p_data[0]) & 0xffff;   // to avoid padding of 0xffff in the higher bytes if valu is negative

   p402_position_actual_internal_value = s32_tmp;
   return (FAL_SUCCESS);
}




//**********************************************************
// Function Name: FalPositionDemandCommandBg
// Description:
//          This function is called in response to object 0x6062 (position demand value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//**********************************************************
int FalPositionDemandCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, s16_temp_time;
   // AXIS_OFF;
   long long s64_tmp = 0LL;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO6062S0, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
      } while (s16_temp_time != Cntr_3125);

     p402_position_demand_value = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_position_demand_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_position_demand_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6062S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPositionDemandInternalCommandBg
// Description:
//          This function is called in response to object 0x60FC (position demand internal value) from Fieldbus.
//       Always return in counts
//
//
// Author: Nitsan
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//**********************************************************
int FalPositionDemandInternalCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, s16_temp_time;
   unsigned long u32_tmp1;
   long s32_tmp2;
   long long s64_tmp = 0LL;

   // AXIS_OFF;


   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60FCS0, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         u32_tmp1 =  LVAR(AX0_u32_Pos_Cmd_User_Lo);
         s32_tmp2 =  LVAR(AX0_s32_Pos_Cmd_User_Hi);
      } while (s16_temp_time != Cntr_3125);

      // fix bug 3367: Object 0x60CF (Position Demand Internal Value) does not work in profile position mode.
      // calculate the output in encoder counts
      // (pfb / 2^32) * (mencres * 4) == (pfb >> 32) * (mencres * 4) == (pfb>>30) * mencres == (pfb * mencres) >> 30
      s64_tmp = ((long long)s32_tmp2) << 32LL;
      s64_tmp |= (((long long)u32_tmp1) & 0x00000000FFFFFFFF);
      p402_position_demand_internal_value = (s64_tmp * BGVAR(u32_User_Motor_Enc_Res)) >> 30;

      p_fieldbus_bg_tx_data[0] = (p402_position_demand_internal_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_position_demand_internal_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60FCS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalMaxAccelerationCommandBg
// Description:
//          This function is called in response to object 0x60C5 (max acceleration) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//**********************************************************
int FalMaxAccelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   p_data += 0;
   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = CheckAccDecLimits(s64_tmp);
     if(ret_val == FAL_SUCCESS)
     {
         BGVAR(u64_CAN_Max_Acc) = s64_tmp;
         ret_val = SalAccCommand(BGVAR(u64_CAN_Acc),drive);
         if(ret_val == FAL_SUCCESS)
           {
            ret_val = FalInitDownloadRespond((int)SDO60C5S0, s16_msg_id);
        }
      }
   }
   else// read operation
   {
      p402_max_acceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_CAN_Max_Acc)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_max_acceleration & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_max_acceleration >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60C5S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalMaxDecelerationCommandBg
// Description:
//          This function is called in response to object 0x60C6 (max deceleration) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//**********************************************************
int FalMaxDecelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   p_data += 0;
   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = CheckAccDecLimits(s64_tmp);
     if(ret_val == FAL_SUCCESS)
     {
         BGVAR(u64_CAN_Max_Dec) = s64_tmp;
         ret_val = SalDecCommand(BGVAR(u64_CAN_Dec),drive);
         if(ret_val == FAL_SUCCESS)
           {
         ret_val = SalDecStopCommand(BGVAR(u64_CAN_Dec_Stop), drive);
         if(ret_val == FAL_SUCCESS)
            {
               ret_val = FalInitDownloadRespond((int)SDO60C6S0, s16_msg_id);
         }
        }
      }
   }
   else// read operation
   {
      p402_max_deceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_CAN_Max_Dec)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_max_deceleration & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_max_deceleration >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60C6S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalActualVelocityCommandBg
// Description:
//          This function is called in response object 0x606C from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: Lior 21/2/17 -//assign 0x606C with the DF velocity variable which contains MVEL/SVBVEL according SFBMODE
//**********************************************************
int FalActualVelocityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_temp_vfb;
   // AXIS_OFF;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO606CS0, s16_msg_id);
   }
   else// read operation
   {
      
       s32_temp_vfb = LVAR(AX0_s32_DF_Vel_Var_Fb_0); //assign 0x606C with the DF velocity variable which contains MVEL/SVBVEL according SFBMODE       

       p402_velocity_actual_value = (long)MultS64ByFixS64ToS64((long long)(s32_temp_vfb),
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_velocity_actual_value & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_velocity_actual_value >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO606CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalTargetTorqueCommandBg
// Description:
//          This function is called in response object 0x6071 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTargetTorqueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_tmp_torque_converted, s16_tmp_torque, s16_tmp_ilim = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {

      s16_tmp_torque = (*(int*)p_data);

      // Bug 3978 - make ILIM saturation only for the cyclic synchronous torque mode because for profile torque mode it is done by SAL function.
      if (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE)  //if cyclic syncronous torque mode
     {
         s16_tmp_ilim = (int)MultS64ByFixS64ToS64((long long)BGVAR(s32_Ilim_Actual),
                             BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                             BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      /* ITAI - changed that from FB_CAN_TORQUE_CONVERSION to FB_CAN_CURRENT_CONVERSION */
     /* In order to avoid MKT usage is units conversion */
       s16_tmp_torque_converted = (int)MultS64ByFixS64ToS64((long long)s16_tmp_torque,
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // BZ 6077: Movement direction is opposite to the given command (6071h \96 Target Torque) in profile torque mode.
      // if the units conversion result has a opposite sign, 
      // it means that probably the input icmd is too high and we got roll-over.
      // in this case, use the opposite ilim as command
      if ((s16_tmp_torque_converted ^ s16_tmp_torque) & 0x8000)
      {
         if (s16_tmp_torque_converted < 0)
            BGVAR(s16_Fb_Target_Torque) = s16_tmp_ilim;
         else
            BGVAR(s16_Fb_Target_Torque) = -s16_tmp_ilim;
      }
      else
      {
         // make sure we do not exceed ILIM
         // need to check for positive and negative values
         if (abs(s16_tmp_torque_converted) < s16_tmp_ilim)
         {
            // requested torque is below ilim
            BGVAR(s16_Fb_Target_Torque) = s16_tmp_torque_converted;
         }
         else if (s16_tmp_torque_converted < 0)
         {
            // requestd torque is beyond ilim (negative value)
            BGVAR(s16_Fb_Target_Torque) = -s16_tmp_ilim;
         }
         else
         {
            // requestd torque is beyond ilim (positive value)
            BGVAR(s16_Fb_Target_Torque) = s16_tmp_ilim;
         }
      }
     } 
     else//profile position mode
     {
        BGVAR(s16_Fb_Target_Torque) = s16_tmp_torque;
     }

      ret_val = FalInitDownloadRespond((int)SDO6071S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_target_torque;
      ret_val = FalInitUploadRespond(SDO6071S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }


   return (ret_val);
}

//**********************************************************
// Function Name: FalTorqueOffsetCommandBg
// Description:
//          This function is called in response object 0x60B2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTorqueOffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp_torque = 0;

   if(u16_operation == 1)// write operation
   {

      s32_tmp_torque = (long)MultS64ByFixS64ToS64((long long)(*(int*)p_data),
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      SalExtAdditiveIcmdCommand((long long)s32_tmp_torque,drive);
      ret_val = FalInitDownloadRespond((int)SDO60B2S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_torque_offset;
      ret_val = FalInitUploadRespond(SDO60B2S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }


   return (ret_val);
}

//**********************************************************
// Function Name: FalVelocityOffsetCommandBg
// Description:
//          This function is called in response object 0x60B1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityOffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (long)p_data[1] & 0xFFFF;
       s32_tmp <<= 16;
       s32_tmp |= ((unsigned long)p_data[0]) & 0xffff;   // to avoid padding of 0xffff in the higher bytes if valu is negative

      //convert to internal FB velocity in of loop units
      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

     SalExtAdditiveVcmdCommand(s64_tmp,drive);
    ret_val = FalInitDownloadRespond((int)SDO60B1S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_velocity_offset & 0xFFFF;        //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = p402_velocity_offset >> 16;            //set 16 MSBits

      ret_val = FalInitUploadRespond((int)SDO60B1S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}



//**********************************************************
// Function Name: FalTorqueDemandValueCommandBg
// Description:
//          This function is called in response object 0x6074 (Torque Demand Value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalTorqueDemandValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0;
   if(u16_operation != 1)// read operation
   {
      /* ITAI - changed that from FB_CAN_TORQUE_CONVERSION to FB_CAN_CURRENT_CONVERSION */
      /* In order to avoid MKT usage is units conversion */
       p402_torque_demand_value = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_Icmd),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = p402_torque_demand_value;

       ret_val = FalInitUploadRespond(SDO6074S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalMotorRatedCurrentCommandBg
// Description:
//          This function is called in response object 0x6075 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMotorRatedCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int s16_tmp;
   float f_tmp  = 0.0;
   int drive = 0, ret_val = FAL_SUCCESS;
   if(u16_operation == 1)// write operation
   {

       // if this object is saved in MTP (motor name plate), it cannot be modified.
       // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
       if ((BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3))
       {
        return MTP_USED;
      }

       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       u32_tmp <<= 16;
       u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);



       BGVAR(u32_Fb_Motor_Rated_Current) = u32_tmp;
       SalMotorIContCommand(BGVAR(u32_Fb_Motor_Rated_Current), drive);

       // attempt to perform config
       // after meeting at: 18/3/2013, it was decided that:
       // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
       s16_tmp = 0x0101;
       ret_val = FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);


       //Update 0x6076
      f_tmp = (float)((float)u32_tmp * ((float)BGVAR(u32_Motor_Kt)/1000.0));
       p402_motor_rated_torque = (unsigned int)f_tmp;

        ConvertFbCanOpenTorque(drive);

       ret_val = FalInitDownloadRespond((int)SDO6075S0,s16_msg_id);
   }
   else// read operation
   {
       BGVAR(u32_Fb_Motor_Rated_Current) = BGVAR(s32_Motor_I_Cont);    // canopen micont and serial micont are both in mA
       p402_motor_rated_current = BGVAR(u32_Fb_Motor_Rated_Current);
       p_fieldbus_bg_tx_data[0] = (p402_motor_rated_current & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_motor_rated_current >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6075S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

void FalUpdateMotorRatedCurrentCommandBg(unsigned long u32_data,int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int s16_tmp;
   // if this object is saved in MTP (motor name plate), it cannot be modified.
   // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
   if ((BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3))
   {
      return;
   }

   BGVAR(u32_Fb_Motor_Rated_Current) = u32_data;
   SalMotorIContCommand(BGVAR(u32_Fb_Motor_Rated_Current), drive);

   // attempt to perform config
   // after meeting at: 18/3/2013, it was decided that:
   // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
   s16_tmp = 0x0101;
   FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
}



//**********************************************************
// Function Name: FalMotorRatedTorqueCommandBg
// Description:
//          This function is called in response object 0x6076 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMotorRatedTorqueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long  u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   float f_tmp = 0.0;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      p402_motor_rated_torque = u32_tmp;
      ConvertFbCanOpenTorque(drive);

      //Update 0x6075
     f_tmp = (float)((float)u32_tmp/((float)BGVAR(u32_Motor_Kt)/1000.0));
      FalUpdateMotorRatedCurrentCommandBg((unsigned long)f_tmp,s16_msg_id,u16_operation);

      ret_val = FalInitDownloadRespond((int)SDO6076S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p402_motor_rated_torque & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_motor_rated_torque >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO6076S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalMotorPeakCurrentCommandBg
// Description:
//          This function is called in response object 0x2036 (Motor Peak Current) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalMotorPeakCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_tmp;

   if(u16_operation == 1)// write operation
   {
       // if this object is saved in MTP (motor name plate), it cannot be modified.
       // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
       if ((BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3))
       {
        return MTP_USED;
      }

       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       u32_tmp <<= 16;
       u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

       ret_val = SalMotorIPeakCommand((long long)u32_tmp, drive);

       if (ret_val != FAL_SUCCESS)
       {
            // restore original value
            manu_spec_Motor_I_Peak_command = BGVAR(s32_Motor_I_Peak);
            return ret_val;
       }
       manu_spec_Motor_I_Peak_command = u32_tmp;

       // attempt to perform config
       // after meeting at: 18/3/2013, it was decided that:
       // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
       // if config fail a fault will be generated
       s16_tmp = 0x0101;
       /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
       // if (ret_val != FAL_SUCCESS)
//        {
//              return ret_val;
//        }
       ret_val = FalInitDownloadRespond((int)SDO2036S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Motor_I_Peak_command = BGVAR(s32_Motor_I_Peak);             // both in mA
       p_fieldbus_bg_tx_data[0] = (manu_spec_Motor_I_Peak_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Motor_I_Peak_command >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO2036S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalDrivePeakCurrentCommandBg
// Description:
//          This function is called in response object 0x207B (Drive Peak Current) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalDrivePeakCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       u32_tmp <<= 16;
       u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

       ret_val = SalDriveIPeakCommand((long long)u32_tmp, drive);
       if (ret_val != FAL_SUCCESS)
       {
            // restore original value
            manu_spec_Drive_I_Peak_command = BGVAR(s32_Drive_I_Peak);
            return ret_val;
       }

       manu_spec_Drive_I_Peak_command = u32_tmp;
       ret_val = FalInitDownloadRespond((int)SDO207BS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Drive_I_Peak_command = BGVAR(s32_Drive_I_Peak);             // both in mA
       p_fieldbus_bg_tx_data[0] = (manu_spec_Drive_I_Peak_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Drive_I_Peak_command >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO207BS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalDriveContCurrentCommandBg
// Description:
//          This function is called in response object 0x207C (Drive Cont Current) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalDriveContCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       u32_tmp <<= 16;
       u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

       ret_val = SalDriveIContCommand((long long)u32_tmp, drive);
       if (ret_val != FAL_SUCCESS)
       {
            // restore original value
            manu_spec_Drive_I_Cont_command = BGVAR(s32_Drive_I_Cont);
            return ret_val;
       }

       manu_spec_Drive_I_Cont_command = u32_tmp;
       ret_val = FalInitDownloadRespond((int)SDO207CS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Drive_I_Cont_command = BGVAR(s32_Drive_I_Cont);             // both in mA
       p_fieldbus_bg_tx_data[0] = (manu_spec_Drive_I_Cont_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Drive_I_Cont_command >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO207CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalMaxCurrentCommandBg
// Description:
//          This function is called in response object 0x20F0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: nitsan: assign to object 0x20F0 instead of 0x60 73 (imax command). this is readonly object
//**********************************************************
int FalMaxCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, s16_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;    // avoid remark

   if(u16_operation == 1)// write operation
   {
       // object is read only
   /*
       // special handling for both axises
      if(Enabled(0))
         return (DRIVE_ACTIVE);

      //convert to internal FB current units
      s16_tmp = (int)MultS64ByFixS64ToS64((long long)(*(int*)p_data),
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

      //convert to user current units
      BGVAR(s32_Ilim_User) = (long)MultS64ByFixS64ToS64((long long)s16_tmp,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

      ret_val = SalIlimCommand((long long)BGVAR(s32_Ilim_User),drive);
        if(ret_val == FAL_SUCCESS)
     {
         //initiate config command
         s16_tmp = 0x0101;
         ret_val = FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         if(ret_val == FAL_SUCCESS)
              ret_val = FalInitDownloadRespond((int)SDO20F0S0, s16_msg_id);
     }
     */
   }
   else// read operation
   {
       //convert to internal current units
      s16_tmp = (int)MultS64ByFixS64ToS64((long long)BGVAR(s32_Imax),
                                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);


       //convert to user FB current units
       manu_spec_imax = (unsigned int)MultS64ByFixS64ToS64((long long)s16_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_imax;
       ret_val = FalInitUploadRespond(SDO20F0S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHomeCommandBg
// Description:
//          This function is called in response object 0x2103 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalHomeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   u16_operation += 0;

   if(u16_operation == 1)// write operation
   {
      s64_Execution_Parameter[0]=p_data[0];
      if (p_data[0]==1)   s16_Number_Of_Parameters=0; // If the user writes a 1, simulate a "HOMECMD" command without argument in order to start the homing.
      if (p_data[0]==0)   s16_Number_Of_Parameters=1; // If the user writes a 0, simulate a "HOMECMD 0" command with 1 argument in order to abort the homing.
      if (p_data[0] > 1) return VALUE_OUT_OF_RANGE;


      ret_val = HomeCommand(drive);

      if(ret_val != FAL_SUCCESS)
        return ret_val;
      else
         ret_val = FalInitDownloadRespond((int)SDO2103S0, s16_msg_id);
   }

   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_Home_Cmd_Command;
       ret_val = FalInitUploadRespond(SDO2103S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return(ret_val);
}

//**********************************************************
// Function Name: FalEstMotorParamsCommandBg
// Description:
//          This function is called in response object 0x214D sub_index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalEstMotorParamsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   u16_operation += 0;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO214DS1, s16_msg_id);
   }
   else
   {
      ret_val = SalEstMotorParamsCommand(1, drive);
      /*
      if (ret_val == SAL_SUCCESS)
      {
         DomainHandler(drive,&manu_spec_Est_Motor_Params_Command,0x214D,&SalEstMotorParamsStCommand,PACKETS_TRANSFER_ACTIVATE);
         p_fieldbus_bg_tx_data[0] = manu_spec_Est_Motor_Params_Command.StatusSelect;
         ret_val = FalInitUploadRespond((int)SDO214DS1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
      }
      */
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalCountsPerRevCommandBg
// Description:
//          This function is called in response object 0x2150 sub index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalCountsPerRevCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data++;

   if(u16_operation == 1)// write operation
   {
       // this is read only object
       return CO_E_NO_WRITE_PERM;
   }
   else// read operation
   {
       manu_spec_Counts_Per_Rev = s32_Sdo_Counts_Per_Rev;
       p_fieldbus_bg_tx_data[0] = (manu_spec_Counts_Per_Rev & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Counts_Per_Rev >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO2150S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalModuloLowerLimitCommandBg
// Description:
//          This function is called in response object 0x214F sub index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalModuloLowerLimitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       s32_tmp <<= 16;
       s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      s64_Execution_Parameter[0] = 1;
       s64_Execution_Parameter[1] = s32_tmp*1000;
       s16_Number_Of_Parameters = 2;

       SalPositionModuloCommand(drive);

       manu_spec_Mod_Limits[1] = s32_tmp;
       ret_val = FalInitDownloadRespond((int)SDO214FS1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (manu_spec_Mod_Limits[1] & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Mod_Limits[1] >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO214FS1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalModuloHigherLimitCommandBg
// Description:
//          This function is called in response object 0x214F sub index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalModuloHigherLimitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
       s32_tmp <<= 16;
       s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      s64_Execution_Parameter[0] = 2;
       s64_Execution_Parameter[1] = s32_tmp*1000;
      s16_Number_Of_Parameters = 2;

       SalPositionModuloCommand(drive);

       manu_spec_Mod_Limits[2] = s32_tmp;
       ret_val = FalInitDownloadRespond((int)SDO214FS2, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (manu_spec_Mod_Limits[2] & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Mod_Limits[2] >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO214FS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalEstMotorParamsStGrabCommandBg
// Description:
//          This function is called in response object 0x214D sub index 2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalEstMotorParamsStGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS,drive = 0;
   p_data += 0;
   u16_operation += 0;

   DomainHandler(drive,&manu_spec_Est_Motor_Params_Command,0x214D,NULL,DOMAIN_DATA_UPLOAD);

   if (IS_EC_DRIVE)
      p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_Est_Motor_Params_Command.Domain & 0xffff);
   ret_val = FalInitUploadRespond((int)(SDO214DS2), s16_msg_id, u16_Domain_Data_Length, (int*)p_fieldbus_bg_tx_data); //The length needs to be in Bytes
   return (ret_val);
}

//**********************************************************
// Function Name: FalKCMODECommandBg
// Description:
//          This function is called in response object 0x2106 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalKCMODECommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, s16_tmp=0;
   // AXIS_OFF;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
     ret_val = SalKCMODECommand((long long)p_data[0], drive);

      if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         s16_tmp = 0x0101;
         /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         //if(ret_val == FAL_SUCCESS)
              ret_val = FalInitDownloadRespond((int)SDO2106S0, s16_msg_id);
     }
   }
   else// read operation
   {
      manu_spec_CL_Kc_Mode_Command=VAR(AX0_S16_Kc_Mode);
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_CL_Kc_Mode_Command;
      ret_val = FalInitUploadRespond((SDO2106S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalMotorEncInterpolationCommandBg
// Description:
//          This function is called in response object 0x2107 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*int FalMotorEncInterpolationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, s16_tmp=0;
   p_data += 0;

   // The required variable is password protected.
   // Therefore the protection will be off and on by the end of the fal function(for read/write)
   if(u16_operation == 1)// write operation
   {
    ret_val = SalMotorEncInterpolationCommand((long long)p_data[0], drive);
     if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         s16_tmp = 0x0101;
          FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         //if(ret_val == FAL_SUCCESS)
              ret_val = FalInitDownloadRespond((int)SDO2107S0, s16_msg_id);
      }

   }
   else// read operation
   {
      manu_spec_Motor_Enc_Interpolation_Command=BGVAR(u16_Motor_Enc_Interpolation);
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Motor_Enc_Interpolation_Command;
      ret_val = FalInitUploadRespond((SDO2107S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}
*/
//**********************************************************
// Function Name: FalGearAccTreshCommandBg
// Description:
//          This function is called in response object 0x2120 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalGearAccTreshCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);


      //value must be between 0.06 rpm/sec to 1,000,000 rpm/sec
      if (s64_tmp > 0x44444444444444) // internal value for 1,000,000 rpm/sec
         return (VALUE_TOO_HIGH);

      if (s64_tmp < 0x100000000)// internal value for 0.23 rpm/sec
         return (VALUE_TOO_LOW);

      BGVAR(u64_Gear_Acc_Tresh) = (long long)s64_tmp;
      DesignGearMsqFilter(drive);

      ret_val = FalInitDownloadRespond((int)SDO2120S0, s16_msg_id);
      return (SAL_SUCCESS);
   }
   else// read operation
   {
      manu_spec_Gear_Acc_Tresh_Command = (long)MultS64ByFixS64ToS64((BGVAR(u64_Gear_Acc_Tresh)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Gear_Acc_Tresh_Command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Gear_Acc_Tresh_Command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2120S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalGearAccCommandBg
// Description:
//          This function is called in response object 0x211F from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*int FalGearAccCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;
   u16_operation += 0;
   manu_spec_Gear_Acc_Out_Command = LVAR(AX0_s32_Gear_Acc_Out);

   p_fieldbus_bg_tx_data[0] = (manu_spec_Gear_Acc_Out_Command & 0xFFFF); //set 16 LSBits
   p_fieldbus_bg_tx_data[1] = (manu_spec_Gear_Acc_Out_Command >> 16);    //set 16 MSBits

   ret_val = FalInitUploadRespond(SDO211FS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}
*/
//**********************************************************
// Function Name: FalNLPEAFFactCommandBg
// Description:
//          This function is called in response object 0x210E from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*int FalNLPEAFFactCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_temp_time;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      s32_tmp = (long)p_data[1];
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0];

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      LVAR(AX0_u32_Aff_Lo) = (unsigned long)(s64_tmp & 0x00000000FFFFFFFF);
      LVAR(AX0_s32_Aff_Hi) = (long)(s64_tmp >> 32LL);
      if(ret_val != FAL_SUCCESS)
          return ret_val;

      ret_val = FalInitDownloadRespond((int)SDO210ES0, s16_msg_id);
   }
   else// read operation
   {
      do
      {
         s16_temp_time = Cntr_3125;
         s64_tmp = ((long long)LVAR(AX0_s32_Aff_Hi)) << 32LL;
         s64_tmp |= (((long long)LVAR(AX0_u32_Aff_Lo)) & 0x00000000FFFFFFFF);
      } while (s16_temp_time != Cntr_3125);


      manu_spec_Aff_Command =  (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Aff_Command & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Aff_Command >> 16);    //set 16 MSBits


      ret_val = FalInitUploadRespond(SDO210ES0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
*/
//**********************************************************
// Function Name: FalStSelectCommandBg
// Description:
//          This function is called in response object 0x210F sub index 0 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*int FalNlKffSpringLPFCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   p_data += 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // The required variable is password protected.
   // Therefore the protection will be off and on by the end of the fal function(for read/write)
   if(u16_operation == 1)// write operation
   {
      BGVAR(s16_Nl_KffSpring_LPF)=p_data[0];
      //s16_Password_Flag = 0;
      ret_val = FalInitDownloadRespond((int)SDO210FS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Nl_KffSpring_LPF_Command=BGVAR(s16_Nl_KffSpring_LPF);
      //s16_Password_Flag = 0;
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Nl_KffSpring_LPF_Command;
      ret_val = FalInitUploadRespond((SDO210FS0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}
*/
//**********************************************************
// Function Name: FalStSelectCommandBg
// Description:
//          This function is called in response object 0x2110 sub index 0 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*int FalTQFWeightShlCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;

   // The required variable is password protected.
   // Therefore the protection will be off and on by the end of the fal function(for read/write)
   if(u16_operation == 1)// write operation
   {
      VAR(AX0_u16_TQF_Weight_Shl)=p_data[0];
      //s16_Password_Flag = 0;
      ret_val = FalInitDownloadRespond((int)SDO2110S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_TQF_Weight_Shl_Command=VAR(AX0_u16_TQF_Weight_Shl);
      //s16_Password_Flag = 0;
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_TQF_Weight_Shl_Command;
      ret_val = FalInitUploadRespond((SDO2110S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}
*/
//**********************************************************
// Function Name: FalNLStandStillDisableCommandBg
// Description:
//          This function is called in response object 0x2111 sub index 0 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
//Removed since it is passwprd protected. -ITAI
/*
int FalNLStandStillDisableCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;

   // The required variable is password protected.
   // Therefore the protection will be off and on by the end of the fal function(for read/write)
   if(u16_operation == 1)// write operation
   {
      VAR(AX0_u16_NL_StandStill_Disable)=p_data[0];
      //s16_Password_Flag = 0;
      ret_val = FalInitDownloadRespond((int)SDO2111S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_TQF_Weight_Shl_Command=VAR(AX0_u16_NL_StandStill_Disable);
      //s16_Password_Flag = 0;
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_TQF_Weight_Shl_Command;
      ret_val = FalInitUploadRespond((SDO2111S0), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}
*/
//**********************************************************
// Function Name: FalStSelectCommandBg
// Description:
//          This function is called in response object 0x2114 sub index 1 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
      p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO2114S1, s16_msg_id);
   }
   else// read operation
   {
      DomainHandler(drive,&manu_spec_ST_Command,0x2114,StCommand,PACKETS_TRANSFER_ACTIVATE);
     p_fieldbus_bg_tx_data[0] = manu_spec_ST_Command.StatusSelect;
      ret_val = FalInitUploadRespond((int)SDO2114S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalStGrabCommandBg
// Description:
//          This function is called in response object 0x2114 sub index 2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStGrabCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   p_data += 0;
   u16_operation += 0;

   DomainHandler(drive,(void *)&manu_spec_ST_Command,0x2114,NULL,DOMAIN_DATA_UPLOAD);

   if (IS_EC_DRIVE)
      p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_ST_Command.Domain & 0xffff);

   ret_val = FalInitUploadRespond((int)(SDO2114S2), s16_msg_id, u16_Domain_Data_Length, (int*)p_fieldbus_bg_tx_data); //The length needs to be in Bytes
   return (ret_val);
}

//**********************************************************
// Function Name: FalStepDuration1CommandBg
// Description:
//          This function is called in response object 0x2115  index 1 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepDuration1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
//   unsigned int u16_drive_table_idx ;

   p_data += 0;

   if(u16_operation == 1)// write operation
   {
      manu_spec_step_Command.Duration2=0;
      manu_spec_step_Command.Velocity1=0;
      manu_spec_step_Command.Velocity2=0;
      manu_spec_step_Command.Activate=0;

      retValue = FalInitDownloadRespond((int)SDO2115S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_step_Command.Duration1;     //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO2115S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   //restoring back the drive inactive check
//   FalGetDriveTableIndex(SDO2115S1, &u16_drive_table_idx);
//   Commands_Table[u16_drive_table_idx].u16_validity_checks |= 0x0020;
   return (retValue);
}
//**********************************************************
// Function Name: FalStepVel1CommandBg
// Description:
//          This function is called in response object 0x2115  index 2 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepVel1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO2115S2, s16_msg_id);
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = ((*(long*)&manu_spec_step_Command.Velocity1) & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = ((*(long*)&manu_spec_step_Command.Velocity1) >> 16);        //set 16 MSBits
      retValue = FalInitUploadRespond((int)SDO2115S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalStepDuration2CommandBg
// Description:
//          This function is called in response object 0x2115  index 3 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepDuration2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO2115S3, s16_msg_id);
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_step_Command.Duration2;     //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO2115S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}
//**********************************************************
// Function Name: FalStepVel2CommandBg
// Description:
//          This function is called in response object 0x2115  index 4 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepVel2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int retValue = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation == 1)// write operation
      retValue = FalInitDownloadRespond((int)SDO2115S4, s16_msg_id);
   else// read operation
   {

      p_fieldbus_bg_tx_data[0] = ((*(long*)&manu_spec_step_Command.Velocity2) & 0xFFFF);     //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = ((*(long*)&manu_spec_step_Command.Velocity2) >> 16);        //set 16 MSBits
      retValue = FalInitUploadRespond((int)SDO2115S4, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalStepActivateCommandBg
// Description:
//          This function is called in response object 0x2115  index 5 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepActivateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   int retValue = FAL_SUCCESS;
   int number_of_parameters=0;
   long long s64_Target_Vel[2]={0,0};

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;

   if(u16_operation == 1)// write operation
   {
      number_of_parameters=(manu_spec_step_Command.Duration1==0?0:1)+(manu_spec_step_Command.Velocity1==0?0:1)+(manu_spec_step_Command.Duration2==0?0:1)+ (manu_spec_step_Command.Velocity2==0?0:1);
      if ((number_of_parameters == 1) || (number_of_parameters > 4)) return SYNTAX_ERROR;
      if (number_of_parameters == 0) return (NOT_AVAILABLE);

     s64_Target_Vel[0]  = (long)MultS64ByFixS64ToS64(manu_spec_step_Command.Velocity1,
        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

     s64_Target_Vel[1]  = (long)MultS64ByFixS64ToS64(manu_spec_step_Command.Velocity2,
        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
        BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

     if (s64_Target_Vel[0] <= 0) return (VALUE_TOO_LOW);

     if (s64_Target_Vel[0] > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
     if (s64_Target_Vel[0] < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

       s32_FB_Step_Time1 = (long)manu_spec_step_Command.Duration1;
      s32_FB_Step_Jog1 = (long)s64_Target_Vel[0];

     if (s16_Number_Of_Parameters == 2)
     {
       s32_FB_Step_Time2 = 0;
      s32_FB_Step_Jog2 = 0;

     }
     else if (number_of_parameters == 3)
     {
        if (s64_Target_Vel[1] <= 0) return (VALUE_TOO_LOW);

       s32_FB_Step_Time2 = (long)manu_spec_step_Command.Duration2;
      s32_FB_Step_Jog2 = 0L;

     }
     else if (number_of_parameters == 4)
     {
        if (manu_spec_step_Command.Duration2 <= 0) return (VALUE_TOO_LOW);

        if (s64_Target_Vel[1]  > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
        if (s64_Target_Vel[1]  < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

         s32_FB_Step_Time2 = (long)manu_spec_step_Command.Duration2;
        s32_FB_Step_Jog2 = (long)s64_Target_Vel[1];
     }
     retValue = FalInitDownloadRespond((int)SDO2115S5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_step_Command.Activate;     //set 16 LSBits
      retValue = FalInitUploadRespond((int)SDO2115S5, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (retValue);
}

//**********************************************************
// Function Name: FalStepSelectorCommandBg
// Description:
//          This function is called in response object 0x2115  index 6 from Fieldbus.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalStepSelectorCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
  int ret_val = FAL_SUCCESS;
  int drive = 0;

  if(u16_operation == 1)// write operation
  {
     if (Enabled(drive) && p_data[0]!=0)
    {
       BGVAR(s32_Time1) = s32_FB_Step_Time1;
        BGVAR(s32_Jog1)  = s32_FB_Step_Jog1;
       BGVAR(s32_Time2) = s32_FB_Step_Time2;
        BGVAR(s32_Jog2)  = s32_FB_Step_Jog2;
        BGVAR(StepHandlerState) = STEP_JOG1;
      manu_spec_step_Command.Select = 0;
    }
     ret_val = FalInitDownloadRespond((int)SDO2115S6, s16_msg_id);

  }
  else
  {
     p_fieldbus_bg_tx_data[0] =manu_spec_step_Command.Select;     //set 16 LSBits
     ret_val = FalInitUploadRespond((SDO2115S6), s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
  }
  return (ret_val);
}



//**********************************************************
// Function Name: FalCurrentLimitCommandBg
// Description:
//          This function is called in response object 0x6073 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalCurrentLimitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned long long u64_tmp;

   if(u16_operation == 1)// write operation
   {
       //u64_tmp = (((unsigned long long)p_data[0] * BGVAR(s32_FB_Crrnt_To_User_Fix) + BGVAR(s64_FB_To_User_Half_For_Round))>> BGVAR(u16_FB_Crrnt_To_User_Shr));
       //if (u64_tmp > (unsigned long long)BGVAR(s32_Imax)) return VALUE_TOO_HIGH;

       // convert to current internal units
       u64_tmp = (unsigned long long)MultS64ByFixS64ToS64((long long)(*(int*)p_data),
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                        BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

       // convert to mA
       u64_tmp = (unsigned long long)MultS64ByFixS64ToS64((long long)u64_tmp,
                        BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                        BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);


       ret_val = SalIlimCommand((long long)u64_tmp, drive);

       if(ret_val == FAL_SUCCESS)
          ret_val = FalInitDownloadRespond((int)SDO6073S0, s16_msg_id);
   }
   else// read operation
   {
      p402_max_current = (unsigned int)(((long long)((long long)(BGVAR(s32_Ilim_User)) * (long long)BGVAR(s32_User_Crrnt_To_FB_Fix)) + BGVAR(s64_User_To_FB_Half_For_Round)) >> BGVAR(u16_User_Crrnt_To_FB_Shr));
      p_fieldbus_bg_tx_data[0] = p402_max_current;
      ret_val = FalInitUploadRespond(SDO6073S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//***********************************************************
// Function Name: FalCurrentLimitCommandRt
// Description:  PDO function for object (0x6073)
// Author: Gil
// Algorithm:
// Revisions:

//**********************************************************
#pragma CODE_SECTION(FalCurrentLimitCommandRt, "ramcan");
int  FalCurrentLimitCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long  s32_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   s32_tmp = (long)(((unsigned long long)p_data[0] * BGVAR(s32_FB_Crrnt_To_User_Fix) + BGVAR(s64_FB_To_User_Half_For_Round))>> BGVAR(u16_FB_Crrnt_To_User_Shr));


   //new
//   s32_tmp = (long)(((unsigned long long)p_data[0] * BGVAR(s64_Can_Current_To_User_mA_Fix) + BGVAR(s64_Can_Current_To_User_mA_Round))>> BGVAR(u16_Can_Current_To_User_mA_Shift));


//s32_tmp_torque = (long)((((long long)(p_data[0]) * (long long)LVAR(AX0_s32_Current_To_Internal_Fix)) + LLVAR(AX0_u32_Current_To_Internal_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_Internal_Shr));
   if (s32_tmp > BGVAR(s32_Imax)) return VALUE_TOO_HIGH;

   BGVAR(s32_Ilim_User) = s32_tmp;
   BGVAR(s32_Ilim_Active_Disable) = BGVAR(s32_Ilim_User);

   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalActualTorqueCommandBg
// Description:
//          This function is called in response object 0x6077 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
/* nitsan
int FalActualTorqueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO6077S0, s16_msg_id);
   }
   else// read operation
   {
      p402_torque_actual_value = (int)MultS64ByFixS64ToS64((long long)BGVAR(u32_EqCrrnt),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = p402_torque_actual_value;
      ret_val = FalInitUploadRespond(SDO6077S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}
*/

//**********************************************************
// Function Name: FalActualTorqueCommandBg
// Description:
//          This function is called in response object 0x6077 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:FalActualTorqueCommandBg
//**********************************************************
int FalActualTorqueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO6077S0, s16_msg_id);
   }
   else// read operation
   {
      p402_torque_actual_value = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_Crrnt_Q_Act_0),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = p402_torque_actual_value;
       ret_val = FalInitUploadRespond(SDO6077S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHaltOptionCodeCommandBg
// Description:
//          This function is called in response object 0x605D from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalHaltOptionCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      if (p_data[0] > 3 || p_data[0] <= 0) return (VALUE_OUT_OF_RANGE);
         ret_val = FalInitDownloadRespond((int)SDO605DS0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_halt_option_code;
       ret_val = FalInitUploadRespond(SDO605DS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalShutdownOptionCodeCommandBg
// Description:
//          This function is called in response object 0x605B from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalShutdownOptionCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   //p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      if (((int)p_data[0] > 1) || ((int)p_data[0] < -1))
         return (VALUE_OUT_OF_RANGE);

      ret_val = FalInitDownloadRespond((int)SDO605BS0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_shutdown_option_code;
       ret_val = FalInitUploadRespond(SDO605BS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDisableOperationOptionCodeCommandBg
// Description:
//          This function is called in response object 0x605C from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalDisableOperationOptionCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;


   if(u16_operation == 1)// write operation
   {
      if (((int)p_data[0] > 1) || ((int)p_data[0] < -1))
         return (VALUE_OUT_OF_RANGE);

      ret_val = FalInitDownloadRespond((int)SDO605CS0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_disable_operation_option_code;
       ret_val = FalInitUploadRespond(SDO605CS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalFaultOptionCodeCommandBg
// Description:
//          This function is called in response object 0x605E from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalFaultOptionCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      if (((int)p_data[0] > 1) || ((int)p_data[0] < -1))
         return (VALUE_OUT_OF_RANGE);

      ret_val = FalInitDownloadRespond((int)SDO605ES0, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_fault_reaction_option_code;
       ret_val = FalInitUploadRespond(SDO605ES0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPositionOptionCodeCommandBg
// Description:
//          This function is called in response object 0x60F2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalPositionOptionCodeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60F2S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_position_option_code;
      ret_val = FalInitUploadRespond(SDO60F2S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalActualCurrentCommandBg
// Description:
//          This function is called in response object 0x6078 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:FalActualCurrentCommandBg
//           nitsan: return value of IQ (AX0_s16_Crrnt_Q_Act_0) instead of I (u32_EqCrrnt).
//                   According to Avishay and Yuval, it is more correct to return IQ
//**********************************************************
int FalActualCurrentCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO6078S0, s16_msg_id);
   }
   else// read operation
   {
      p402_current_actual_value = (int)MultS64ByFixS64ToS64((long long)BGVAR(s32_EqCrrnt_Signed),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = p402_current_actual_value;
       ret_val = FalInitUploadRespond(SDO6078S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalActualPositionErrorCommandBg
// Description:
//          This function is called in response object 0x60F4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalActualPositionErrorCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS, s16_temp_time;
   // AXIS_OFF;
   long long s64_tmp = 0LL;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {

       ret_val = FalInitDownloadRespond((int)SDO60F4S0, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pos_Err_Lo);//(AX0_u32_Int_Loop_Pos_Err_Lo);
      } while (s16_temp_time != Cntr_3125);

      p402_following_error_actual_value = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_following_error_actual_value & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_following_error_actual_value >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO60F4S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//***********************************************************
// Function Name: FalFollowingErrorCommandRt
// Description:  PDO function for object 0x60F4 (Following Error Actual Value)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalFollowingErrorCommandRt, "ramcan");
int  FalFollowingErrorCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_Lo = 0L;
   long  s32_pos_Hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_following_error_actual_value = s32_tmp_pos;
   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalFollowingErrorWindowCommandBg
// Description:
//          This function is called in response object 0x6065 (Following Error Window) from Fieldbus.
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalFollowingErrorWindowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      // 0xffffFFFF means that following error window is switched off in CANopen standard
      // 0 is a valid window, so replace it with 1 to avoid disabling the feature in cdhd internal variable
      if (s32_tmp == 0xffffFFFF)
      {
        BGVAR(u64_Pe_Max) = 0ULL;
      }
      else if (s32_tmp == 0L)
      {
         BGVAR(u64_Pe_Max) = 1ULL;
         BGVAR(s16_CAN_Flags) |= CANOPEN_PEMAX_IS_ZERO_MASK;
      }
      else
      {
         BGVAR(s16_CAN_Flags) &= ~CANOPEN_PEMAX_IS_ZERO_MASK;
         BGVAR(u64_Pe_Max) = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
      }

      ret_val = FalInitDownloadRespond((int)SDO6065S0, s16_msg_id);
   }
   else// read operation
   {
      // if pemax is set to zero then following error window is switched off (i.e. 0xffffFFFF)
      if (u64_Pe_Max == 0ULL)
      {
         p402_following_error_window = 0xffffFFFF;
      }
      else if ((u64_Pe_Max == 1ULL) && (BGVAR(s16_CAN_Flags) & CANOPEN_PEMAX_IS_ZERO_MASK))
      {
         p402_following_error_window = 0UL;
      }
      else
      {
         p402_following_error_window = (long)MultS64ByFixS64ToS64(BGVAR(u64_Pe_Max),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
      }

      p_fieldbus_bg_tx_data[0] = (p402_following_error_window & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_following_error_window >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6065S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalFollowingErrorWindowCommandRt
// Description: PDO handler for object 0x6065 (Following Error Window)
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalFollowingErrorWindowCommandRt, "ramcan");
int FalFollowingErrorWindowCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    unsigned long u32_followingErrVal = 0L;
    // AXIS_OFF;

    //  to prevent remark
    s16_msg_id += 0;
    //  to prevent remark
    u16_operation += 0;

    //  take data int at a time
    u32_followingErrVal = (unsigned long)(p_data[1] & 0xFFFF);
    u32_followingErrVal <<= 16;
    u32_followingErrVal |= (unsigned long)(p_data[0] & 0xFFFF);

    if (u32_followingErrVal == 0xFFFFFFFF)
    {
        BGVAR(u64_Pe_Max) = 0LL;
    }
    else
    {
        //  Setting the Low and High part of Tmp Pos with 64bit convertion
        LLVAR(AX0_u32_Tmp_Pos_Lo) = (long long)u32_followingErrVal;
        //  Units conversion
        CONV_FB_POSITION_UNITS_TO_INTERNAL();

        BGVAR(u64_Pe_Max) = LLVAR(AX0_u32_Tmp_Pos_Lo);
    }

    return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalFollowingTimeoutWindowCommandBg
// Description:
//          This function is called in response object 0x6066 (Following Error Timeout) from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalFollowingErrorTimeoutCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
 int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO6066S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_following_error_time_out;
      ret_val = FalInitUploadRespond(SDO6066S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalFollowingErrorTimeoutCommandRt
// Description: RPDO handler for object 0x6066 (Following Error Timeout)
//                  units are ms, so no conversion is required
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalFollowingErrorTimeoutCommandRt, "ramfunc_5");
int FalFollowingErrorTimeoutCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_following_error_time_out = p_data[0];
   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalTorqueSlopeCommandBg
// Description:
//          This function is called in response object 0x6087  from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTorqueSlopeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   long long s64_tmp;
   int ret_val = FAL_SUCCESS;
   //drive += 0;
   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
      u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // get value in internal units (convert from CAN units to internal units)
      s64_tmp = MultS64ByFixS64ToS64((long long)u32_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // get value in cdhd user units (convert from internal units to user units)
      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_user_fix,
               BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_user_shr);

      //ret_val = SalSetTorqueSlopeCommand(s64_tmp, drive);
      if (ret_val == FAL_SUCCESS)
          ret_val = FalInitDownloadRespond((int)SDO6087S0, s16_msg_id);
   }
   else// read operation
   {

      // get value in internal units (convert from user units to internal units)
      s64_tmp = MultS64ByFixS64ToS64((long long)BGVAR(u32_Icmd_Slp),
               BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // get value in CAN units (convert from user units to CAN units)
      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_user_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_user_shr);

      p402_torque_slope = (long)s64_tmp;
      p_fieldbus_bg_tx_data[0] = (p402_torque_slope & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_torque_slope >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO6087S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalTorqueSlopeCommandRt
// Description: PDO handler for object 0x6087 (Torque Slope)
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTorqueSlopeCommandRt, "ramcan");
int FalTorqueSlopeCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_slopeVal = 0L, s32_icmd_slp_sat = 0x7FFF0000; //Internal limit of current slope
   long s32_slopeVal_internal, s32_slopeVal_user;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // take data int at a time
   s32_slopeVal = (unsigned long)(p_data[1] & 0xFFFF);
   s32_slopeVal <<= 16;
   s32_slopeVal |= (unsigned long)(p_data[0] & 0xFFFF);

   // get value in internal units (convert from CAN units to internal units)
   s32_slopeVal_internal = (long)((((long long)(s32_slopeVal) * LVAR(AX0_s32_Crrnt_Per_Sec_To_Intrn_Fix))
                         + LLVAR(AX0_s64_Crrnt_Per_Sec_To_Intrn_Round)) >>  VAR(AX0_u16_Crrnt_Per_Sec_To_Intrn_Shr));

   // get value in cdhd user units (convert from internal units to user units)
   s32_slopeVal_user     = (long)((((long long)(s32_slopeVal_internal) * LVAR(AX0_s32_Intrn_To_Crrnt_Per_Sec_Fix))
                         + LLVAR(AX0_s64_Intrn_To_Crrnt_Per_Sec_Round)) >>  VAR(AX0_u16_Intrn_To_Crrnt_Per_Sec_Shr));

   if ((s32_slopeVal_user >= 1) && (s32_slopeVal_user <= 30000000LL))
   {
      BGVAR(u32_Icmd_Slp) = s32_slopeVal_user;
      LVAR(AX0_s32_Icmd_Slp) = (s32_slopeVal_internal > s32_icmd_slp_sat) ? s32_icmd_slp_sat : s32_slopeVal_internal;
   }
   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalInterpolationDataRecord1CommandBg
// Description:
//          This function is called in response object 0x60C1 sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationDataRecord1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60C1S1, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p402_interpolation_data_record[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_interpolation_data_record[1] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60C1S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalInterpolationDataRecord2CommandBg
// Description:
//          This function is called in response object 0x60C1 sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationDataRecord2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
 int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60C1S2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p402_interpolation_data_record[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_interpolation_data_record[2] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60C1S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalTouchProbeControlCommandBg
// Description:
//          This function is called in response object 0x60B8 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:

//**********************************************************
int FalTouchProbeControlCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    int ret_val = FAL_SUCCESS;
    unsigned int u16_temp=0;
    unsigned int u16_temp_config;
    //  Disabled
    int s16_enable_use = 0;
    //  one-time capture
    int s16_capture_method = 0;
    //  in-mode setting
    int s16_probe_source = 0;
    int s16_capture_edge = 0;

    // AXIS_OFF;
    //For Remark Avoidance
    p_data += 0;
    /////////////////////////////////////////////////////////
    //Pointing the probe source for position capture
    /////////////////////////////////////////////////////////
    // write operation
    if (u16_operation == 1)
    {
        /************************************************************/
        /*** From here we are adjusting the touch probe channel 1 ***/
        /************************************************************/

        VAR(AX0_u16_Temp_Cmd) = 0;
        u16_temp= p_data[0];

        if ((u16_temp & PROBE_FUNC_ENABLE_LO) != 0)
        {
            //  Enabled
            s16_enable_use = 1;
        }

        if ((u16_temp & PROBE_FUNC_CONTINUOUS_LO) != 0)
        {
            //  continous capture
            s16_capture_method = 1;
        }

        if ((u16_temp & PROBE_FUNC_DS402_INPUT_MSK_LO) == PROBE_FUNC_DS402_USE_0x60D0_LO)
        {
            //  Tell the "ProbeConfiguration" function to look at object 0x60D0
            s16_probe_source = 2;
        }
        else if ((u16_temp & PROBE_FUNC_INPUT_LO) != 0)
        {
            //  Tell the "ProbeConfiguration" function to look at the index signal
            s16_probe_source = 1;
        }

        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO) != 0)
        {
            //  Bit 0 = 1 --> Capture upon positive edge
            s16_capture_edge = 1;
        }
        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO) != 0)
        {
            //  Bit 1 = 1 --> Capture upon negative edge
            s16_capture_edge |= 2;
        }

        if ((ret_val = ProbeConfiguration(1 ,s16_enable_use, s16_capture_method, s16_probe_source,s16_capture_edge,PROBE_FAST_CONF)) != SAL_SUCCESS)
        {
            //  Return in case that there is a problem configuring the touch probe 1
            return ret_val;
        }

        u16_temp_config = VAR(AX0_u16_Temp_Cmd) & 0x00FF;

        /************************************************************/
        /*** From here we are adjusting the touch probe channel 2 ***/
        /************************************************************/
        //  Disabled
        s16_enable_use = 0;
        //  one-time capture
        s16_capture_method = 0;
        //  in-mode setting
        s16_probe_source = 0;
        s16_capture_edge = 0;

        if ((u16_temp & PROBE_FUNC_ENABLE_HI) != 0)
        {
            //  Enabled
            s16_enable_use = 1;
        }

        if ((u16_temp & PROBE_FUNC_CONTINUOUS_HI) != 0)
        {
            //  continous capture
            s16_capture_method = 1;
        }

        if((u16_temp & PROBE_FUNC_DS402_INPUT_MSK_HI) == PROBE_FUNC_DS402_USE_0x60D0_HI)
        {
            //  Tell the "ProbeConfiguration" function to look at object 0x60D0
            s16_probe_source = 2;
        }
        else if ((u16_temp & PROBE_FUNC_INPUT_HI) != 0)
        {
            //  Tell the "ProbeConfiguration" function to look at the index signal
            s16_probe_source = 1;
        }

        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI) != 0)
        {
            //  Bit 0 = 1 --> Capture upon positive edge
            s16_capture_edge = 1;
        }
        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI) != 0)
        {
            //  Bit 1 = 1 --> Capture upon negative edge
            s16_capture_edge |= 2;
        }

        if ((ret_val = ProbeConfiguration(2 ,s16_enable_use, s16_capture_method, s16_probe_source,s16_capture_edge,PROBE_FAST_CONF)) != SAL_SUCCESS)
        {
            //  Return in case that there is a problem configuring the touch probe 2
            return ret_val;
        }
        VAR(AX0_u16_Temp_Cmd) = (VAR(AX0_u16_Temp_Cmd) & 0xFF00) | u16_temp_config;
        VAR(AX0_u16_TProbe_Lock) |= PROBE_LOAD_CMND;
        ret_val = FalInitDownloadRespond((int)SDO60B8S0, s16_msg_id);
    }
    //  read operation
    else
    {
        p402_touch_probe_function= VAR(AX0_u16_TProbe_Cmnd);
        p_fieldbus_bg_tx_data[0] = p402_touch_probe_function;
        ret_val = FalInitUploadRespond(SDO60B8S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
    }

    return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeStatusCommandBg
// Description:
//          This function is called in response object 0x60B9 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeStatusCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60B9S0, s16_msg_id);
   }
   else// read operation
   {
      p402_touch_probe_status = VAR(AX0_u16_TProbe_Status);
      p_fieldbus_bg_tx_data[0] = VAR(AX0_u16_TProbe_Status);
      ret_val = FalInitUploadRespond(SDO60B9S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalInterpolationDataRecord3CommandBg
// Description:
//          This function is called in response object 0x60C1 sub 3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationDataRecord3CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
 int ret_val = FAL_SUCCESS;
   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60C1S3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p402_interpolation_data_record[3] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_interpolation_data_record[3] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60C1S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalInterpolationDataRecord4CommandBg
// Description:
//          This function is called in response object 0x60C1 sub 4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationDataRecord4CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
 int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60C1S4, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p402_interpolation_data_record[4] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_interpolation_data_record[4] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60C1S4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeValuePos1CommandBg
// Description:
//          This function is called in response object 0x60BA from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeValuePos1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60BAS0, s16_msg_id);
   }
   else// read operation
   {
      p402_touch_probe_1_positive_value = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Rise_Record).s64_position,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_touch_probe_1_positive_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_touch_probe_1_positive_value >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60BAS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbeValueNeg1CommandBg
// Description:
//          This function is called in response object 0x60BB from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeValueNeg1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60BBS0, s16_msg_id);
   }
   else// read operation
   {
      p402_touch_probe_1_negative_value = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Fall_Record).s64_position,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_touch_probe_1_negative_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_touch_probe_1_negative_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60BBS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeValuePos2CommandBg
// Description:
//          This function is called in response object 0x60BC from Fieldbus.
//          This object shall provide the position value of the touch probe 2 at positive edge.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeValuePos2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60BCS0, s16_msg_id);
   }
   else// read operation
   {
      p402_touch_probe_2_positive_value = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Rise_Record).s64_position,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_touch_probe_2_positive_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_touch_probe_2_positive_value >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO60BCS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbeValueNeg2CommandBg
// Description:
//          This function is called in response object 0x60BD from Fieldbus.
//          This object shall provide the position value of the touch probe 2 at negative edge.
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeValueNeg2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO60BDS0, s16_msg_id);
   }
   else// read operation
   {
      p402_touch_probe_2_negative_value = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Fall_Record).s64_position,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_touch_probe_2_negative_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_touch_probe_2_negative_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60BDS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbePosEdgePErr1CommandBg
// Description:
//          This function is called in response object 0x2147 sub_index 1 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgePErr1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;
   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2147S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise.PE_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Rise_Record).s64_position_err,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Rise.PE_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Rise.PE_INTRP >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2147S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbePosEdgePErr2CommandBg
// Description:
//          This function is called in response object 0x2184 sub_index 1 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbePosEdgePErr1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgePErr2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;
   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2184S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise_2.PE_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Rise_Record).s64_position_err,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Rise_2.PE_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Rise_2.PE_INTRP >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2184S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeNegEdgePErr1CommandBg
// Description:
//          This function is called in response object 0x2148 sub_index  1 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgePErr1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2148S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall.PE_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Fall_Record).s64_position_err,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Fall.PE_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Fall.PE_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2148S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeNegEdgePErr2CommandBg
// Description:
//          This function is called in response object 0x2185 sub_index  1 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbeNegEdgePErr1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgePErr2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2185S1, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall_2.PE_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Fall_Record).s64_position_err,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Fall_2.PE_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Fall_2.PE_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2185S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbePosEdgeVel1CommandBg
// Description:
//          This function is called in response object 0x2147 sub_index  2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgeVel1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2147S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise.V_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Rise_Record).s32_velocity,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Rise.V_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Rise.V_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2147S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}

//**********************************************************
// Function Name: FalTouchProbePosEdgeVel2CommandBg
// Description:
//          This function is called in response object 0x2184 sub_index  2 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbePosEdgeVel1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgeVel2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2184S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise_2.V_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Rise_Record).s32_velocity,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Rise_2.V_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Rise_2.V_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2184S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);

}
//**********************************************************
// Function Name: FalTouchProbeNegEdgeVel1CommandBg
// Description:
//          This function is called in response object 0x2148 sub_index  2 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgeVel1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2148S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall.V_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_1_Fall_Record).s32_velocity,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Fall.V_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Fall.V_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2148S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbeNegEdgeVel2CommandBg
// Description:
//          This function is called in response object 0x2185 sub_index  2 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbeNegEdgeVel1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgeVel2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2185S2, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall_2.V_INTRP = (long)MultS64ByFixS64ToS64(BGVAR(Probe_2_Fall_Record).s32_velocity,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Probed_Data_Fall_2.V_INTRP & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Probed_Data_Fall_2.V_INTRP >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2185S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbePosEdgeCurr1CommandBg
// Description:
//          This function is called in response object 0x2147 sub_index  3 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgeCurr1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;
   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2147S3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise.I_INTRP = (int)MultS64ByFixS64ToS64((long long)BGVAR(Probe_1_Rise_Record).s16_current,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr);
      p_fieldbus_bg_tx_data[0] = manu_spec_Probed_Data_Rise.I_INTRP;
      ret_val = FalInitUploadRespond(SDO2147S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbePosEdgeCurr2CommandBg
// Description:
//          This function is called in response object 0x2184 sub_index  3 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbePosEdgeCurr1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbePosEdgeCurr2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;
   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2184S3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Rise_2.I_INTRP = (int)MultS64ByFixS64ToS64((long long)BGVAR(Probe_2_Rise_Record).s16_current,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr);
      p_fieldbus_bg_tx_data[0] = manu_spec_Probed_Data_Rise_2.I_INTRP;
      ret_val = FalInitUploadRespond(SDO2184S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbeNegEdgeCurr1CommandBg
// Description:
//          This function is called in response object 0x2148 sub_index  3 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgeCurr1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2148S3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall.I_INTRP = (int)MultS64ByFixS64ToS64((long long)BGVAR(Probe_1_Fall_Record).s16_current,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr);
      p_fieldbus_bg_tx_data[0] = manu_spec_Probed_Data_Fall.I_INTRP;
      ret_val = FalInitUploadRespond(SDO2148S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
//**********************************************************
// Function Name: FalTouchProbeNegEdgeCurr2CommandBg
// Description:
//          This function is called in response object 0x2185 sub_index  3 from Fieldbus.
//          The function is the equivalent of the "FalTouchProbeNegEdgeCurr1CommandBg" function,
//          implemented by Gil.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeNegEdgeCurr2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
      ret_val = FalInitDownloadRespond((int)SDO2185S3, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probed_Data_Fall_2.I_INTRP = (int)MultS64ByFixS64ToS64((long long)BGVAR(Probe_2_Fall_Record).s16_current,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr);
      p_fieldbus_bg_tx_data[0] = manu_spec_Probed_Data_Fall_2.I_INTRP;
      ret_val = FalInitUploadRespond(SDO2185S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeVarSelectCommandBg
// Description:
//          This function is called in response object 0x2149 from Fieldbus.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalTouchProbeVarSelectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   // AXIS_OFF;
   p_data += 0; //For Remark Avoidance
   u16_operation += 0;//For Remark Avoidance

   if(u16_operation == 1)// write operation
   {
       VAR(AX0_u16_TProbe_Src)= p_data[0];
       ret_val = FalInitDownloadRespond((int)SDO2149S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Probe_Var_Select =  VAR(AX0_u16_TProbe_Src);
      p_fieldbus_bg_tx_data[0] = manu_spec_Probe_Var_Select;
      ret_val = FalInitUploadRespond(SDO2149S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalTouchProbeControlCommandRt
// Description: PDO handler for object 0x60B8
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeControlCommandRt, "ramcan_2");
int FalTouchProbeControlCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
    unsigned int u16_temp = 0;
    unsigned int u16_temp_config;
    int s16_enable_use = 0;
    int s16_capture_method = 0;
    int s16_probe_source = 0;
    int s16_capture_edge = 0;
    p_data += 0;
    // AXIS_OFF;

    //  only if there is a change in control word
    if (u16_temp != u16_raw_data_tprobe_ctrl)
    {
        //  to prevent remark
        s16_msg_id += 0;
        //  to prevent remark
        u16_operation += 0;
        VAR(AX0_u16_Temp_Cmd)= 0;

        if ((u16_temp & PROBE_FUNC_ENABLE_LO) != 0)
        {
            //  Enabled
            s16_enable_use = 1;
        }

        if ((u16_temp & PROBE_FUNC_CONTINUOUS_LO) != 0)
        {
            //  continous capture
            s16_capture_method = 1;
        }

        if ((u16_temp & PROBE_FUNC_DS402_INPUT_MSK_LO) == PROBE_FUNC_DS402_USE_0x60D0_LO)
        {
            //  Tell the "ProbeConfiguration" function to look at object 0x60D0
            s16_probe_source = 2;
        }
        else if ((u16_temp & PROBE_FUNC_INPUT_LO) != 0)
        {
            //  Tell the "ProbeConfiguration" function to look at the index signal
            s16_probe_source = 1;
        }

        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_LO) != 0)
        {
            //  Bit 0 = 1 --> Capture upon positive edge
            s16_capture_edge = 1;
        }
        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_LO) != 0)
        {
            //  Bit 1 = 1 --> Capture upon negative edge
            s16_capture_edge |= 2;
        }

        ProbeConfiguration(1 ,s16_enable_use, s16_capture_method, s16_probe_source,s16_capture_edge,PROBE_FAST_CONF);
        u16_temp_config = VAR(AX0_u16_Temp_Cmd) & 0x00FF;

        s16_enable_use = 0;
        s16_capture_method = 0;
        s16_probe_source = 0;
        s16_capture_edge = 0;

        if ((u16_temp & PROBE_FUNC_ENABLE_HI) != 0)
        {
            //  Enabled
            s16_enable_use = 1;
        }

        if ((u16_temp & PROBE_FUNC_CONTINUOUS_HI) != 0)
        {
            //  continous capture
            s16_capture_method = 1;
        }

        if ((u16_temp & PROBE_FUNC_DS402_INPUT_MSK_HI) == PROBE_FUNC_DS402_USE_0x60D0_HI)
        {
            //  Tell the "ProbeConfiguration" function to look at object 0x60D0
            s16_probe_source = 2;
        }
        else if ((u16_temp & PROBE_FUNC_INPUT_HI) != 0)
        {
            //  Tell the "ProbeConfiguration" function to look at the index signal
            s16_probe_source = 1;
        }

        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_POS_EDGE_HI) != 0)
        {
            //  Bit 0 = 1 --> Capture upon positive edge
            s16_capture_edge = 1;
        }
        if ((u16_temp & PROBE_FUNC_ENABLE_SAMPLE_NEG_EDGE_HI) != 0)
        {
            //  Bit 1 = 1 --> Capture upon negative edge
            s16_capture_edge |= 2;
        }

        ProbeConfiguration(2 ,s16_enable_use, s16_capture_method, s16_probe_source,s16_capture_edge,PROBE_FAST_CONF);
        VAR(AX0_u16_Temp_Cmd) = (VAR(AX0_u16_Temp_Cmd) & 0xFF00) | u16_temp_config;
        VAR(AX0_u16_TProbe_Lock) |= PROBE_LOAD_CMND;
        u16_raw_data_tprobe_ctrl = u16_temp;
    }

    return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeValuePos1CommandRt
// Description: Service function for PDO60BA
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeValuePos1CommandRt, "ramcan_2");
int  FalTouchProbeValuePos1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_Lo = 0L;
   long  s32_pos_Hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   p_data+= 0;         //to prevent remark

   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_touch_probe_1_positive_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeValuePos2CommandRt
// Description: Service function for PDO60BC
// This object shall provide the position value of the touch probe 2 at positive edge.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************

#pragma CODE_SECTION(FalTouchProbeValuePos2CommandRt, "ramcan_2");
int  FalTouchProbeValuePos2CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_Lo = 0L;
   long  s32_pos_Hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_touch_probe_2_positive_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbePosEdgePErr1CommandRt
// Description:
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbePosEdgePErr1CommandRt, "ramcan_2");
int  FalTouchProbePosEdgePErr1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_err_lo = 0L;
   long  s32_pos_err_hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   u32_pos_err_lo =  p_data[1];
   u32_pos_err_lo <<= 16;
   u32_pos_err_lo |= ((unsigned long)p_data[0] & 0xFFFF);


   s32_pos_err_hi =  p_data[3];
   s32_pos_err_hi <<= 16;
   s32_pos_err_hi |= ((long)p_data[2] & 0xFFFF);

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_err_lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_err_hi);

   p402_touch_probe_1_positive_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalSfbPfbPeCommandRt
// Description: 2168 PDO
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalSfbPfbPeCommandRt, "ramcan");
int  FalSfbPfbPeCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_err_lo = 0L;
   long  s32_pos_err_hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   u32_pos_err_lo =  p_data[1];
   u32_pos_err_lo <<= 16;
   u32_pos_err_lo |= ((unsigned long)p_data[0] & 0xFFFF);


   s32_pos_err_hi =  p_data[3];
   s32_pos_err_hi <<= 16;
   s32_pos_err_hi |= ((long)p_data[2] & 0xFFFF);

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_err_lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_err_hi);

   manu_spec_Sfb_Pfb_Pe_Command = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeNegEdgePErr1CommandRt
// Description:
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeNegEdgePErr1CommandRt, "ramcan_2");
int  FalTouchProbeNegEdgePErr1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_err_lo = 0L;
   long  s32_pos_err_hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark



   //we need to take data int by int to prevent bad data due to odd address
   u32_pos_err_lo =  p_data[1];
   u32_pos_err_lo <<= 16;
   u32_pos_err_lo |= ((unsigned long)p_data[0] & 0xFFFF);


   s32_pos_err_hi =  p_data[3];
   s32_pos_err_hi <<= 16;
   s32_pos_err_hi |= ((long)p_data[2] & 0xFFFF);

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_err_lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_err_hi);

   p402_touch_probe_1_positive_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeValueNeg1CommandRt
// Description: Service function for PDO60BB
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeValueNeg1CommandRt, "ramcan_2");
int  FalTouchProbeValueNeg1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_Lo = 0L;
   long  s32_pos_Hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark



   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_touch_probe_1_negative_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalTouchProbeDataRiseCommandRt
// Description: Service function for PDO2147s3
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeDataRiseCommandRt, "ramcan_2");
int  FalTouchProbeDataRiseCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   p_data += 0;        //to prevent remark

   manu_spec_Probed_Data_Rise.I_INTRP = BGVAR(Probe_1_Rise_Record).s16_current;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeDataFallCommandRt
// Description: Service function for PDO2148s3
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeDataFallCommandRt, "ramcan_2");
int  FalTouchProbeDataFallCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   p_data += 0;        //to prevent remark

   manu_spec_Probed_Data_Fall.I_INTRP = BGVAR(Probe_1_Fall_Record).s16_current;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbeValueNeg2CommandRt
// Description: Service function for PDO60BD
// This object shall provide the position value of the touch probe 2 at negative edge.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeValueNeg2CommandRt, "ramcan_2");
int  FalTouchProbeValueNeg2CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos;
   unsigned long  u32_pos_Lo = 0L;
   long  s32_pos_Hi = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark



   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_touch_probe_2_negative_value = s32_tmp_pos;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbePosEdgeVel1CommandRt
// Description:
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbePosEdgeVel1CommandRt , "ramcan_2");
int  FalTouchProbePosEdgeVel1CommandRt (int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_vel = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp_vel =  (long)p_data[1];
   s32_tmp_vel <<= 16;
   s32_tmp_vel |= ((unsigned long)p_data[0] & 0xFFFF);

   //units conversion
   manu_spec_Probed_Data_Rise.V_INTRP = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_Out_Loop_To_User_Fix)) + LLVAR(AX0_u32_Vel_Out_Loop_Half_For_Round_To_User_Lo)) >> VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr));

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalTouchProbeNegEdgeVel1CommandRt
// Description:
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeNegEdgeVel1CommandRt , "ramfunc_4");
int  FalTouchProbeNegEdgeVel1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_vel = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp_vel =  (long)p_data[1];
   s32_tmp_vel <<= 16;
   s32_tmp_vel |= ((unsigned long)p_data[0] & 0xFFFF);

   //units conversion
   manu_spec_Probed_Data_Fall.V_INTRP = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_Out_Loop_To_User_Fix)) + LLVAR(AX0_u32_Vel_Out_Loop_Half_For_Round_To_User_Lo)) >> VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr));

   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalTargetPositionCommandBg
// Description:
//          This function is called in response object 0x607A from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTargetPositionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         BGVAR(s64_Fb_Target_Position) = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

         ret_val = FalInitDownloadRespond((int)SDO607AS0, s16_msg_id);
   }
   else// read operation
   {
         s32_tmp = p402_target_position;
         p_fieldbus_bg_tx_data[0] = (s32_tmp & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1] = (s32_tmp >> 16);    //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO607AS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalPositionOffsetCommandBg
// Description:
//          This function is called in response object 0x60B0 (position ffset) from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalPositionOffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int ret_val = FAL_SUCCESS;
   long long s64_tmp;
   unsigned int u16_temp_time;
   long s32_offset_hi;
   unsigned long u32_offset_lo;

   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

       LVAR(AX0_s32_Pcmd_Offset_Hi) = s64_tmp >> 32;
       LVAR(AX0_u32_Pcmd_Offset_Lo) = (unsigned long)s64_tmp;

         ret_val = FalInitDownloadRespond((int)SDO60B0S0, s16_msg_id);
   }
   else// read operation
   {
        do
        {
           u16_temp_time = Cntr_3125; // the same RT Interrupt
           s32_offset_hi = LVAR(AX0_s32_Pcmd_Offset_Hi);
           u32_offset_lo = LVAR(AX0_u32_Pcmd_Offset_Lo);
        } while (u16_temp_time != Cntr_3125);

        s64_tmp = ((long long)s32_offset_hi << 32) | (long long)u32_offset_lo;
        p402_position_offset = (long)MultS64ByFixS64ToS64((s64_tmp),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

         p_fieldbus_bg_tx_data[0] = (p402_position_offset & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1] = (p402_position_offset >> 16);    //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO60B0S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalSecondaryFeedbackModeBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   int s16_tmp = 0;
   unsigned int u16_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = p_data[0] & 0xff;
       
       ret_val = SalSFBModeCommand((long long)u16_tmp, drive);
       manu_spec_Sfb_Mode = BGVAR(s16_SFBMode);
       if(ret_val == FAL_SUCCESS)
       {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         s16_tmp = 0x0101;
         /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO2139S0, s16_msg_id);
       }
   }
   else     // read operation
   {
       p_fieldbus_bg_tx_data[0] = BGVAR(s16_SFBMode);
       ret_val = FalInitUploadRespond(SDO2139S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);   
}

//**********************************************************
// Function Name: FalSecondaryFeedbackOffsetCommandBg
// Description:
//          This function is called in response object 0x213f (Secondary Feedback Offset) from Fieldbus.
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
//          Is not used. 0x2162 is used instead
int FalSecondaryFeedbackOffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int ret_val = FAL_SUCCESS;
   long long s64_tmp;
   unsigned int u16_temp_time;
   long s32_offset_hi;
   unsigned long u32_offset_lo;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

 //        SalSfbOffCommand(s64_tmp, drive);
         ret_val = FalInitDownloadRespond((int)SDO213FS0, s16_msg_id);
   }
   else// read operation
   {
        do
        {
           u16_temp_time = Cntr_3125; // the same RT Interrupt
           s32_offset_hi = LVAR(AX0_s32_Sfb_Pos_Fdbk_Offset_Hi);
           u32_offset_lo = LVAR(AX0_u32_Sfb_Pos_Fdbk_Offset_Lo);
        } while  (u16_temp_time != Cntr_3125);


        s64_tmp = ((long long)s32_offset_hi << 32) | (long long)u32_offset_lo;
        manu_spec_Sfb_Offset = (long)MultS64ByFixS64ToS64((s64_tmp),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

         p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Offset & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Offset >> 16);    //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO213FS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalSecondaryFeedbackPeMaxCommandBg
// Description:
//          This function is called in response object 0x2144 (Secondary Feedback PEMAX) from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
//          Is not used. 0x2163 is used instead
int FalSecondaryFeedbackPeMaxCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(u64_Sfb_Pe_Max) = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO2144S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Sfb_Pe_Max = (long)MultS64ByFixS64ToS64(BGVAR(u64_Sfb_Pe_Max),
                               BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Pe_Max & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Pe_Max >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2144S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalSecondaryFeedbackThreshCommandBg
// Description:
//          This function is called in response object 0x2145 (Secondary Feedback THRESH) from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
//          Is not used. 0x2164 is used instead
int FalSecondaryFeedbackThreshCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      BGVAR(u64_Sfb_Pe_Thresh) = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = FalInitDownloadRespond((int)SDO2145S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Sfb_Pe_Thresh = (long)MultS64ByFixS64ToS64(BGVAR(u64_Sfb_Pe_Thresh),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Pe_Thresh & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Pe_Thresh >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2145S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalSecondaryFeedbackPositionActualValueCommandBg
// Description:
//          This function is called in response object 0x2140 (Secondary Feedback Position Actual Value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalSecondaryFeedbackPositionActualValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
    int ret_val = FAL_SUCCESS;
   long long s64_tmp = 0LL;
   unsigned int u16_temp_time;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO2140S0, s16_msg_id);
   }
   else// read operation
   {   //get SFB
      do {
         u16_temp_time = Cntr_3125; // the same RT Interrupt
         s64_tmp = LLVAR(AX0_u32_Pos_Fdbk_Lo_2);
      }while (u16_temp_time != Cntr_3125);

      manu_spec_Sfb_Pfb = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Pfb & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Pfb >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2140S0, s16_msg_id,2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalSecondaryFeedbackVelocityActualValueCommandBg
// Description:
//          This function is called in response object 0x2141 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalSecondaryFeedbackVelocityActualValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO2141S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Sfb_Vfb = (long)MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Sfb_Vel_Var_Fb_0),
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Vfb & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Vfb >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2141S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPrimaryPositionActualValueCommandBg
// Description:
//          This function is called in response object 0x2142 (Primary Position Actual Value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalPrimaryPositionActualValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   long long s64_tmp = 0LL;
   unsigned int u16_temp_time;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO2142S0, s16_msg_id);
   }
   else// read operation
   {  //get PFB
      do {
         u16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pfb_Internal_After_Mod_Lo);
      } while (u16_temp_time != Cntr_3125);

       manu_spec_Primary_Pfb = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Primary_Pfb & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Primary_Pfb >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2142S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPrimaryVelocityActualValueCommandBg
// Description:
//          This function is called in response object 0x2143 (Primary Velocity Actual Value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalPrimaryVelocityActualValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO2143S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Primary_Vfb = (long)MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Vel_Var_Fb_0),
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                               BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Primary_Vfb & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Primary_Vfb >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2143S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalPositionWindowCommandBg
// Description:
//          This function is called in response object 0x6067 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions: nitsan: remove s64_fb_position_window from code since it is not updated on startup and gives a wrong value
//**********************************************************
int FalPositionWindowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int ret_val = FAL_SUCCESS, s16_temp_time;
   // AXIS_OFF;
   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         // convert from internal units to PUU user units and update the p-param (to have the new value saved in p-param)
         s32_tmp = (unsigned long)MultS64ByFixS64ToS64(s64_tmp,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);

         BGVAR(u32_P1_54_PER_User_Value) = (unsigned long)s32_tmp;
      }

      do {
         s16_temp_time = Cntr_3125;
         LLVAR(AX0_u32_Inpos_Threshold_Lo) = s64_tmp;
      } while (s16_temp_time != Cntr_3125);
      ret_val = FalInitDownloadRespond((int)SDO6067S0, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Inpos_Threshold_Lo);
      } while (s16_temp_time != Cntr_3125);

      p402_position_window = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_position_window & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_position_window >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6067S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPositionWindowCommandRt
// Description: PDO handler for object 0x6067 (Position Window)
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalPositionWindowCommandRt, "ramcan");
int FalPositionWindowCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_posWindowVal = 0L;
   int s16_temp;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // take data int at a time
   u32_posWindowVal = (unsigned long)(p_data[1] & 0xFFFF);
   u32_posWindowVal <<= 16;
   u32_posWindowVal |= (unsigned long)(p_data[0] & 0xFFFF);

   //units conversion
   do {
      s16_temp = Cntr_3125;
      LLVAR(AX0_u32_Tmp_Pos_Lo) = (long long)u32_posWindowVal;
   } while (s16_temp != Cntr_3125);
   CONV_FB_POSITION_UNITS_TO_INTERNAL();
   do {
      s16_temp = Cntr_3125;
      LLVAR(AX0_u32_Inpos_Threshold_Lo) = LLVAR(AX0_u32_Tmp_Pos_Lo);
   } while (s16_temp != Cntr_3125);
   //u64_tmp = (long long)((((long long)(u32_posWindowVal) * (long long)LLVAR(AX0_u32_Pos_Fix_To_Internal_Lo)) /* + (long long)LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo) */ ) >> VAR(AX0_u16_Pos_Shr_To_Internal));

   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalPositionWindowTimeCommandBg
// Description:
//          This function is called in response object 0x6068 from Fieldbus.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalPositionWindowTimeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   if(u16_operation == 1)// write operation
   {
      VAR(AX0_u16_Inpos_Time) = p_data[0]=(unsigned int)p_data[0] & 0xFFFF;
      ret_val = FalInitDownloadRespond((int)SDO6068S0, s16_msg_id);
   }
   else// read operation
   {
      p402_position_window_time = VAR(AX0_u16_Inpos_Time);
      p_fieldbus_bg_tx_data[0] = p402_position_window_time;
      ret_val = FalInitUploadRespond(SDO6068S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalPolarityCommandBg
// Description:
//          This function is called in response object 0x607E (polarity) from Fieldbus.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalPolarityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   unsigned int u16_tmp;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      // object defined in canopen as uint8
      u16_tmp = (unsigned int)p_data[0] & 0x00FF;
      if (u16_tmp > 0xc0) // max in ds402
      {
          ret_val = VALUE_TOO_HIGH;
      }
      else if (Enabled(drive)) // to prevent hump on enabled
      {
         ret_val = DRIVE_ACTIVE;
      }
      else
      {
          p402_polarity = (unsigned int)p_data[0] & 0x00FF;
          VAR(AX0_s16_Pos_Polarity) = (p402_polarity & 0x80) ? (-1) : 1;
          VAR(AX0_s16_Vel_Polarity) = (p402_polarity & 0x40) ? (-1) : 1;
          ret_val = FalInitDownloadRespond((int)SDO607ES0, s16_msg_id);
      }
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_polarity;
      ret_val = FalInitUploadRespond(SDO607ES0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalHomeOffsetCommandBg
// Description:
//          This function is called in response object 0x607C from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHomeOffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)(p_data[1] & 0xFFFF);
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

         BGVAR(s64_Home_Offset) = MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

         ret_val = FalInitDownloadRespond((int)SDO607CS0, s16_msg_id);
   }
   else// read operation
   {
      p402_home_offset = (long)MultS64ByFixS64ToS64((BGVAR(s64_Home_Offset)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_home_offset & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_home_offset >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO607CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }


   return (ret_val);
}

//**********************************************************
// Function Name: FalAnOutCmdCommandBg
// Description:
//          This function is called in response object 0x2134 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnOutCmdCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      ret_val = SalAnalogOutCommand((long long)s32_tmp,drive);

      if(ret_val == FAL_SUCCESS)
      {
        FalInitDownloadRespond((int)SDO2134S0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_AnOutCommand = BGVAR(s32_Analog_Out_Volt_Cmd);

      p_fieldbus_bg_tx_data[0] = manu_spec_AnOutCommand & 0xFFFF;
      p_fieldbus_bg_tx_data[1] = manu_spec_AnOutCommand >> 16;

      ret_val = FalInitUploadRespond(SDO2134S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}
//**********************************************************
// Function Name: FalAnOutIscaleCommandBg
// Description:
//          This function is called in response object 0x2135 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnOutIscaleCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;
      s32_tmp = (long)(*(float*)(&s32_tmp) * 1000.0);

      s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).u16_unit_conversion_to_internal_shr);





      ret_val = SalIScaleOutCommand((long long)s32_tmp,drive);

      if(ret_val == FAL_SUCCESS)
      {
        FalInitDownloadRespond((int)SDO2135S0, s16_msg_id);
      }
   }
   else// read operation
   {
      s32_tmp =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(s32_Current_Scale_Out)),
                                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).u16_unit_conversion_to_user_shr);

      manu_spec_AnOutIscale = (float)s32_tmp * .001;

      p_fieldbus_bg_tx_data[0] = (*(long*)&manu_spec_AnOutIscale) & 0xFFFF;
      p_fieldbus_bg_tx_data[1] = (*(long*)&manu_spec_AnOutIscale) >> 16;

      ret_val = FalInitUploadRespond(SDO2135S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnOutVscaleCommandBg
// Description:
//          This function is called in response object 0x2138 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnOutVscaleCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   int drive = 0, ret_val = FAL_SUCCESS;


   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s64_tmp = (long)p_data[1] & 0xFFFF;
      s64_tmp <<= 16;
      s64_tmp |= (unsigned long)p_data[0] & 0xFFFF;
      s64_tmp = (long long)(*(float*)(&s64_tmp) * 1000.0);

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION]).u16_unit_conversion_to_internal_shr);


      ret_val = SalVScaleOutCommand(s64_tmp,drive);

      if(ret_val == FAL_SUCCESS)
      {
        FalInitDownloadRespond((int)SDO2138S0, s16_msg_id);
      }
   }
   else// read operation
   {
      s64_tmp =  MultS64ByFixS64ToS64((long long)(BGVAR(s32_Velocity_Scale_Out)),
                                  BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION]).u16_unit_conversion_to_user_shr);

      manu_spec_AnOutVscale = (float)s64_tmp * .001;

      p_fieldbus_bg_tx_data[0] = (*(long*)&manu_spec_AnOutVscale) & 0xFFFF;
      p_fieldbus_bg_tx_data[1] = (*(long*)&manu_spec_AnOutVscale) >> 16;

      ret_val = FalInitUploadRespond(SDO2138S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalMaxProfileVelocityCommandBg
// Description:
//          This function is called in response object 0x607F from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMaxProfileVelocityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS, s16_tmp = 0;
   long long s64_tmp;

   if(u16_operation == 1)// write operation
   {
        //we need to take data int by int to prevent bad data due to odd address
        s32_tmp = (long)(p_data[1] & 0xFFFF);
        s32_tmp <<= 16;
        s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

        // bugzilla 4057: fix erorr code for value 0xffffffff
        s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

        s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = SalVLimCommand(s64_tmp, drive);
      
      if (ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:

         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitUploadRequest((int)SDO2177S2,1, INIT_UPLOAD_REQUEST_MSG);

         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         return(FAL_SUCCESS);
      }
      if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         s16_tmp = 0x0101;
         /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
         //if(ret_val == FAL_SUCCESS)
              ret_val = FalInitDownloadRespond((int)SDO607FS0, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_max_profile_velocity =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(s32_V_Lim_Design)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_max_profile_velocity & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_max_profile_velocity >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO607FS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalProfileVelocityCommandBg
// Description:
//          This function is called in response object 0x6081 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: nitsan: limit vel according to vmax
//**********************************************************
int FalProfileVelocityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      s32_tmp = (long)MultS64ByFixS64ToS64((long long)(s32_tmp),
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // bug 3662: limit velocity according to vmax (object 607f)
      if (s32_tmp > BGVAR(s32_V_Lim_Design))
      {
         s32_tmp = BGVAR(s32_V_Lim_Design);
      }

      BGVAR(s32_Fb_Target_Velocity_For_Pos) = s32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO6081S0, s16_msg_id);
   }
   else// read operation
   {
      s32_tmp = p402_profile_velocity;

      p_fieldbus_bg_tx_data[0] = (s32_tmp & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (s32_tmp >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6081S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalVelocityDemandValueCommandBg
// Description:
//          This function is called in response object 0x606B (Velocity Demand Value) from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityDemandValueCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0;   // avoid remark
   if(u16_operation != 1)// read operation
   {
/* nitsan: need to finish solving bugzilla 3512 (opmode dependant
      // bugzilla 3512 - Object 0x606b (Velocity Demand Value) does not work in Profile Velocity mode.
      // 0x606b is used in profile position mode only as vcmd output of the generator
      p402_velocity_demand_value = (long)MultS64ByFixS64ToS64((long long)VAR(AX0_s32_Pos_Vcmd),
                            BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                            BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);
*/

      p402_velocity_demand_value = (long)MultS64ByFixS64ToS64((long long)(VAR(AX0_s16_Vel_Var_Ref_0)),
                            BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                            BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);


      p_fieldbus_bg_tx_data[0] = (p402_velocity_demand_value & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_velocity_demand_value >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO606BS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalMaxMotorSpeedCommandBg
// Description:
//          This function is called in response object 0x6080 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMaxMotorSpeedCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS, s16_tmp = 0;
   long s32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
      // if this object is saved in MTP (motor name plate), it cannot be modified.
      // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
      if ((BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3))
      {
        return MTP_USED;
     }

      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      s32_tmp = (long)MultS64ByFixS64ToS64((long long)(s32_tmp),
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = SalMspeedCommand((long long)s32_tmp,drive);
        if(ret_val == FAL_SUCCESS)
     {
        //initiate config command
        // after meeting at: 18/3/2013, it was decided that:
        // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
        // if config fail a fault will be generated
        s16_tmp = 0x0101;
        /*ret_val =*/ FalConfigCommandBg(&s16_tmp, s16_msg_id, u16_operation);
        //if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO6080S0, s16_msg_id);
     }
   }
   else// read operation
   {
       //write data
      p402_max_motor_speed =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(u32_Mspeed)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_max_motor_speed & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_max_motor_speed >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO6080S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: Can_SetAcc
// Description:
//          This function set acc.
//          input param: s32_acc: acceleration in canopen units
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int Can_SetAcc(long s32_acc, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_acc & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalAccCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
      BGVAR(u64_CAN_Acc) = s64_tmp;
   }

   return ret_val;
}

int Can_SetMotorAcc(long s32_acc, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_acc & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalMotorAccCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
      BGVAR(u64_CAN_Acc) = s64_tmp;
   }

   return ret_val;
}

int Can_SetSfbAcc(long s32_acc, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_acc & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalSfbAccCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
      BGVAR(u64_CAN_Acc) = s64_tmp;
   }

   return ret_val;
}

//**********************************************************
// Function Name: Can_SetDec
// Description:
//          This function set dec.
//          input param: s32_dec: acceleration in canopen units
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int Can_SetDec(long s32_dec, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_dec & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalDecCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
       BGVAR(u64_CAN_Dec) = s64_tmp;
   }

   return ret_val;
}

int Can_SetMotorDec(long s32_dec, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_dec & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalMotorDecCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
       BGVAR(u64_CAN_Dec) = s64_tmp;
   }

   return ret_val;
}

int Can_SetSfbDec(long s32_dec, int drive)
{
   int ret_val;
   long long s64_tmp;

   // bugzilla 4057: fix erorr code for value 0xffffffff
   s64_tmp = (long long)s32_dec & 0xffffffff; // zero upper 32 bit

   s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   ret_val = SalSfbDecCommand(s64_tmp, drive);
   if(ret_val == FAL_SUCCESS)
   {
       BGVAR(u64_CAN_Dec) = s64_tmp;
   }

   return ret_val;
}

//**********************************************************
// Function Name: FalProfileAccelerationCommandBg
// Description:
//          This function is called in response object 0x6083 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalProfileAccelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetAcc(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO6083S0, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_profile_acceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_AccRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_profile_acceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_profile_acceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO6083S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalProfileMotorAccelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetMotorAcc(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO21A8S0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_profile_motor_acceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_Mfb_AccRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_profile_motor_acceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_profile_motor_acceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO21A8S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalProfileSfbAccelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetSfbAcc(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO21A9S0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_profile_sfb_acceleration = (long)MultS64ByFixS64ToS64((BGVAR(u64_Sfb_AccRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_profile_sfb_acceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_profile_sfb_acceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO21A9S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalProfileAccelerationCommandRt
// Description: PDO handler for object 0x6083 (Profile Acceleration)
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalProfileAccelerationCommandRt, "ramcan");
int FalProfileAccelerationCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   unsigned long u32_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   u32_tmp = (long)(p_data[1] & 0xFFFF);
   u32_tmp <<= 16;
   u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);


   BGVAR(u32_CAN_Rpdo_Acc) = u32_tmp;
   BGVAR(s16_CAN_Flags) |= CANOPEN_ACC_CHANGED_MASK;
   return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalProfileDecelerationCommandRt
// Description: PDO handler for object 0x6084 (Profile Deceleration)
// Author: nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalProfileDecelerationCommandRt, "ramcan");
int FalProfileDecelerationCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   unsigned long u32_tmp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   u32_tmp = (long)(p_data[1] & 0xFFFF);
   u32_tmp <<= 16;
   u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);


   BGVAR(u32_CAN_Rpdo_Dec) = u32_tmp;
   BGVAR(s16_CAN_Flags) |= CANOPEN_DEC_CHANGED_MASK;
   return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalProfileDecelerationCommandBg
// Description:
//          This function is called in response object 0x6084 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalProfileDecelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetDec(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO6084S0, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_profile_deceleration =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(u64_DecRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_profile_deceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_profile_deceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO6084S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalProfileMotorDecelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetMotorDec(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO21AAS0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_profile_motor_deceleration =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(u64_Mfb_DecRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_profile_motor_deceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_profile_motor_deceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO21AAS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalProfileSfbDecelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = Can_SetSfbDec(s32_tmp, drive);
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO21ABS0, s16_msg_id);
      }
   }
   else// read operation
   {
      manu_spec_profile_sfb_deceleration =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(u64_Sfb_DecRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_profile_sfb_deceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_profile_sfb_deceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO21ABS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalQuickStopDecelerationCommandBg
// Description:
//          This function is called in response object 0x6085 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalQuickStopDecelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   long  s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);


     ret_val = CheckAccDecLimits(s64_tmp);
     if(ret_val == FAL_SUCCESS)
     {
      ret_val = SalDecStopCommand(s64_tmp,drive);
        if(ret_val == FAL_SUCCESS)
      {
         BGVAR(u64_CAN_Dec_Stop) = s64_tmp;
            ret_val = FalInitDownloadRespond((int)SDO6085S0, s16_msg_id);
      }
     }
   }
   else// read operation
   {
      p402_quick_stop_deceleration =  (long)MultS64ByFixS64ToS64((long long)(BGVAR(u64_CAN_Dec_Stop)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_quick_stop_deceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_quick_stop_deceleration >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO6085S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPosEncResEncoderIncrementsCommandBg
// Description:
//          This function is called in response object 0x608F index 1 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions: nitsan: make he object place holder, mencres moved to 0x20F1
//**********************************************************
int FalPosEncResEncoderIncrementsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;
   unsigned long s32_tmp = 0L;

   if(u16_operation == 1)  // write
   {
      // if this object is saved in MTP (motor name plate), it cannot be modified.
      // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
      if ((BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3))
      {
         return MTP_USED;
      }

      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // in meeting at 18/3/2013 was decided for now that 608F will be place holder
      // mencres will be set in a manufacturer specifice object

     // in meeting at 06/11/2013 was decided that 0x608F will be MENCRES

      ret_val = SalMotorEncResCommand((long long)s32_tmp, drive);

      if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         ret_val = 0x0101;
         /*ret_val =*/ FalConfigCommandBg(&ret_val, s16_msg_id, 1);
         //if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond(SDO608FS1, s16_msg_id);
      }
   }
   else     // read
   {
      // in meeting at 18/3/2013 was decided for now that 608F will be place holder
      // mencres will be set in a manufacturer specifice object

     // in meeting at 06/11/2013 was decided that 0x608F will be MENCRES
      /*
      s32_tmp = BGVAR(u32_User_Motor_Enc_Res) * p402_position_encoder_resolution[2];
      if(s32_tmp != p402_position_encoder_resolution[1])
      {
         p402_position_encoder_resolution[1] = s32_tmp;
      }
      else
         s32_tmp = p402_position_encoder_resolution[1];
      */
      s32_tmp = (long)BGVAR(u32_User_Motor_Enc_Res);
      p402_position_encoder_resolution[1] = s32_tmp;


      p_fieldbus_bg_tx_data[0] = s32_tmp & 0xFFFF; //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = s32_tmp >> 16;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO608FS1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPosEncResMotorRevolutionsCommandBg
// Description:
//          This function is called in response object 0x608F index 2 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalPosEncResMotorRevolutionsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long s32_tmp = 0L;
   *p_data += 0;
/*
   if(u16_operation == 1)
   {
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      ret_val = FalPosEncResolutionValidateAccess(SDO608FS2, s32_tmp);
      if(ret_val == FAL_SUCCESS)
         ret_val = SalMotorEncResCommand((long long)(p402_position_encoder_resolution[1] / s32_tmp), drive);

      if(ret_val == FAL_SUCCESS)
      {
         //initiate config command
         // after meeting at: 18/3/2013, it was decided that:
         // after setting the value try to pefrom config. if fail dont return error as the value itself was set properly.
         // if config fail a fault will be generated
         ret_val = 0x0101;
          FalConfigCommandBg(&ret_val, s16_msg_id, 1);
         //if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond(SDO608FS2, s16_msg_id);
      }
   }
   else
*/
   // read only object
   if(u16_operation != 1)  // read operation
   {
    /*  s32_tmp = p402_position_encoder_resolution[1] / BGVAR(u32_User_Motor_Enc_Res);
      if(s32_tmp != p402_position_encoder_resolution[2])
      {
         p402_position_encoder_resolution[2] = s32_tmp;
      }
      else
         s32_tmp = p402_position_encoder_resolution[2];

      p_fieldbus_bg_tx_data[0] = s32_tmp & 0xFFFF; //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = s32_tmp >> 16;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO608FS2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   */
      p402_position_encoder_resolution[2] = BGVAR(u32_Abs_Feedback_Max_Num_Of_Turns);
      s32_tmp = p402_position_encoder_resolution[2];
      p_fieldbus_bg_tx_data[0] = s32_tmp & 0xFFFF; //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = s32_tmp >> 16;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO608FS2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPosEncResolutionValidateAccess
// Description:
//          This function is responsible to validate the access of the 0x608F field bus object
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalPosEncResolutionValidateAccess(unsigned int u16_sub_id, long s32_data)
{
   long long s64_mencres = 0LL;
   unsigned int u16_drive_table_idx = 0;

   if((SDO608FS1 - u16_sub_id) == 0)
   {
      if((s32_data % p402_position_encoder_resolution[2]) != 0)
      {
         return CONFIG_FAIL_MENCRES;
      }
      s64_mencres = (long long)(s32_data / p402_position_encoder_resolution[2]);
   }
   else
   {
      if((p402_position_encoder_resolution[1] % s32_data) != 0)
      {
         return CONFIG_FAIL_MENCRES;
      }
      s64_mencres = (long long)(p402_position_encoder_resolution[1] / s32_data);
   }

   if(FalGetDriveTableIndex(SDO608FS1, &u16_drive_table_idx) != FAL_SUCCESS)
   {
      return FAL_INTERNAL_FW_FAULT;
   }

   if(s64_mencres > Commands_Table[u16_drive_table_idx].s64_max_value)
      return (CONFIG_FAIL_MENCRES);

   if(s64_mencres < Commands_Table[u16_drive_table_idx].s64_min_value)
      return (CONFIG_FAIL_MENCRES);

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalGearMotorShaftRevsCommandBg
// Description:
//          This function is called in response object 0x6091 index 1 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalGearMotorShaftRevsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      // IPR 796: Object for "Scaling factor" ?6092? cannot be written.
      // allow write pos scaling values even when NMT state is OPERATIONAL but only when drive is disabled
      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         if (Enabled(drive))
            ret_val = DRIVE_ACTIVE;
      }
      else if (FalIsInOperationMode() ) // This variable is not allowed to be wriiten on OPERATION mode
      {
         ret_val = FAL_WRONG_NMT_STATE;
      }

      if (ret_val == FAL_SUCCESS)
      {
         u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
         u32_tmp <<= 16;
         u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);                               //get the new motor shaft revolutions value

         ret_val = SalFbGearMotorShaftRevs((long long)u32_tmp, 0);
         if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO6091S1, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_gear_ratio[1]         = BGVAR(u32_Fb_Gear_Motor_Shaft_Revs);          //set the numerator value to the 0x6091 index 1 object
      p_fieldbus_bg_tx_data[0]   = (p402_gear_ratio[1] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (p402_gear_ratio[1] >> 16);       //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6091S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalGearDrivingShaftRevsCommandBg
// Description:
//          This function is called in response object 0x6091 index 2 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalGearDrivingShaftRevsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      // IPR 796: Object for "Scaling factor" ?6092? cannot be written.
      // allow write pos scaling values even when NMT state is OPERATIONAL but only when drive is disabled
      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         if (Enabled(drive))
            ret_val = DRIVE_ACTIVE;
      }
      else if (FalIsInOperationMode() ) // This variable is not allowed to be wriiten on OPERATION mode
      {
         ret_val = FAL_WRONG_NMT_STATE;
      }

      if (ret_val == FAL_SUCCESS)
      {
         u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
         u32_tmp <<= 16;
         u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);                                  //get the new driving shaft revolutions value

         ret_val = SalFbGearDrivingShaftRevs((long long)u32_tmp, 0);
         if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO6091S2, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_gear_ratio[2]         = BGVAR(u32_Fb_Gear_Driving_Shaft_Revs);     //set the denominator value to the 0x6091 index 2 object
      p_fieldbus_bg_tx_data[0]   = (p402_gear_ratio[2] & 0xFFFF);             //set 16 LSBits
         p_fieldbus_bg_tx_data[1]   = (p402_gear_ratio[2] >> 16);                //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO6091S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPNumCommandBg
// Description:
//          This function is called in response object 0x6092 index 1 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalPNumCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      // IPR 796: Object for "Scaling factor" ?6092? cannot be written.
      // allow write pos scaling values even when NMT state is OPERATIONAL but only when drive is disabled
      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         if (Enabled(drive))
            ret_val = DRIVE_ACTIVE;
      }
      else if (FalIsInOperationMode() ) // This variable is not allowed to be wriiten on OPERATION mode
      {
         ret_val = FAL_WRONG_NMT_STATE;
      }

      if (ret_val == FAL_SUCCESS)
      {
         u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
         u32_tmp <<= 16;
         u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);   //get the new pnum value

         ret_val = SalScalingPnumerator((long long)u32_tmp, 0);
         if(ret_val == FAL_SUCCESS)
         {
            // send pnum again to fix units calculations, fix bug IPR 538
            ret_val = SalScalingPnumerator((long long)u32_tmp, 0);
            if(ret_val == FAL_SUCCESS)
               ret_val = FalInitDownloadRespond((int)SDO6092S1, s16_msg_id);
         }
      }
   }
   else// read operation
   {
      p402_feed_constant[1]      = BGVAR(u32_Scaling_Pnumerator);    //set the denominator value to the 0x6092 index 1 object
      p_fieldbus_bg_tx_data[0]   = (p402_feed_constant[1] & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1]   = (p402_feed_constant[1] >> 16);    //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO6092S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPDenCommandBg
// Description:
//          This function is called in response object 0x6092 index 2 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalPDenCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long u32_tmp = 0UL;
   int drive = 0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      // IPR 796: Object for "Scaling factor" ?6092? cannot be written.
      // allow write pos scaling values even when NMT state is OPERATIONAL but only when drive is disabled
      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         if (Enabled(drive))
            ret_val = DRIVE_ACTIVE;
      }
      else if (FalIsInOperationMode() ) // This variable is not allowed to be wriiten on OPERATION mode
      {
         ret_val = FAL_WRONG_NMT_STATE;
      }

      if (ret_val == FAL_SUCCESS)
      {
         u32_tmp = (unsigned long)(p_data[1] & 0xFFFF);
         u32_tmp <<= 16;
         u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);    //get the new pden value

         ret_val = SalScalingPdenominator((long long)u32_tmp, 0);
         if(ret_val == FAL_SUCCESS)
            ret_val = FalInitDownloadRespond((int)SDO6092S2, s16_msg_id);
      }
   }
   else// read operation
   {
      p402_feed_constant[2]      = BGVAR(u32_Scaling_Pdenominator);     //set the denominator value to the 0x6092 index 2 object
      p_fieldbus_bg_tx_data[0]   = (p402_feed_constant[2] & 0xFFFF);    //set 16 LSBits
         p_fieldbus_bg_tx_data[1]   = (p402_feed_constant[2] >> 16);       //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO6092S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHomingMethodCommandBg
// Description:
//          This function is called in response object 0x6098 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHomingMethodCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_tmp = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // home mode is signed 8 bit
   s16_tmp = p_data[0];

   //move from 16 bits to 8 bits with sign extension
   s16_tmp <<= 8;
   s16_tmp >>= 8;

   if(u16_operation == 1)// write operation
   {

      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         // IPR 718: CANOPEN 6098 OBJECT: Limit homing methods to 0 to 35
         // in lxm32, minimum value is 1
         if (s16_tmp < 1) return VALUE_TOO_LOW;
         if (s16_tmp > 35) return VALUE_TOO_HIGH;
      }

      ret_val = SalHomeTypeCommand((long long)s16_tmp, 0);
      if (ret_val == FAL_SUCCESS)
      {
         ret_val = FalInitDownloadRespond((int)SDO6098S0, s16_msg_id);
//   ClearStatusWordMask = TARGET_REACHED|HM_ATTAINED|HM_ERROR;
      }
   }
   else// read operation
   {
      p402_homing_method = BGVAR(s16_Home_Type);
      p_fieldbus_bg_tx_data[0] = p402_homing_method;
      ret_val = FalInitUploadRespond(SDO6098S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHomingSpeed1CommandBg
// Description:
//          This function is called in response object 0x6099 index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: nitsan 14/7/2016: changed unit conversion to FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION
//                  since home speed value was double and since homespeed is used in position loop and not vel loop
//**********************************************************
int FalHomingSpeed1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   long long s64_tmp;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (long)(p_data[1] & 0xFFFF);
       s32_tmp <<= 16;
       s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = SalHomeSpeed1Command(s64_tmp,drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO6099S1, s16_msg_id);
   }
   else// read operation
   {
      p402_homing_speeds[1] = (unsigned long)MultS64ByFixS64ToS64((long long)BGVAR(u32_Home_Switch_Speed),
                                          BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                          BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_homing_speeds[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_homing_speeds[1] >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6099S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHomingSpeed2CommandBg
// Description:
//          This function is called in response object 0x6099 index 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: nitsan 14/7/2016: changed unit conversion to FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION
//                  since home speed value was double and since homespeed is used in position loop and not vel loop
//**********************************************************
int FalHomingSpeed2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   long long s64_tmp;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = (long long)MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = SalHomeSpeed2Command(s64_tmp,drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO6099S2, s16_msg_id);
   }
   else// read operation
   {
      p402_homing_speeds[2] = (unsigned long)MultS64ByFixS64ToS64((long long)BGVAR(u32_Home_Zero_Speed),
                                          BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                          BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_homing_speeds[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_homing_speeds[2] >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO6099S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHomingAccelerationCommandBg
// Description:
//          This function is called in response object 0x609A from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHomingAccelerationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long long s64_tmp = 0LL;
   long s32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)(p_data[1] & 0xFFFF);
      s32_tmp <<= 16;
      s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      // bugzilla 4057: fix erorr code for value 0xffffffff
      s64_tmp = (long long)s32_tmp & 0xffffffff; // zero upper 32 bit

      s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      ret_val = SalHomeAccCommand(s64_tmp,drive);
      if(ret_val == FAL_SUCCESS)
        ret_val = FalInitDownloadRespond((int)SDO609AS0, s16_msg_id);
   }
   else// read operation
   {
      p402_homing_acceleration =  (long)MultS64ByFixS64ToS64((BGVAR(u64_HomeAccRate)),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (p402_homing_acceleration & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_homing_acceleration >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO609AS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalFbInterpTimePeriod
// Description:
//          This function is called in response object 0x60C2 index 1 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFbInterpTimePeriod(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   unsigned char u8_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u8_tmp = (unsigned char)p_data[0] & 0x00FF;

      if(FalIsInOperationMode()) // This variable is not allowed to be wriiten on OPERATION mode
      {
         // dont allow changing if drive is enabled.
         if (Enabled(drive))
         {
            return DRIVE_ACTIVE;
         }

         // nitsan: if in NMT operational, allow writing the same value only (bugzilla 3555: BYang00000241)
         // this is a workaround to schnieder demand to be able to write in NMT operational
         if (u8_tmp == (unsigned char)BGVAR(u8_Fb_Interp_Time_Period))
         {
            ret_val = FalInitDownloadRespond((int)SDO60C2S1, s16_msg_id);
            return ret_val;
         }

         // for cdhd and all non-lxm28 drives, reject value in NMT state OPERATIONAL.
         if (u16_Product != SHNDR && u16_Product != SHNDR_HW)
         {
            return FAL_WRONG_NMT_STATE;
         }

         // IPR 241: CANmotion: Object 16#60C2 Sub1 can be changed only in pre-operational state
         // this is for lxm28: reject value if in a cyclic mode and NMT state OPERATIONAL.
         if ((p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE)        ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
         {
            return FAL_WRONG_NMT_STATE;
         }


/*
         // nitsan: if not in NMT operational, allow writing the same value only (bugzilla 3555: BYang00000241)
         // this is a workaround to schnieder demand to be able to write in NMT operational
         if (u8_tmp != (unsigned char)BGVAR(u8_Fb_Interp_Time_Period))
         {
            return FAL_WRONG_NMT_STATE;
         }
         else
         {
            ret_val = FalInitDownloadRespond((int)SDO60C2S1, s16_msg_id);
            return ret_val;
         }
*/
      }

      ret_val = SalFbInterpTimePeriod((long long)u8_tmp, drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalSetFBSyncOpmode(drive, p402_modes_of_operation_display);
      else
        return NOT_ALLOWED_CYCLE_TIME;

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO60C2S1, s16_msg_id);
      else       
        return NOT_ALLOWED_CYCLE_TIME;
   }
   else// read operation
   {
      p402_interpolation_time_period.timeUnits = (unsigned int)BGVAR(u8_Fb_Interp_Time_Period);
      p_fieldbus_bg_tx_data[0] = (int)p402_interpolation_time_period.timeUnits;
      ret_val = FalInitUploadRespond(SDO60C2S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalFbInterpTimeIdx
// Description:
//          This function is called in response object 0x60C2 index 2 from Fieldbus.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalFbInterpTimeIdx(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int s16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      s16_tmp = p_data[0];

      //move from 16 bits to 8 bits with sign extension
      s16_tmp <<= 8;
      s16_tmp >>= 8;

      if(FalIsInOperationMode()) // This variable is not allowed to be wriiten on OPERATION mode
      {
         // dont allow changing if drive is enabled.
         if (Enabled(drive))
         {
            return DRIVE_ACTIVE;
         }

         // nitsan: if in NMT operational, allow writing the same value only (bugzilla 3555: BYang00000241)
         // this is a workaround to schnieder demand to be able to write in NMT operational
         if ((char)s16_tmp == (char)BGVAR(s8_Fb_Interp_Time_Idx))
         {
            ret_val = FalInitDownloadRespond((int)SDO60C2S2, s16_msg_id);
            return ret_val;
         }

         // for cdhd and all non-lxm28 drives, reject value in NMT state OPERATIONAL.
         if (u16_Product != SHNDR && u16_Product != SHNDR_HW)
         {
            return FAL_WRONG_NMT_STATE;
         }

         // IPR 241: CANmotion: Object 16#60C2 Sub1 can be changed only in pre-operational state
         // this is for lxm28: reject value if in a cyclic mode and NMT state OPERATIONAL.
         if ((p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE)        ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
             (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
         {
            return FAL_WRONG_NMT_STATE;
         }

      /*
         // nitsan: if not in NMT operational, allow writing the same value only (bugzilla 3555: BYang00000241)
         // this is a workaround to schnieder demand to be able to write in NMT operational
         if ((char)s16_tmp != (char)BGVAR(s8_Fb_Interp_Time_Idx))
         {
            return FAL_WRONG_NMT_STATE;
         }
         else
         {
            ret_val = FalInitDownloadRespond((int)SDO60C2S2, s16_msg_id);
            return ret_val;
         }
         */
      }

      ret_val = SalFbInterpTimeIdx((long long)s16_tmp, drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalSetFBSyncOpmode(drive, p402_modes_of_operation_display);
      else
         return CYCLE_TIME_GREATER_THAN_1SEC ;

      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO60C2S2, s16_msg_id);
      else
         return FAL_SUBINDEX_NOT_FOUND;
   }
   else// read operation
   {
      p402_interpolation_time_period.timeIndex = BGVAR(s8_Fb_Interp_Time_Idx);
      p_fieldbus_bg_tx_data[0] = (int)p402_interpolation_time_period.timeIndex;
      ret_val = FalInitUploadRespond(SDO60C2S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDigitalInputsCommandBg
// Description:
//          This function is called in response object 0x60FD from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalDigitalInputsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   unsigned long u32_tmp = 0L;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
       ret_val = FalInitDownloadRespond((int)SDO60FDS0, s16_msg_id);
   }
   else// read operation
   {
      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         // for lxm28, take inputs before limit switches xor manipulation
         u32_tmp = VAR(AX0_u16_P4_07_Read);
      }
      else
      {
         // for cdhd
         u32_tmp = VAR(AX0_u16_Input_State);
      }

      //set manufacturer specific bits in 16 MSbit
      u32_tmp = ((u32_tmp << 16) & 0xFFFF0000);

      //set ccw limit switch bit
      if ((VAR(AX0_u16_CCW_LS) & 0x03) != 0)
          u32_tmp |= 0x0001;

      //set cw limit switch bit
      if ((VAR(AX0_u16_CW_LS) & 0x03) != 0)
          u32_tmp |= 0x0002;

      //set home limit switch bit
     if (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)
       u32_tmp |= 0x0004;

     //set interlock bit (STO)
//     if (u16_Sto_state != 0)
      if (u16_STO_Flt_Indication_Actual == 1)  //  Udi Feb 16, 2014
       u32_tmp |= 0x0008;

      p402_digital_inputs = u32_tmp;

      p_fieldbus_bg_tx_data[0] = (p402_digital_inputs & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_digital_inputs >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60FDS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalDigitalOutputsCommandBg
// Description:
//          This function is called in response object 0x60FE index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************

int FalDigitalOutputsCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int drive = 0;
   int ret_val = FAL_SUCCESS,i=0;
   // AXIS_OFF;
   p_data += 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;


      p402_digital_outputs[1] = s32_tmp;

     /* Disregard standard bit 0 for now */

     //if ((p402_digital_outputs[2] & 0x0001) == 1)
     //{
     //   if((p402_digital_outputs[1] & 0x0001) == 1)
     //       VAR(AX0_u16_BrakeOn) = 0;
     //   else
     //       VAR(AX0_u16_BrakeOn) = 1;
     //}
     //else
     //{
     //    VAR(AX0_u16_BrakeOn) = 1;
     //}



       // go over all supported outputs (bits)
       i=0;
       while (u16_Supported_Dig_Outputs_Mask & (1<<i))
       {
            if ((p402_digital_outputs[2]>>(i+16)) & 0x1)
            {
                 ret_val = OutCommand((long long)(i+1),(long long)((p402_digital_outputs[1]>>(i+16)) & 0x1));

                 // nitsan: dont fail SDO if outmode is not idle.
                 // allow other outputs (with mode idle) to be written
                 if (ret_val==NOT_PROGRAMMABLE)
                 {
                      ret_val = SAL_SUCCESS;
                 }
                 else if (ret_val!=SAL_SUCCESS)
                 {
                      return ret_val;
                 }
            }

            i++;
       }
       ret_val = FalInitDownloadRespond((int)SDO60FES1, s16_msg_id);

   }

   else// read operation
   {
     if ((p402_digital_outputs[2] & 0x0001) == 1)
     {
         if (VAR(AX0_u16_BrakeOn)) //if brake on
            s32_tmp &= ~0x0001;
         else
            s32_tmp |= 0x0001;
     }

      for (i=0;i<s16_Num_Of_Outputs;i++)
      {
         s32_tmp |= ((long)(OxValue((int)(i))))<<(16+i);
      }

      /* ITAI - line s32_tmp &= p402_digital_outputs[2]; - DO not remove or change without consulting me */
      /* It was decided that the mask must be reflected in digital outputs read value*/
      s32_tmp &= p402_digital_outputs[2];

      p402_digital_outputs[1] = s32_tmp;

      //See BZ 4794
      u32_Dig_Out_60FE = s32_tmp;

      p_fieldbus_bg_tx_data[0] = (s32_tmp & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (s32_tmp >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO60FES1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDigitalOutputsCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDigitalOutputsCommandRt, "ramcan");
int  FalDigitalOutputsCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{

   long s32_tmp = 0L;
   // AXIS_OFF;


   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // We can take the data as 32-bit since it is ensured that the array to which
   // the pointer points to is aligned to even addresses. Look at the pragma instruction
   // for the "p_s16_rt_rx_data" array.
   s32_tmp = *(unsigned long *)&p_data[0];

   //update RT variable for later use in BG
   LVAR(AX0_s32_CAN_Dig_Out) = s32_tmp;
   VAR(AX0_s16_CAN_Dig_Out_Changed_Flag) = 1;

   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalDigitalOutputsMaskCommandBg
// Description:
//          This function is called in response object 0x60FE index 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalDigitalOutputsMaskCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address


      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      p402_digital_outputs[2] = s32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO60FES2, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (p402_digital_outputs[2] & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_digital_outputs[2] >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO60FES2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalTargetVelocityCommandBg
// Description:
//          This function is called in response object 0x60FF from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTargetVelocityCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_tmp = 0L;
   long long lparam;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      // this section only set a value into the canopen object.
      // the actual movement is evaluated and executed in 402fsa.c (canopen ds402 state machine)

       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (long)p_data[1] & 0xFFFF;
       s32_tmp <<= 16;
       s32_tmp |= ((unsigned long)p_data[0]) & 0xffff;   // to avoid padding of 0xffff in the higher bytes if valu is negative

       BGVAR(s32_Fb_Target_Velocity_For_Vel)  = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                 BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                 BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

       // nitsan: since halt bit in control word set hold, the jog command will fail if halt is set.
       // but in can open standard the velocity value should be accepted if valid regardless of halt bit state.
       // so check min/max here and try to send new jog command in order to chaneg velocity if motion is active.
       lparam = BGVAR(s32_Fb_Target_Velocity_For_Vel) * VAR(AX0_s16_Vel_Polarity);
       if (lparam > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
       if (lparam < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

       /*
       nitsan: if vel is changed while in motion opmode state machine will handle the new jog command, so this section is commnted out
       // if drive is in hold mode, it means that halt bit is set and no need to perform jog.
       // in this case, just update the new velocity value but dont return "hold_mod" error.
       if (BGVAR(u16_Hold) == 0 && BGVAR(u16_Hold_User) == 0)
       {
         ret_val = JogCommand(lparam, drive);
       }
       */

       if(ret_val == FAL_SUCCESS)
          ret_val = FalInitDownloadRespond((int)SDO60FFS0, s16_msg_id);
   }
   else// read operation
   {
       s32_tmp = p402_target_velocity;

       p_fieldbus_bg_tx_data[0] = (s32_tmp & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (s32_tmp >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO60FFS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVelocityWindowCommandBg
// Description:
//          This function is called in response object 0x606D from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityWindowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   // IPR 791: object 0x4328 is used instaed of 0x606D
   if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
   {
      return FAL_OBJECT_NOT_FOUND;
   }

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       BGVAR(s32_Velocity_Window)  = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

       ret_val = FalInitDownloadRespond((int)SDO606DS0, s16_msg_id);
   }
   else// read operation
   {
       /* ITAI - The below units conversion causes 64 bits over flow. We take p402_velocity_window as it was previously set by FB user */
       //p402_velocity_window = (int)MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Velocity_Window),
       //                           BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
       //                           BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = p402_velocity_window;

       ret_val = FalInitUploadRespond(SDO606DS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVelocityWindowTimeCommandBg
// Description:
//          This function is called in response object 0x606E from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityWindowTimeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       BGVAR(u16_Velocity_Window_Time) = u16_tmp;
       ret_val = FalInitDownloadRespond((int)SDO606ES0, s16_msg_id);
   }
   else// read operation
   {
       p402_velocity_window_time = BGVAR(u16_Velocity_Window_Time);
       p_fieldbus_bg_tx_data[0] = (int)p402_velocity_window_time;
       ret_val = FalInitUploadRespond(SDO606ES0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVelocityThresholdCommandBg
// Description:
//          This function is called in response object 0x606F from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityThresholdCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   // IPR 791: object 0x4328 is used instaed of 0x606D
   if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
   {
      return FAL_OBJECT_NOT_FOUND;
   }

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       BGVAR(s32_Velocity_Threshold)  = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

       ret_val = FalInitDownloadRespond((int)SDO606FS0, s16_msg_id);
   }
   else// read operation
   {
       /* ITAI - The below units conversion causes 64 bits over flow. We take p402_velocity_threshold as it was previously set by FB user */
       //p402_velocity_threshold = (unsigned int)MultS64ByFixS64ToS64((long long)LVAR(AX0_s32_Velocity_Threshold),
       //                           BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
       //                           BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)p402_velocity_threshold;

       ret_val = FalInitUploadRespond(SDO606FS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVelocityThresholdTimeCommandBg
// Description:
//          This function is called in response object 0x6070 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVelocityThresholdTimeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       BGVAR(u16_Velocity_Threshold_Time) = u16_tmp;
       ret_val = FalInitDownloadRespond((int)SDO6070S0, s16_msg_id);
   }
   else// read operation
   {
      p402_velocity_threshold_time = BGVAR(u16_Velocity_Threshold_Time);
       p_fieldbus_bg_tx_data[0] = (int)p402_velocity_threshold_time;
       ret_val = FalInitUploadRespond(SDO6070S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}
//**********************************************************
// Function Name: FalEcRxPdoMapCommandBg
// Description:
//          This function is called in response of RPDO mapping object coming from EtherCAT communication.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalEcRxPdoMapCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_pdo_nr,u16_trans_type,u16_inhibit_time,u16_num_of_objs,u16_obj_index;
   int i,j, ret_val = FAL_SUCCESS;
   s16_msg_id += 0;

   if(u16_operation == 1)// write operation
   {
      //get data from tranmission buffer
      u16_pdo_nr       = *p_data++; //get pdo number
      u16_trans_type   = *p_data++; //get transmission type
      u16_inhibit_time = *p_data++; //get inhibit time



     p_data+=6;       //3-8 are reserved
      u16_num_of_objs  = *p_data++; //get number of mapped objects

      //put data in RPDOs array
      p_rpdo_entries[u16_pdo_nr-1].u16_trans_type     = u16_trans_type;
      p_rpdo_entries[u16_pdo_nr-1].u16_inhibit_time   = u16_inhibit_time;
      p_rpdo_entries[u16_pdo_nr-1].u16_num_of_objects = u16_num_of_objs;

      for(i=0;i<u16_num_of_objs;i++)
      {
         u16_obj_index = *p_data++;   //get object index
         *p_data++;                   //next int is reserved

         //need to find the index in the PDO table
         for(j=0;j<NUM_OF_PDO_MAPABLE_OBJS;j++)
         {
            if(FB_Pdos_Array[j].u16_id == u16_obj_index)
               break;
         }

         if(FB_Pdos_Array[j].u16_id == u16_obj_index)
         {
            p_rpdo_entries[u16_pdo_nr-1].map_array[i].u16_id = j;
         }

         p_rpdo_entries[u16_pdo_nr-1].map_array[i].u16_size = FB_objects_size_array[u16_obj_index];  // get size
         p_rpdo_entries[u16_pdo_nr-1].map_array[i].u32_addr = FB_objects_array[u16_obj_index].u32_addr;  // get data address
      }

   ret_val = FalInitDownloadRespond((int)SDO_PDO_RX_MAP, s16_msg_id);
  }

   return (ret_val);
}


//**********************************************************
// Function Name: FalEcTxPdoMapCommandBg
// Description:
//          This function is called in response of TPDO mapping object coming from EtherCAT communication.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalEcTxPdoMapCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_pdo_nr,u16_trans_type,u16_inhibit_time,u16_num_of_objs,u16_obj_index;
   int i,j, ret_val = FAL_SUCCESS;
   s16_msg_id += 0;

   if(u16_operation == 1)// write operation
   {
      //get data from tranmission buffer
      u16_pdo_nr       = *p_data++; //get pdo number
      u16_trans_type   = *p_data++; //get transmission type
      u16_inhibit_time = *p_data++; //get inhibit time

      p_data+=6;       //3-8 are reserved
      u16_num_of_objs  = *p_data++; //get number of mapped objects


      //put data in TPDOs array
      p_tpdo_entries[u16_pdo_nr-1].u16_trans_type     = u16_trans_type;
      p_tpdo_entries[u16_pdo_nr-1].u16_inhibit_time   = u16_inhibit_time;
      p_tpdo_entries[u16_pdo_nr-1].u16_num_of_objects = u16_num_of_objs;

      for(i=0;i<u16_num_of_objs;i++)
      {
         u16_obj_index = *p_data++;  //get object index
         *p_data++;                   //next int is reserved

            //need to find the index in the PDO table
      for(j=0;j<NUM_OF_PDO_MAPABLE_OBJS;j++)
      {
         if(FB_Pdos_Array[j].u16_id == u16_obj_index)
            break;
      }

      if(FB_Pdos_Array[j].u16_id == u16_obj_index)
      {
         p_tpdo_entries[u16_pdo_nr-1].map_array[i].u16_id = j;
      }


        p_tpdo_entries[u16_pdo_nr-1].map_array[i].u16_size = FB_objects_size_array[u16_obj_index];  // get size
        p_tpdo_entries[u16_pdo_nr-1].map_array[i].u32_addr = FB_objects_array[u16_obj_index].u32_addr;  // get data address

      }

      ret_val = FalInitDownloadRespond((int)SDO_PDO_TX_MAP, s16_msg_id);
  }

   return (ret_val);
}

//**********************************************************
// Function Name: FalEcRxPdoActivateCommandBg
// Description:
//          This function is called in response of RPDO activation object coming from EtherCAT communication.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalEcRxPdoActivateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int i;
   // AXIS_OFF;
   s16_msg_id += 0;

   if(u16_operation == 1)// write operation
   {
      //get data from tranmission buffer
      VAR(u16_EtherCat_Pdo_Rx_Activation) = *((int*)p_data); //get data
     VAR(u16_EtherCat_Pdo_Rx_Is_Synced) = 0; //init tarnsmission type

     //if only one RPDO is defined as sync - ALL RPDOs will be looked upon as synced
     for(i=0;i<RPDO_MAX_NUM;i++)
     {
        if(p_rpdo_entries[i].u16_trans_type) //if the RPDO was mapped as synced
          VAR(u16_EtherCat_Pdo_Rx_Is_Synced) = 1;

     }

      ret_val = FalInitDownloadRespond((int)SDO_PDO_RX_ACT, s16_msg_id);
  }

   return (ret_val);
}

//**********************************************************
// Function Name: FalEcTxPdoActivateCommandBg
// Description:
//          This function is called in response of TPDO activation object coming from EtherCAT communication.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalEcTxPdoActivateCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int i;
   // AXIS_OFF;
   s16_msg_id += 0;

   if(u16_operation == 1)// write operation
   {
      //get data from tranmission buffer
      VAR(u16_EtherCat_Pdo_Tx_Activation) = *((int*)p_data); //get data
     VAR(u16_EtherCat_Pdo_Tx_Is_Synced) = 0; //init tarnsmission type

     //if only one TPDO is defined as sync - ALL TPDOs will be looked upon as synced
     for(i=0;i<TPDO_MAX_NUM;i++)
     {
        if(p_tpdo_entries[i].u16_trans_type) //if the TPDO was mapped as synced
          VAR(u16_EtherCat_Pdo_Tx_Is_Synced) = 1;

     }

      ret_val = FalInitDownloadRespond((int)SDO_PDO_TX_ACT, s16_msg_id);
  }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDCLinkVoltageCommandBg
// Description:
//          This function is called in response object 0x6079 from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalDCLinkVoltageCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
       // convert from V to mV (multiply by 1000).
       p402_DC_link_circuit_voltage = (unsigned long)BGVAR(u16_Vbus_Volts) * 1000;
       p_fieldbus_bg_tx_data[0] = (p402_DC_link_circuit_voltage & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_DC_link_circuit_voltage >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO6079S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);

}

//**********************************************************
// Function Name: FalpSoftwareMinPositionLimitCommandBg
// Description:
//          This function is called in response object 0x607DS1 from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalpSoftwareMinPositionLimitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int s16_temp_time, drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      // mask with 0xFFFF to prevent negative values overwrite upper word with 0xffff
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         // convert from internal units to PUU user units and update the p-param (to have the new value saved in p-param)
         s32_tmp = (long)MultS64ByFixS64ToS64(s64_tmp,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);

         BGVAR(s32_PUU_SW_Neg_Limit) = s32_tmp;
      }

      do {
         s16_temp_time = Cntr_3125;
         LLVAR(AX0_u32_Pos_Min_Lim_Lo) = s64_tmp;
      } while (s16_temp_time != Cntr_3125);

      UpdateHystVal(drive);

      ret_val = FalInitDownloadRespond((int)SDO607DS1, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pos_Min_Lim_Lo);
      } while (s16_temp_time != Cntr_3125);
      p402_software_position_limit[1] =  (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_software_position_limit[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_software_position_limit[1] >> 16);    //set 16 MSBits


      ret_val = FalInitUploadRespond(SDO607DS1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalpSoftwareMaxPositionLimitCommandBg
// Description:
//          This function is called in response object 0x607DS2 from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalpSoftwareMaxPositionLimitCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_tmp = 0LL;
   int s16_temp_time, drive = 0, ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
      // mask with 0xFFFF to prevent negative values overwrite upper word with 0xffff
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      s64_tmp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

      if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
      {
         // convert from internal units to PUU user units and update the p-param (to have the new value saved in p-param)
         s32_tmp = (long)MultS64ByFixS64ToS64(s64_tmp,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);

         BGVAR(s32_PUU_SW_Pos_Limit) = s32_tmp;
      }

      do {
         s16_temp_time = Cntr_3125;
         LLVAR(AX0_u32_Pos_Max_Lim_Lo) = s64_tmp;
      } while (s16_temp_time != Cntr_3125);

      UpdateHystVal(drive);

      ret_val = FalInitDownloadRespond((int)SDO607DS2, s16_msg_id);
   }
   else// read operation
   {
      do {
         s16_temp_time = Cntr_3125;
         s64_tmp = LLVAR(AX0_u32_Pos_Max_Lim_Lo);
      } while (s16_temp_time != Cntr_3125);
      p402_software_position_limit[2] =  (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (p402_software_position_limit[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_software_position_limit[2] >> 16);    //set 16 MSBits


      ret_val = FalInitUploadRespond(SDO607DS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalpSoftwarePosLimHystCommandBg
// Description:
//          This function is called in response object 0x214A from Fieldbus.
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int FalpSoftwarePosLimHystCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   long long s64_temp = 0LL;
   int drive = 0, ret_val = FAL_SUCCESS;
   if(u16_operation == 1)// write operation
   {
      // mask with 0xFFFF to prevent negative values overwrite upper word with 0xffff
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<= 16;
      s32_tmp |= (long)p_data[0] & 0xFFFF;

      if (s32_tmp < 0)
         return (VALUE_TOO_LOW);

      s64_temp = MultS64ByFixS64ToS64((long long)s32_tmp,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

     ret_val = SalPosLimHystCommand(s64_temp,drive);
      if (ret_val!=SAL_SUCCESS)
        return ret_val;

      ret_val = FalInitDownloadRespond((int)SDO214AS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Sw_Pos_lim_Hyst =  (long)MultS64ByFixS64ToS64(BGVAR(s64_Pos_Lim_Hyst),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Sw_Pos_lim_Hyst & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Sw_Pos_lim_Hyst >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO214AS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalSupportedModesCommandBg
// Description:
//          This function is called in response object 0x6502 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions: nitsan: set value on read operation
//**********************************************************
int FalSupportedModesCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0; //For Remark Avoidance

   if(u16_operation != 1)
   {
      p402_supported_drive_modes = 0x3ED;

      //if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      //{
         // for ipr 615, do not support modes: sync torque and sync vel
         // port post build tool is updated accordingly
         //p402_supported_drive_modes &= ~(0x100 | 0x200);
      //}


      p_fieldbus_bg_tx_data[0] = (p402_supported_drive_modes & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p402_supported_drive_modes >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO6502S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalMpitchCommandBg
// Description:
//          This function is called in response object 0x207D from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMpitchCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {

       // if this object is saved in MTP (motor name plate), it cannot be modified.
       // e.g. lxm28 is a bundle of drive/motor/encoder so have fixed resolution
       if (BGVAR(u16_MTP_Mode) == 3)
       {
        return MTP_USED;
      }

       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       u32_tmp <<=16;
       u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       manu_spec_Mpitch_command = u32_tmp;
       u32_tmp *= 1000;

       if (u32_tmp > 100000000)
          return (VALUE_TOO_HIGH);

       if (u32_tmp < 10000)
         return (VALUE_TOO_LOW);

       ret_val = SalMpitchCommand((long long)u32_tmp,0);
       if (SAL_SUCCESS == ret_val) FalInitDownloadRespond((int)SDO207DS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Mpitch_command = (u32_Mpitch / 1000);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Mpitch_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Mpitch_command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO207DS0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalMpitchHighResCommandBg
// Description:
//          This function is called in response object 0x2173 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalMpitchHighResCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       u32_tmp <<=16;
       u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       manu_spec_Motor_Pitch_High_Res = u32_tmp;


       if (u32_tmp > 100000000)
          return (VALUE_TOO_HIGH);

       if (u32_tmp < 10000)
         return (VALUE_TOO_LOW);

       ret_val = SalMpitchCommand((long long)u32_tmp,0);
       if (SAL_SUCCESS == ret_val) FalInitDownloadRespond((int)SDO2173S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Motor_Pitch_High_Res = u32_Mpitch;

       p_fieldbus_bg_tx_data[0] = (manu_spec_Motor_Pitch_High_Res & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Motor_Pitch_High_Res >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2173S0, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalHwFeaturesControlAddress
// Description:
//          This function is called in response object 0x2174 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHwFeaturesControlAddress(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       u32_tmp <<=16;
       u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       manu_spec_Hw_Features_Control[1] = u32_tmp;  //Address

       if(!((u32_tmp >= 0x1000) && (u32_tmp <= 0x1005))) //0x1000 - 0x1005 are speacial features - allow that
      {
          if (u32_tmp > 255)
             return (VALUE_TOO_HIGH);

          if (u32_tmp < 86)
            return (VALUE_TOO_LOW);
      }

       FalInitDownloadRespond((int)SDO2174S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (manu_spec_Hw_Features_Control[1] & 0xFFFF); //set 16 LSBits Address
       p_fieldbus_bg_tx_data[1] = (manu_spec_Hw_Features_Control[1] >> 16);    //set 16 MSBits Address

       ret_val = FalInitUploadRespond(SDO2174S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHwFeaturesControlValue
// Description:
//          This function is called in response object 0x2174 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHwFeaturesControlValue(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_index;
   unsigned int u16_byte;

   p_data++;

   if(u16_operation == 1)// write operation
   {
       // this is read only object
       return CO_E_NO_WRITE_PERM;
   }
   else// read operation
   {
      if(!((manu_spec_Hw_Features_Control[1] >= 0x1000) && (manu_spec_Hw_Features_Control[1] <= 0x1005))) //0x1000 - 0x1005 are speacial features - allow that
      {
         if (manu_spec_Hw_Features_Control[1] > 255LL) return (VALUE_TOO_HIGH); //Address
         if (manu_spec_Hw_Features_Control[1] < 86LL)  return (VALUE_TOO_LOW); //Address
      }

      //speacial features
      if((manu_spec_Hw_Features_Control[1] >= 0x1000) && (manu_spec_Hw_Features_Control[1] <= 0x1005)) //0x1000 - 0x1005 are speacial features
      {
         manu_spec_Hw_Features_Control[2] = (unsigned long)HwFeaturesFeaturesValue((unsigned int)manu_spec_Hw_Features_Control[1]);
      }
      else//regular EEPROM read
     {
         u16_byte = manu_spec_Hw_Features_Control[1] & 0x0001; //Address
         u16_index = manu_spec_Hw_Features_Control[1] >> 1; //Address
         manu_spec_Hw_Features_Control[2] = u16_Control_Board_Eeprom_Buffer[u16_index]; //Value

         if (u16_byte == 1)
         {
            manu_spec_Hw_Features_Control[2] = (manu_spec_Hw_Features_Control[2] >> 8);   //Value
         }

         manu_spec_Hw_Features_Control[2] = manu_spec_Hw_Features_Control[2] & 0x00ff; //Value
      }

      p_fieldbus_bg_tx_data[0] = (manu_spec_Hw_Features_Control[2] & 0xFFFF); //set 16 LSBits Value
      p_fieldbus_bg_tx_data[1] = (manu_spec_Hw_Features_Control[2] >> 16);    //set 16 MSBits Value
      ret_val = FalInitUploadRespond(SDO2174S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);

      return(ret_val);
   }
}

unsigned int HwFeaturesFeaturesValue(unsigned int u16_addr)
{
   unsigned int ret_value = 0;
   switch(u16_addr)
   {
      case 0x1000:  //Private label
        ret_value = BGVAR(u16_Hw_Features_Private_Label);
     break;

     case 0x1001: //Dual Drive
        if (u16_Num_Of_Axes > 1)
           ret_value = 1;
         else
            return 0;
     break;

     case 0x1002: //Voltage Type
        if(BGVAR(s16_Vbus) == 540)
           ret_value = 1;
         else
            ret_value = 0;
     break;

     case 0x1003: //Fieldbus
        ret_value = u16_Board_Type;
     break;

     case 0x1004: //Line Loss Detection
        ret_value = BGVAR(u16_Line_Loss_Type);
     break;

     case 0x1005: //Counts per Rev
        ret_value = s32_Sdo_Counts_Per_Rev;
     break;

     case 0x1006: //read Resident code type from EEPROM address 0x67
        ret_value = GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,RESIDENT_CODE_TYPE_ADDR);
        if((ret_value >= 1) && (ret_value <= 3)) break;

        ret_value = GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,PRODUCT_CODE_TYPE_ADDR);

        // if the value of 0x1006 (EEPROM ADDR 0x67) is not 1,2 or 3 check the resident type from the product ADDR 0x60
        switch(ret_value)
        {
            case 2:// DDHD product code
               ret_value = 2;
            break;

            case 4:// ????? product code
               ret_value = 3;
            break;

            default:// CDHD product code
               ret_value = 1;
            break;
        }
     break;

     case 0x1007: //read Drive code type code from EEPROM address 0x68
        ret_value = GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,DRIVE_CODE_TYPE_ADDR);
        // if the value of 0x1007 (EEPROM ADDR 0x68) is not 1 or 2 return DRIVE TYPE "1" (TMS320C2844)
        if(ret_value == 0xFF) ret_value = 1;
     break;

     case 0x1008: //read FPGA image type code from EEPROM address 0x69
        ret_value = GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,FPGA_CODE_TYPE_ADDR);
        if (((ret_value & 0xFF) == 6) && ((GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,HW_FEATURES) & 0x01) == 0))
        {
            ret_value = 12;
            break;
        }
         
        if((((ret_value & 0xFF) >= 1) && ((ret_value & 0xFF) <= 11)) || ((ret_value & 0xFF) == 13)) break;
        ret_value = GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,PRODUCT_CODE_TYPE_ADDR);
        // if the value of 0x1008 (EEPROM ADDR 0x69) is not 1 - 11 check the FPGA type from the product ADDR 0x60 in conjuction with EEPROM ADDR 0x56

        switch(ret_value)
        {
            case 2:// DDHD FPGA code
               ret_value = 4;

               // For DDHD without CAN (AP model) check if CAN transceiver is present
               if ((GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,HW_FEATURES) & 0x01) == 0) ret_value = 12;
            break;

            case 4:// SE CAN FPGA code
               ret_value = 7;
            break;

            case 5:// CDHD EC Basic FPGA code
               ret_value = 3;
            break;

            case 255:// CDHD product code == 255 read EEPROM ADDR 0x56 to get the FPGA TYPE CODE
                     // "1" - CDHD CAN FPGA code , "2" - CDHD EtherCAT FPGA code
                 ret_value =  GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,CDHD_FPGA_TYPE_CODE_ADDR);

                 // For CDHD without CAN (AP model) check if CAN transceiver is present
                 if ((ret_value == 1) && ((GetEEpromRightByteFromAddress(CONTROL_BOARD_EEPROM,HW_FEATURES) & 0x01) == 0)) ret_value = 12;
            break;

            default:
                 ret_value = 0;
            break;
        }
     break;
     case 0x1009: //Counts per Rev
        ret_value = u8_Is_Retro_Display;
     break;
     


   }
   return ret_value;
}

//**********************************************************
// Function Name: FalHwFeaturesPowerAddress
// Description:
//          This function is called in response object 0x2175 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHwFeaturesPowerAddress(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       u32_tmp <<=16;
       u32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       manu_spec_Hw_Features_Power[1] = u32_tmp; //Address


       if(!((u32_tmp >= 0x1000) && (u32_tmp <= 0x1005))) //0x1000 - 0x1005 are speacial features - allow that
      {
          if (u32_tmp > 255)
             return (VALUE_TOO_HIGH);

          if (u32_tmp < 86)
            return (VALUE_TOO_LOW);
      }

       FalInitDownloadRespond((int)SDO2175S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (manu_spec_Hw_Features_Power[1] & 0xFFFF); //set 16 LSBits Address
       p_fieldbus_bg_tx_data[1] = (manu_spec_Hw_Features_Power[1] >> 16);    //set 16 MSBits Address

       ret_val = FalInitUploadRespond(SDO2175S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalHwFeaturesPowerValue
// Description:
//          This function is called in response object 0x2175 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalHwFeaturesPowerValue(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_index;
   unsigned int u16_byte;

   p_data++;

   if(u16_operation == 1)// write operation
   {
       // this is read only object
       return CO_E_NO_WRITE_PERM;
   }
   else// read operation
   {

      if(!((manu_spec_Hw_Features_Power[1] >= 0x1000) && (manu_spec_Hw_Features_Power[1] <= 0x1005))) //0x1000 - 0x1005 are speacial features - allow that
      {
         if (manu_spec_Hw_Features_Power[1] > 255LL) return (VALUE_TOO_HIGH); //Address
         if (manu_spec_Hw_Features_Power[1] < 86LL)  return (VALUE_TOO_LOW); //Address
      }

      //speacial features
      if((manu_spec_Hw_Features_Power[1] >= 0x1000) && (manu_spec_Hw_Features_Power[1] <= 0x1005)) //0x1000 - 0x1005 are speacial features
      {
         manu_spec_Hw_Features_Power[2] = (unsigned long)HwFeaturesFeaturesValue((unsigned int)manu_spec_Hw_Features_Power[1]);
      }
      else//regular EEPROM read
     {
         u16_byte = manu_spec_Hw_Features_Power[1] & 0x0001; //Address
         u16_index = manu_spec_Hw_Features_Power[1] >> 1; //Address
         manu_spec_Hw_Features_Power[2] = u16_Power1_Board_Eeprom_Buffer[u16_index]; //Value

         if (u16_byte == 1)
         {
            manu_spec_Hw_Features_Power[2] = (manu_spec_Hw_Features_Power[2] >> 8); //Value
         }

         manu_spec_Hw_Features_Power[2] = manu_spec_Hw_Features_Power[2] & 0x00ff; //Value
      }


     p_fieldbus_bg_tx_data[0] = (manu_spec_Hw_Features_Power[2] & 0xFFFF); //set 16 LSBits Value
     p_fieldbus_bg_tx_data[1] = (manu_spec_Hw_Features_Power[2] >> 16);    //set 16 MSBits Value
     ret_val = FalInitUploadRespond(SDO2175S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);

     return(ret_val);
   }
}

//**********************************************************
// Function Name: FalBissCFields1
// Description:
//          This function is called in response object 0x2176 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCFields1(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int p_fields[4];
   unsigned int u16_tmp;
   int drive = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       ret_val = BiSSC_GetDataFields((unsigned int*)p_fields,(unsigned int)drive);

      p_fields[0] = u16_tmp;
      ret_val = BiSSC_SetDataFields(p_fields, drive);

      if(ret_val != FAL_SUCCESS)
         return ret_val;

       FalInitDownloadRespond((int)SDO2176S1, s16_msg_id);
   }
   else// read operation
   {
      BiSSC_GetDataFields((unsigned int*)p_fields, (unsigned int)drive);
      p_fieldbus_bg_tx_data[0] = p_fields[0];
      manu_spec_Biss_C_Fields[1] = p_fields[0];
      ret_val = FalInitUploadRespond(SDO2176S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}




//**********************************************************
// Function Name: FalBissCFields2
// Description:
//          This function is called in response object 0x2176 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCFields2(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int p_fields[4];
   unsigned int u16_tmp;
   int drive = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       ret_val = BiSSC_GetDataFields((unsigned int*)p_fields,(unsigned int)drive);

      p_fields[1] = u16_tmp;
      ret_val = BiSSC_SetDataFields(p_fields, drive);

      if(ret_val != FAL_SUCCESS)
         return ret_val;

       FalInitDownloadRespond((int)SDO2176S2, s16_msg_id);
   }
   else// read operation
   {
      BiSSC_GetDataFields((unsigned int*)p_fields, (unsigned int)drive);
      p_fieldbus_bg_tx_data[0] = p_fields[1];
      manu_spec_Biss_C_Fields[2] = p_fields[1];
      ret_val = FalInitUploadRespond(SDO2176S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalBissCFields3
// Description:
//          This function is called in response object 0x2176 sub 3 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCFields3(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int p_fields[4];
   unsigned int u16_tmp;
   int drive = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       ret_val = BiSSC_GetDataFields((unsigned int*)p_fields,(unsigned int)drive);

      p_fields[2] = u16_tmp;
      ret_val = BiSSC_SetDataFields(p_fields, drive);

      if(ret_val != FAL_SUCCESS)
         return ret_val;

       FalInitDownloadRespond((int)SDO2176S3, s16_msg_id);
   }
   else// read operation
   {
      BiSSC_GetDataFields((unsigned int*)p_fields, (unsigned int)drive);
      p_fieldbus_bg_tx_data[0] = p_fields[2];
      manu_spec_Biss_C_Fields[3] = p_fields[2];
      ret_val = FalInitUploadRespond(SDO2176S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalBissCFields14
// Description:
//          This function is called in response object 0x2176 sub 4 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCFields4(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int p_fields[4];
   unsigned int u16_tmp;
   int drive = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       ret_val = BiSSC_GetDataFields((unsigned int*)p_fields,(unsigned int)drive);

      p_fields[3] = u16_tmp;
      ret_val = BiSSC_SetDataFields(p_fields, drive);

      if(ret_val != FAL_SUCCESS)
         return ret_val;

       FalInitDownloadRespond((int)SDO2176S4, s16_msg_id);
   }
   else// read operation
   {
      BiSSC_GetDataFields((unsigned int*)p_fields, (unsigned int)drive);
      p_fieldbus_bg_tx_data[0] = p_fields[3];
      manu_spec_Biss_C_Fields[4] = p_fields[3];
      ret_val = FalInitUploadRespond(SDO2176S4, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalBissCFields15
// Description:
//          This function is called in response object 0x2176 sub 5 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCFields5(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      if(u16_tmp == 0xE110)//open password
      {
         s16_Password_Flag = 1;
      }

      FalInitDownloadRespond((int)SDO2176S5, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Biss_C_Fields[5]; 
      ret_val = FalInitUploadRespond(SDO2176S5, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalBissCReadAddress
// Description:
//          This function is called in response object 0x2177 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCReadAddress(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       manu_spec_Biss_C_Read[1] = (unsigned int)p_data[0] & 0xFFFF;
       FalInitDownloadRespond((int)SDO2177S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_Biss_C_Read[1];
       ret_val = FalInitUploadRespond(SDO2177S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalBissCReadValue
// Description:
//          This function is called in response object 0x2177 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCReadValue(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int value = 0;

   p_data+=0;

   if(u16_operation == 0)// read operation
   {
      ret_val = BiSSC_ReadRegister(manu_spec_Biss_C_Read[1], &value, drive);
      if (ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:

         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitUploadRequest((int)SDO2177S2,1, INIT_UPLOAD_REQUEST_MSG);

         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         return(FAL_SUCCESS);
      }

      if(ret_val == SAL_SUCCESS)
      {
         manu_spec_Biss_C_Read[2] = value;
         p_fieldbus_bg_tx_data[0] = value;
         ret_val = FalInitUploadRespond(SDO2177S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
      }
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalBissCWriteAddress
// Description:
//          This function is called in response object 0x2178 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCWriteAddress(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       manu_spec_Biss_C_Write[1] = (unsigned int)p_data[0] & 0xFFFF;
       FalInitDownloadRespond((int)SDO2178S1, s16_msg_id);
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = manu_spec_Biss_C_Write[1];
       ret_val = FalInitUploadRespond(SDO2178S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalBissCWriteValue
// Description:
//          This function is called in response object 0x2178 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalBissCWriteValue(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      //we need to take data int by int to prevent bad data due to odd address
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      ret_val = BiSSC_WriteRegister(manu_spec_Biss_C_Write[1], u16_tmp, drive);
      if (ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO2178S2, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);

         if (IS_EC_DRIVE_AND_COMMODE_1)
            BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         return(FAL_SUCCESS);
      }
      if(ret_val == SAL_SUCCESS)
         FalInitDownloadRespond((int)SDO2178S2, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Biss_C_Write[2];
      ret_val = FalInitUploadRespond(SDO2178S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalPfbBackupCommandBg
// Description:
//          This function is called in response object 0x2088 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalPfbBackupCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {

       ret_val = FalInitDownloadRespond((int)SDO2088S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Pfb_Backup_command = (long)MultS64ByFixS64ToS64(BGVAR(s64_Pfb_Backup),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_Pfb_Backup_command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_Pfb_Backup_command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2088S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalInterpolationModeCommandBg
// Description:
//          This function is called in response object 0x60C0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationModeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;

   int ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      if(FalIsInOperationMode()) // This variable is not allowed to be wriiten on OPERATION mode
      {
         ret_val = FAL_WRONG_NMT_STATE;
      }
      else // not in operational mode
      {
         // object defined in canopen as uint8
         if ( ((int)p_data[0] < 0) || ((int)p_data[0] > 4) )   // max value is 3
            return VALUE_OUT_OF_RANGE;

         p402_interpolation_sub_mode_select = (int)p_data[0] ;
         BGVAR(u16_Interpolation_Type) = p402_interpolation_sub_mode_select;

         //save data in a rt variable
         VAR(AX0_u16_FieldBus_Int_Mode) = p402_interpolation_sub_mode_select;
         ret_val = FalInitDownloadRespond((int)SDO60C0S0, s16_msg_id);
      }
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = p402_interpolation_sub_mode_select;
      ret_val = FalInitUploadRespond(SDO60C0S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalInterpolationMaxBuffSizeCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 1 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationMaxBuffSizeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data++;

   if(u16_operation == 1)// write operation
   {
       // this is read only object
       return CO_E_NO_WRITE_PERM;
   }
   else// read operation
   {
       p_fieldbus_bg_tx_data[0] = (p402_interpolation_data_configuration.MaxBuffSize & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (p402_interpolation_data_configuration.MaxBuffSize >> 16);    //set 16 MSBits
       ret_val = FalInitUploadRespond(SDO60C4S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}

//**********************************************************
// Function Name: FalInterpolationActualBufSizeCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 2 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationActualBufSizeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned long u32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
       u32_tmp <<= 16;
       u32_tmp |= ((unsigned long)p_data[0]) & 0xffff;   // to avoid padding of 0xffff in the higher bytes if valu is negative

       if (u32_tmp != 1) return VALUE_OUT_OF_RANGE;
       ret_val = FalInitDownloadRespond((int)SDO60C4S2, s16_msg_id);
   }
   else     // read operation
   {
       u32_tmp = p402_interpolation_data_configuration.ActBuffSize;

       p_fieldbus_bg_tx_data[0] = (u32_tmp & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (u32_tmp >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO60C4S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalInterpolationBuffOrganizationCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 3 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationBuffOrganizationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = p_data[0] & 0xff;

       if (u16_tmp > 1) return VALUE_TOO_HIGH;
       ret_val = FalInitDownloadRespond((int)SDO60C4S3, s16_msg_id);
   }
   else     // read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_interpolation_data_configuration.BuffOrg;
       ret_val = FalInitUploadRespond(SDO60C4S3, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalInterpolationBuffPositionCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 4 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationBuffPositionCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = p_data[0] & 0xffff;

       if (u16_tmp != 0) return VALUE_TOO_HIGH;
       ret_val = FalInitDownloadRespond((int)SDO60C4S4, s16_msg_id);

   }
   else     // read operation
   {
       p_fieldbus_bg_tx_data[0] = p402_interpolation_data_configuration.BuffPos;
       ret_val = FalInitUploadRespond(SDO60C4S4, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalInterpolationSizeDataRecordCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 5 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationSizeDataRecordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = p_data[0] & 0x00ff;

       if (u16_tmp != 4) return VALUE_OUT_OF_RANGE;
       ret_val = FalInitDownloadRespond((int)SDO60C4S5, s16_msg_id);
   }
   else     // read operation
   {
       // write only object
       return CO_E_NO_READ_PERM;
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalInterpolationBufClearCommandBg
// Description:
//          This function is called in response object 0x60C4 sub 6 from Fieldbus.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalInterpolationBufClearCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = p_data[0] & 0x00ff;

       if (u16_tmp > 1) return VALUE_TOO_HIGH;
       ret_val = FalInitDownloadRespond((int)SDO60C4S6, s16_msg_id);
   }
   else     // read operation
   {
       // write only object
       return CO_E_NO_READ_PERM;
   }

   return (ret_val);
}




//**********************************************************
// Function Name: FalAnalogInput1CommandBg
// Description:
//          This function is called in response to object 0x20F2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:FalPfbBackupCommandBg
//**********************************************************
int FalAnalogInput1CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
       manu_spec_anin1 = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_An_In_1_Filtered),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = manu_spec_anin1;
       ret_val = FalInitUploadRespond(SDO20F2S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);

}

//**********************************************************
// Function Name: FalTpSrc1CommandBg
// Description:
//          This function is called in response object 0x60D0 sub1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTpSrc1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s16_tmp = p_data[0];

       if (s16_tmp < 1) return VALUE_TOO_LOW;
       if (s16_tmp > 5) return VALUE_TOO_HIGH;
       BGVAR(s16_Touch_Probe_Source_1) = s16_tmp;
       ret_val = FalInitDownloadRespond((int)SDO60D0S1, s16_msg_id);

   }
   else// read operation
   {
       touch_probe_source[1] = BGVAR(s16_Touch_Probe_Source_1);
       p_fieldbus_bg_tx_data[0] = touch_probe_source[1];
       ret_val = FalInitUploadRespond(SDO60D0S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalTpSrc2CommandBg
// Description:
//          This function is called in response object 0x60D0 sub2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalTpSrc2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_tmp = 0;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s16_tmp = p_data[0];

       if (s16_tmp < 1) return VALUE_TOO_LOW;
       if (s16_tmp > 5) return VALUE_TOO_HIGH;
       BGVAR(s16_Touch_Probe_Source_2) = s16_tmp;
       ret_val = FalInitDownloadRespond((int)SDO60D0S2, s16_msg_id);

   }
   else// read operation
   {
       touch_probe_source[2] = BGVAR(s16_Touch_Probe_Source_2);
       p_fieldbus_bg_tx_data[0] = touch_probe_source[2];
       ret_val = FalInitUploadRespond(SDO60D0S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalZeroMphaseDegCommand
// Description:
//          This function is called in response to object 0x217B from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:FalAnalogInput1CommandBg
//**********************************************************
int FalZeroMphaseDegsCommand(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   p_data += 0;

   if(u16_operation == 0)// read operation
   {
       manu_spec_s16_Zero_Mphase_Degree =  CalcZeroMphaseDegs(drive);

       p_fieldbus_bg_tx_data[0] = manu_spec_s16_Zero_Mphase_Degree;
       ret_val = FalInitUploadRespond(SDO217BS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnalogInput1DeadbandCommandBg
// Description:
//          This function is called in response object 0x20F3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput1DeadbandCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;
   unsigned long u32_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       u32_tmp = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       if (u32_tmp > 26214) return (VALUE_TOO_HIGH);

       VAR(AX0_u16_An_In_1_Deadband) = (unsigned int)u32_tmp;

       ret_val = FalInitDownloadRespond((int)SDO20F3S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin1db = (unsigned int)MultS64ByFixS64ToS64((long long)VAR(AX0_u16_An_In_1_Deadband),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_anin1db;

       ret_val = FalInitUploadRespond(SDO20F3S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbSlowMoveLowerVoltageCommandBg
// Description:
//          This function is called in response object 0x2153 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbSlowMoveLowerVoltageCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_tmp = 0;
   long s32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s16_tmp = (int)(p_data[0] & 0xFFFF);

       s32_tmp = (long)MultS64ByFixS64ToS64((long long)s16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       if (s32_tmp > 26214)  return (VALUE_TOO_HIGH);
      if (s32_tmp < -26214) return (VALUE_TOO_LOW);
       s16_Slow_Movement_Lower_Voltage = (int)s32_tmp;

       ret_val = FalInitDownloadRespond((int)SDO2153S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Sfb_Slow_Movement_Lower_Voltage = (unsigned int)MultS64ByFixS64ToS64((long long)s16_Slow_Movement_Lower_Voltage,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_Sfb_Slow_Movement_Lower_Voltage;

       ret_val = FalInitUploadRespond(SDO2153S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbSlowMoveUpperVoltageCommandBg
// Description:
//          This function is called in response object 0x2154 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbSlowMoveUpperVoltageCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int s16_tmp = 0;
   long s32_tmp = 0L;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s16_tmp = (int)(p_data[0] & 0xFFFF);

       s32_tmp = (long)MultS64ByFixS64ToS64((long long)s16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       if (s32_tmp > 26214)  return (VALUE_TOO_HIGH);
       if (s32_tmp < -26214) return (VALUE_TOO_LOW);
       s16_Slow_Movement_Upper_Voltage = (int)s32_tmp;

       ret_val = FalInitDownloadRespond((int)SDO2154S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_Sfb_Slow_Movement_Upper_Voltage = (unsigned int)MultS64ByFixS64ToS64((long long)s16_Slow_Movement_Upper_Voltage,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_Sfb_Slow_Movement_Upper_Voltage;

       ret_val = FalInitUploadRespond(SDO2154S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalSfbPfbPeCommandBg
// Description:
//          This function is called in response object 0x2168 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbPfbPeCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;

   if(u16_operation == 0)// read operation
   {

      manu_spec_Sfb_Pfb_Pe_Command = (long)MultS64ByFixS64ToS64(s64_Sfb_Pfb_Pe,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Sfb_Pfb_Pe_Command & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Sfb_Pfb_Pe_Command >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO2168S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray1CommandBg
// Description:
//          This function is called in response object 0x2169 sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[1] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[0] / 26214.0));

      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[1] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectSanity1CommandBg
// Description:
//          This function is called in response object 0x2171 sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectSanity1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Secondary_Feedback_Voltage_Correct_1[1] =  BGVAR(u16_Voltage_Correct_Sanity_Check_Err_Code);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[1]  & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2171S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectFeature1CommandBg
// Description:
//          This function is called in response object 0x2171 sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectFeature1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Secondary_Feedback_Voltage_Correct_1[2] =  BGVAR(u16_Voltage_Correct_Enable);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[2]  & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2171S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectSfbNegative1CommandBg
// Description:
//          This function is called in response object 0x2171 sub 3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectSfbNegative1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {


      manu_spec_Secondary_Feedback_Voltage_Correct_1[3] = (long)MultS64ByFixS64ToS64(BGVAR(s64_Voltage_Correct_Position_Negative),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);


      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[3] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[3] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2171S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectSfbPositive1CommandBg
// Description:
//          This function is called in response object 0x2171 sub 4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectSfbPositive1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {


      manu_spec_Secondary_Feedback_Voltage_Correct_1[4] = (long)MultS64ByFixS64ToS64(BGVAR(s64_Voltage_Correct_Position_Positive),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);


      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[4] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Secondary_Feedback_Voltage_Correct_1[4] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2171S4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectSanity2CommandBg
// Description:
//          This function is called in response object 0x2172 sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectSanity2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Secondary_Feedback_Voltage_Correct_2[1] =  BGVAR(u16_Voltage_Correct_2_Sanity_Check_Err_Code);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_2[1]  & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2172S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectFeature2CommandBg
// Description:
//          This function is called in response object 0x2172 sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectFeature2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Secondary_Feedback_Voltage_Correct_2[2] =  BGVAR(u16_Voltage_Correct_2_Enable);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_2[2]  & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2172S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbVoltageCorrectNvSave2CommandBg
// Description:
//          This function is called in response object 0x2172 sub 3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectNvSave2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Secondary_Feedback_Voltage_Correct_2[3] =  BGVAR(u16_Voltage_Correct_NvSave_Status);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Secondary_Feedback_Voltage_Correct_2[3]  & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0;    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2172S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray2CommandBg
// Description:
//          This function is called in response object 0x2169 sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[2] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[1] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[2] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray3CommandBg
// Description:
//          This function is called in response object 0x2169 sub 3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray3CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[3] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[2] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[3] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[3] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray4CommandBg
// Description:
//          This function is called in response object 0x2169 sub 4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray4CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[4] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[3] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[4] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[4] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray5CommandBg
// Description:
//          This function is called in response object 0x2169 sub 5 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray5CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[5] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[4] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[5] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[5] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S5, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray6CommandBg
// Description:
//          This function is called in response object 0x2169 sub 6 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray6CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[6] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[5] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[6] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[6] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S6, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray7CommandBg
// Description:
//          This function is called in response object 0x2169 sub 7 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray7CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[7] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[6] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[7] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[7] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S7, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray8CommandBg
// Description:
//          This function is called in response object 0x2169 sub 8 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray8CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[8] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[7] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[8] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[8] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S8, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray9CommandBg
// Description:
//          This function is called in response object 0x2169 sub 9 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray9CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[9] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[8] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[9] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[9] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169S9, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray10CommandBg
// Description:
//          This function is called in response object 0x2169 sub 10 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray10CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[10] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[9] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[10] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[10] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169SA, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArray11CommandBg
// Description:
//          This function is called in response object 0x2169 sub 11 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArray11CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_Command[11] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_Array)[10] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_Command[11] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_Command[11] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO2169SB, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo1CommandBg
// Description:
//          This function is called in response object 0x216A sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo1CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[1] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[0] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[1] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo2CommandBg
// Description:
//          This function is called in response object 0x216A sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo2CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[2] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[1] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[2] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo3CommandBg
// Description:
//          This function is called in response object 0x216A sub 3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo3CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[3] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[2] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[3] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[3] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo4CommandBg
// Description:
//          This function is called in response object 0x216A sub 4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo4CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[4] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[3] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[4] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[4] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo5CommandBg
// Description:
//          This function is called in response object 0x216A sub 5 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo5CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[5] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[4] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[5] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[5] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS5, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo6CommandBg
// Description:
//          This function is called in response object 0x216A sub 6 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo6CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[6] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[5] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[6] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[6] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS6, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo7CommandBg
// Description:
//          This function is called in response object 0x216A sub 7 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo7CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[7] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[6] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[7] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[7] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS7, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo8CommandBg
// Description:
//          This function is called in response object 0x216A sub 8 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo8CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[8] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[7] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[8] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[8] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS8, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo9CommandBg
// Description:
//          This function is called in response object 0x216A sub 9 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo9CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[9] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[8] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[9] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[9] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216AS9, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo10CommandBg
// Description:
//          This function is called in response object 0x216A sub 10 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo10CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[10] =  (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[9] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[10] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[10] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216ASA, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalVoltageCorrectArrayTwo11CommandBg
// Description:
//          This function is called in response object 0x216A sub 11 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageCorrectArrayTwo11CommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      manu_spec_Voltage_Correct_Array_2_Command[11] = (long)(100000.0 * (float)((float)BGVAR(s16_Voltage_Correct_2_Array)[10] / 26214.0));
      p_fieldbus_bg_tx_data[0] = (manu_spec_Voltage_Correct_Array_2_Command[11] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Voltage_Correct_Array_2_Command[11] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216ASB, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnin2UserHighbitsBg
// Description:
//          This function is called in response object 0x216D sub 1 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnin2UserHighbitsBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;
   long long s64_tmp = 0LL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      s64_tmp = (((long long)BGVAR(s16_Voltage_Correct_2_Result) * (long long)BGVAR(s32_Anin2_To_User_Fix)) >> BGVAR(u16_Anin2_To_User_Shr)) + (long long)BGVAR(s32_Anin2_User_Offset);
      manu_spec_u32_Anin2_User_Command[1] = (unsigned long)(s64_tmp >> 32);
      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_Anin2_User_Command[1] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_Anin2_User_Command[1] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216DS1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnin2UserLowbitsBg
// Description:
//          This function is called in response object 0x216D sub 2 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnin2UserLowbitsBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int drive=0, ret_val = FAL_SUCCESS;
   long long s64_tmp = 0LL;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   p_data += 0;
   if(u16_operation == 0)// read operation
   {
      s64_tmp = (((long long)BGVAR(s16_Voltage_Correct_2_Result) * (long long)BGVAR(s32_Anin2_To_User_Fix)) >> BGVAR(u16_Anin2_To_User_Shr)) + (long long)BGVAR(s32_Anin2_User_Offset);
      manu_spec_u32_Anin2_User_Command[2] = (unsigned long)(s64_tmp & 0xFFFFFFFF);
      p_fieldbus_bg_tx_data[0] = (manu_spec_u32_Anin2_User_Command[2] & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_u32_Anin2_User_Command[2] >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO216DS2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}
//**********************************************************
// Function Name: FalSfbVoltageCorrectResult2CommandBg
// Description:
//          This function is called in response object 0x2156 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageCorrectResult2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;


   p_data += 0;


   if(u16_operation == 0)// read operation
   {
       manu_spec_Sfb_Voltage_Correct_Result_2 = s16_Voltage_Correct_2_Result;

       p_fieldbus_bg_tx_data[0] = manu_spec_Sfb_Voltage_Correct_Result_2;
       ret_val = FalInitUploadRespond(SDO2156S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


int FalIdentityObjectVendorIdCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
      SetCanOpenIdentityObject(0);

      p_fieldbus_bg_tx_data[0] = (p301_identity.vendorId & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p301_identity.vendorId >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO1018S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalIdentityObjectProducetCodeCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
      SetCanOpenIdentityObject(0);

      p_fieldbus_bg_tx_data[0] = (p301_identity.productCode & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p301_identity.productCode >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO1018S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalIdentityObjectRevisionNumberCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
      SetCanOpenIdentityObject(0);

      p_fieldbus_bg_tx_data[0] = (p301_identity.revisionNumber & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p301_identity.revisionNumber >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO1018S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

int FalIdentityObjectSerialNumberCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
      SetCanOpenIdentityObject(0);

      p_fieldbus_bg_tx_data[0] = (p301_identity.serialNumber & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (p301_identity.serialNumber >> 16);    //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO1018S4, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalErrorRegisterCommandBg
// Description:
//          This function is called in response object 0x1001 from Fieldbus.
// Author: Sergei.P
// Algorithm:
// Revisions:
//**********************************************************
int FalErrorRegisterCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 0)// read operation
   {
      p_fieldbus_bg_tx_data[0] = (p301_error_register & 0xFF);
      ret_val = FalInitUploadRespond(SDO1001S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalSfbManualCalibrationProcessActivationCommandBg
// Description:
//          This function is called in response object 0x2157 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbManualCalibrationProcessActivationCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static int numOfCalls = 0;
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

      s64_Execution_Parameter[0] = (long long)u16_tmp;
      ret_val = SalManualCalibrationProcess(drive);

      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {
         return(ret_val);
      }
      else if (ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         if (IS_CAN_DRIVE_AND_COMMODE_1)
            InitDownloadRequest((int)SDO2157S0, INIT_DOWNLOAD_REQUEST_MSG, 1, 0, 0, 1, (int*)&u16_tmp);
       if (IS_EC_DRIVE_AND_COMMODE_1)
          BGVAR(u16_Ec_Sal_Not_Finished_Flag) = 1;

         numOfCalls++;
         return(FAL_SUCCESS);
      }

      numOfCalls = 0;
      manu_spec_Sfb_Manual_Calibration_Process_Activation = 0; // initialize the value
      ret_val = FalInitDownloadRespond((int)SDO2157S0, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_Sfb_Manual_Calibration_Process_Activation;
      ret_val = FalInitUploadRespond((int)SDO2157S0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return(ret_val);
}


//**********************************************************
// Function Name: FalAnalogInput1CurrentScaleCommandBg
// Description:
//          This function is called in response object 0x20F4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput1CurrentScaleCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       s32_tmp = (long)p_data[1] & 0xFFFF;
       s32_tmp <<=16;
       s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

       s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);

       manu_spec_anin1iscale = s32_tmp;

       SalIScaleInCommand((long long)manu_spec_anin1iscale,0);

       ret_val = FalInitDownloadRespond((int)SDO20F4S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin1iscale = (unsigned int)MultS64ByFixS64ToS64((long long)BGVAR(s32_Current_Scale_In),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_anin1iscale & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_anin1iscale >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO20F4S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalAnalogInput1OffsetCommandBg
// Description:
//          This function is called in response object 0x20F6 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput1OffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int u16_tmp = 0;
   long u32_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (int)p_data[0] & 0xFFFF;

       u32_tmp = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       SalAnin1OffsetCommand((long long)u32_tmp,0);

       ret_val = FalInitDownloadRespond((int)SDO20F6S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin1offset = (int)MultS64ByFixS64ToS64((long long)BGVAR(s16_An_In_1_Offset_User_Setting),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_anin1offset;

       ret_val = FalInitUploadRespond(SDO20F6S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnalogInput1VelocityScaleCommandBg
// Description:
//          This function is called in response object 0x20F7 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput1VelocityScaleCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;


       s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);

       manu_spec_anin1vscale = s32_tmp;

       SalVScaleInCommand((long long)manu_spec_anin1vscale,0);

       ret_val = FalInitDownloadRespond((int)SDO20F7S0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin1vscale = (unsigned int)MultS64ByFixS64ToS64((long long)BGVAR(s32_Velocity_Scale_In),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_anin1vscale & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_anin1vscale >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO20F7S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalAnalogInput2CommandBg
// Description:
//          This function is called in response to object 0x20F9 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:FalAnalogInput2CommandBg
//**********************************************************
int FalAnalogInput2CommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;

   p_data += 0;


   if(u16_operation == 0)// read operation
   {
       manu_spec_anin2 = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_An_In_2_Filtered),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = manu_spec_anin2;
       ret_val = FalInitUploadRespond(SDO20F9S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);

}




//**********************************************************
// Function Name: FalAnalogInput2DeadbandCommandBg
// Description:
//          This function is called in response object 0x20F3 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput2DeadbandCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_tmp = 0;
   unsigned long u32_tmp = 0;
   // AXIS_OFF;

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (unsigned int)p_data[0] & 0xFFFF;

       u32_tmp = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       if (u32_tmp > 26214) return (VALUE_TOO_HIGH);

       VAR(AX0_u16_An_In_2_Deadband) = (unsigned int)u32_tmp;

       ret_val = FalInitDownloadRespond((int)SDO20FAS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin2db = (unsigned int)MultS64ByFixS64ToS64((long long)VAR(AX0_u16_An_In_2_Deadband),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_anin2db;

       ret_val = FalInitUploadRespond(SDO20FAS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalAnalogInput2CurrentScaleCommandBg
// Description:
//          This function is called in response object 0x20F4 from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput2CurrentScaleCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   long s32_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
      s32_tmp = (long)p_data[1] & 0xFFFF;
      s32_tmp <<=16;
      s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;


       s32_tmp = (long)MultS64ByFixS64ToS64((long long)s32_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);

       manu_spec_anin2iscale = s32_tmp;

       SalIScaleCommand2((long long)manu_spec_anin2iscale,0);


       ret_val = FalInitDownloadRespond((int)SDO20FBS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin2iscale = (unsigned int)MultS64ByFixS64ToS64((long long)BGVAR(u32_Current_Scale_2),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_anin2iscale & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_anin2iscale >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO20FBS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalAnalogInput2OffsetCommandBg
// Description:
//          This function is called in response object 0x20FD from Fieldbus.
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalAnalogInput2OffsetCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   int u16_tmp = 0;
   long u32_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
       //we need to take data int by int to prevent bad data due to odd address
       u16_tmp = (int)p_data[0] & 0xFFFF;

       u32_tmp = (long)MultS64ByFixS64ToS64((long long)u16_tmp,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr);


       SalAnin2OffsetCommand((long long)u32_tmp,0);

       ret_val = FalInitDownloadRespond((int)SDO20FDS0, s16_msg_id);
   }
   else// read operation
   {
       manu_spec_anin2offset = (int)MultS64ByFixS64ToS64((long long)BGVAR(s16_An_In_2_Offset_User_Setting),
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (int)manu_spec_anin2offset;

       ret_val = FalInitUploadRespond(SDO20FDS0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalClVdCommandBg
// Description:
//          This function is called in response object 0x2013 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalClVdCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;
   u16_operation += 0;

   manu_spec_CL_Vd_command = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_Vd),
                                                       BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).u16_unit_conversion_to_user_shr);

   p_fieldbus_bg_tx_data[0] = manu_spec_CL_Vd_command;


   ret_val = FalInitUploadRespond(SDO2013S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}

//**********************************************************
// Function Name: FalWarningbitsLowCommandBg
// Description:
//          This function is called in response object 0x2011 sub 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalWarningbitsLowCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   manu_spec_Warning_Bits_command[1] = (unsigned long)BGVAR(u64_Sys_Warnings);

   p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Warning_Bits_command[1] & 0xFFFF);
   p_fieldbus_bg_tx_data[1] = (int)((manu_spec_Warning_Bits_command[1] >> 16) & 0xFFFF);

   ret_val = FalInitUploadRespond(SDO2011S1, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}

//**********************************************************
// Function Name: FalWarningbitsHighCommandBg
// Description:
//          This function is called in response object 0x2011 sub 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalWarningbitsHighCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0, ret_val = FAL_SUCCESS;
   p_data += 0;
   u16_operation += 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   manu_spec_Warning_Bits_command[2] = (unsigned long)((BGVAR(u64_Sys_Warnings) >> 32) & 0xFFFFFFFF);

   p_fieldbus_bg_tx_data[0] = (int)(manu_spec_Warning_Bits_command[2] & 0xFFFF);
   p_fieldbus_bg_tx_data[1] = (int)((manu_spec_Warning_Bits_command[2] >> 16) & 0xFFFF);

   ret_val = FalInitUploadRespond(SDO2011S2, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}

//**********************************************************
// Function Name: FalClVqCommandBg
// Description:
//          This function is called in response object 0x2014 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalClVqCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   // AXIS_OFF;
   p_data += 0;
   u16_operation += 0;

   manu_spec_CL_Vq_command = (int)MultS64ByFixS64ToS64((long long)VAR(AX0_s16_Vq),
                                                       BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).u16_unit_conversion_to_user_shr);

   p_fieldbus_bg_tx_data[0] = manu_spec_CL_Vq_command;


   ret_val = FalInitUploadRespond(SDO2014S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   return (ret_val);
}


//**********************************************************
// Function Name: FalDataTunnelObjectCommandBg
// Description:
//          This function is called in response object 0x5FFD sub index 1 from Fieldbus.
//
// Author: Lionel
// Algorithm:
// Revisions:
//**********************************************************

int FalDataTunnelObjectCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int i = 0;
   int ret_val = FAL_SUCCESS;
   p_data += 0;

   if(u16_operation == 1)// write operation
   {
     if (!(u16_General_Domain[i++] & 0x007F))
     {
         while (i < FB_objects_size_array[SDO5FFDS1])
         {
            //  Move to Modbus buffer
            if (i%2)
            MB_ST.s16_Recive_Buff[MB_ST.s16_Recive_Counter++] = u16_General_Domain[i++/2] >> 8;
         else
            MB_ST.s16_Recive_Buff[MB_ST.s16_Recive_Counter++] = u16_General_Domain[i++/2] & 0x00FF;
         }
         MB_ST.u16_Modbus_Over_Can = 1;
         MB_ST.s16_Transmit_Counter  = 0;
         MB_ST.s16_Transmit_Idx      = 0;
         MB_ST.ModbusStatus = MB_PARSE_COMMAND;
      }
     ret_val = FalInitDownloadRespond((int)SDO5FFDS1, s16_msg_id);
   }
   else // read operation
   {
      // Tunnel Header
      // bit 7 - flow control = 0 - no need for modbus
      // bit 6-0 - protocol ID = 0 - Modbus
      u16_General_Domain[0] = 0;
      //if (MB_ST.ModbusStatus == MB_WAIT_COMMAND)
      if ((MB_ST.u16_Modbus_Over_Can == 1) && (MB_ST.ModbusStatus == MB_SEND_ANSWER))

      {
         while ( MB_ST.s16_Transmit_Counter > MB_ST.s16_Transmit_Idx)
         {
            //  Write data from transmit buffer to domain buffer
            if (MB_ST.s16_Transmit_Idx % 2)
               u16_General_Domain[MB_ST.s16_Transmit_Idx / 2 + 1] = MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Idx];
            else
               u16_General_Domain[MB_ST.s16_Transmit_Idx / 2] |= MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Idx] << 8;

            /*
            if ((MB_ST.s16_Transmit_Idx & 1) == 0)
               u16_General_Domain[(MB_ST.s16_Transmit_Idx >> 1) + 1] = MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Idx];
            else
               u16_General_Domain[(MB_ST.s16_Transmit_Idx >> 1)] |= (MB_ST.s16_Transmit_Buff[MB_ST.s16_Transmit_Idx] << 8);
            */


           MB_ST.s16_Transmit_Idx++;
         }
         setDomainSize(0x5FFD, 1, (MB_ST.s16_Transmit_Counter + 1) CO_COMMA_LINE_PARA_DECL);
     }
     else
         setDomainSize(0x5FFD, 1, 1 CO_COMMA_LINE_PARA_DECL);

//  no support for ethercat needed
//      if (IS_EC_DRIVE)
//         p_fieldbus_bg_tx_data[0] = (int)(*(int*)manu_spec_Data_Tunnel_Object.targetAdr[0] & 0xffff);
      ret_val = FalInitUploadRespond((int)SDO5FFDS1, s16_msg_id, MB_ST.s16_Transmit_Counter+1, (int*)p_fieldbus_bg_tx_data);
      setDomainSize(0x5FFD,1, DTO_DOMAIN_SIZE CO_COMMA_LINE_PARA_DECL);
   }
   return (ret_val);
}



//**********************************************************
// Function Name: FalInitDomainObjects
// Description:
//          This function is responsible to assign domain objects pointers to their initial locations.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int FalInitDomainObjects(void)
{
   int i = 0, ret_val = FAL_SUCCESS;

   for(i = 1; i < SDO_LAST_OBJECT; i++)
   {
      if((FB_objects_array[i].u16_attr & FAL_DOMAIN_OBJ_MASK) != 0)
      {
         // TBD !!!
      }
   }

   return ret_val;
}

//**********************************************************
// Function Name: FalCheckObjectValidity
// Description:
//          This function is responsible to check the validity of the requested object according to provided index and subindex
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//    u16_length in words
//***********************************************************
int FalCheckObjectValidity(unsigned int u16_operation, unsigned int u16_length, int* p_data, int drive)
{
   int ret_val = FAL_SUCCESS;
   unsigned int u16_drive_table_idx = 0, u16_obj_attr = 0;

   u16_obj_attr = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_attr;

   if(((u16_obj_attr & FAL_VAR_LEN_OBJ_MASK) != 0) && (u16_operation == 1))
      FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)] = u16_length;

   // prevent validation for the ASCII non-equivalent objects
   if((u16_obj_attr & FAL_ASCII_NON_EQUIV_OBJ_MASK) == 0)
   {
      // get the corresponding drive table index
      ret_val = FalGetDriveTableIndex(BGVAR(u16_fb_curr_obj_id), &u16_drive_table_idx);
      if(ret_val == FAL_SUCCESS)
      {
         // proceed to validation sequence
         if(u16_operation == 1)
         {
            //Validate object write access according to drive table:
            ret_val = FalValidateObjectWriteAccess(u16_length, u16_drive_table_idx, p_data, drive);
         }
         else
         {
            //Validate object read access according to drive table:
            ret_val = FalValidateObjectReadAccess(u16_drive_table_idx, drive);
         }
      }
   }
   return ret_val;
}

//**********************************************************
// Function Name: FalCheckObjectDefinition
// Description:
//          This function is responsible to check an object definition in the drive database according to the provided object index and subIndex.
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//    s16_size in words
//***********************************************************
int FalCheckObjectDefinition(int s16_size, unsigned int u16_operation, int drive)
{
   LIST_ELEMENT_T *pCurObj;
   unsigned long  u32_obj_size   = 0;
   unsigned int   u16_index      = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index;
   unsigned char  u8_sub_index   = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   /* Check the existance of the object index in the object dictionary */
   if((pCurObj = searchObj(u16_index CO_COMMA_LINE_PARA_DECL)) == NULL)
      return FAL_OBJECT_NOT_FOUND;

   /* Check the existance of the object sub-index in the object dictionary */
   if (CO_READ_OD8(pCurObj->numOfElem) <= u8_sub_index)
   {
      return FAL_SUBINDEX_NOT_FOUND;
   }

   // check the validity of the size
   if(u16_operation == 1)
   {
      u32_obj_size = getObjSize(pCurObj, u8_sub_index);
      if((pCurObj->pValDesc[u8_sub_index].attribute & CO_NUM_VAL) != 0)
         u32_obj_size = (u32_obj_size + 1) >> 1;

      if(s16_size > (int)u32_obj_size)
         return FAL_WRONG_SIZE;
   }

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalValidateObjectWriteAccess
// Description:
//          This function is responsible to validate a write access to the object of provided index and subindex
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int FalValidateObjectWriteAccess(unsigned int u16_length, unsigned int u16_drive_table_index, int* p_data, int drive)
{
   int ret_val = FAL_SUCCESS, i = 0;
   int s16_arg_type = 0, s16_drive_units_index = 0, s16_fb_drive_units_index = -1;
   long long s64_backupVal, s64_valueBeforeScale;
   long long s64_execution_parameter_backup = 0LL;

   if(p_data == NULL)
   {
      return FAL_INTERNAL_FW_FAULT;
   }

   //backup s64_Execution_Parameter[0] in order to restore so we will not interfear with the serial communication
   s64_execution_parameter_backup = s64_Execution_Parameter[0];

   // get the argument type
   if(MAX_WRITE_PARAMS(Commands_Table[u16_drive_table_index].u16_min_max_arg) > 1)
      s16_arg_type = M_PARAM_TYPE(Commands_Table[u16_drive_table_index].u32_argument_types, (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index - 1));
   else
      s16_arg_type = M_PARAM_TYPE(Commands_Table[u16_drive_table_index].u32_argument_types, 0);

   // load the execution paramter if the argument is not a string or not a non-argument
   if(((s16_arg_type & 0x0003) != 3) && (s16_arg_type != 0))
   {
      if(UNSIGNED_PARAM(Commands_Table[u16_drive_table_index].u8_flags))
      {
         // nitsan 16/1/14: bug 3266.
         //            fix: added casting to uint because unsinged values which
         //                 are negative (when cast to int inside p_data),
         //                 fill s64_Execution_Parameter[0] with 0xffff...
         //                 and made it also negative instead of unsinged positive
         s64_Execution_Parameter[0] = (unsigned long long)(unsigned int)p_data[(u16_length - 1)];
      }
      else
      {
         s64_Execution_Parameter[0] = (long long)p_data[(u16_length - 1)];
      }

      for(i = (u16_length - 2); i >= 0; i--)
      {
         s64_Execution_Parameter[0] <<= 16;
         s64_Execution_Parameter[0] |= (unsigned long long)p_data[i] & 0xFFFF;
      }
   }


   // save value before scaling
   s64_valueBeforeScale = s64_Execution_Parameter[0];

   // handle the units conversion if needed
   if(Commands_Table[u16_drive_table_index].str_units_var != 0)
   {
      s16_drive_units_index = UnitsConversionIndex(Commands_Table[u16_drive_table_index].str_units_var, drive);
      if(s16_drive_units_index >= 0)
      {
         ret_val = FalConvertUnitIndex(s16_drive_units_index, &s16_fb_drive_units_index);
         if(ret_val != FAL_SUCCESS)
         {
          s64_Execution_Parameter[0] = s64_execution_parameter_backup;
            return ret_val;
         }

         if(s16_fb_drive_units_index != -1)// if actual units conversion is not available for Field Bus
         {
            s64_Execution_Parameter[0] = MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                           BGVAR(Unit_Conversion_Table[s16_fb_drive_units_index]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[s16_fb_drive_units_index]).u16_unit_conversion_to_internal_shr);
         }
      }
      else
      {
       s64_Execution_Parameter[0] = s64_execution_parameter_backup;
         return (FAL_INTERNAL_FW_FAULT);
      }
   }

/*
   // if decimal value and a schnider objects (in the 0x4000 range)
   if ((s16_arg_type == 2) && (FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index & 0xf000) == 0x4000)
   {
       // fix for p params which are float in commands table but integer in eds file
       // (these p params are multiplied by 1000 by the user instaed of being treated as float)
       LIST_ELEMENT_T* pCurObj = searchObj(FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index CO_COMMA_LINE_PARA_DECL);
       unsigned int subIndex = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index;
       if(pCurObj == NULL)
            return CO_E_NOT_EXIST;


       // object is a number (not string or domain).
       if (pCurObj->pValDesc[subIndex].attribute & CO_NUM_VAL)
      {
            if (CO_READ_OD_DESC_U8(pCurObj, subIndex, varType) != CO_TYPEDESC_REAL32)
               s16_arg_type = 1;// integer
       }
   }
*/

//   else if(s16_arg_type == 2)\ //  prevented conversion to float
   if((s16_arg_type == 2) && (s16_fb_drive_units_index == -1))
   {
       s64_Execution_Parameter[0] = (long long)((*(float*)&s64_Execution_Parameter[0]) * 1000.0);
       // proceed to validation sequence
       ret_val = ValidateSerialWrite((int)u16_drive_table_index, drive);
   }
   else
   {
       // nitsan
       s64_backupVal = s64_Execution_Parameter[0];
       s64_Execution_Parameter[0] = s64_valueBeforeScale;

       // proceed to validation sequence
       ret_val = ValidateSerialWrite((int)u16_drive_table_index, drive);
       s64_Execution_Parameter[0] = s64_backupVal; // nitsan
   }

   s64_Execution_Parameter[0] = s64_execution_parameter_backup;
   return ret_val;
}

//**********************************************************
// Function Name: FalValidateObjectReadAccess
// Description:
//          This function is responsible to validate a read access to the object of provided index and subindex
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int FalValidateObjectReadAccess(unsigned int u16_drive_table_index, int drive)
{
   int ret_val = FAL_SUCCESS;

   if(BGVAR(u16_fb_curr_obj_id) > (unsigned int)SDO_LAST_OBJECT - 1)
   {
      return FAL_INTERNAL_FW_FAULT;
   }

   drive += 0;// for remark avoidance

   // proceed to validation sequence
   if (Commands_Table[u16_drive_table_index].u16_validity_checks & 0x0001) // PASSWORD_PROTECTED
   {
      if (!s16_Password_Flag)   ret_val = PASSWORD_PROTECTED;
   }

   return ret_val;
}


//**********************************************************
// Function Name: FalPutObject
// Description:
//          This function is responsible to put an object new value into the object dictionary
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Notes:
//    s16_size in words
//***********************************************************
int FalPutObject(int s16_size, int* p_data, int drive)
{
   int co_num = 1; // for remark avoidance
   LIST_ELEMENT_T *pCurObj;
   unsigned char* p_obj_addr;
   unsigned int u16_index = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u16_index;
   unsigned char u8_subIndex = FB_objects_array[BGVAR(u16_fb_curr_obj_id)].u8_sub_index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   /* For the CAN bus drive the object has been already assigned into the object dictionary,
      therefore no additional actions required */

   if((pCurObj = searchObj(u16_index CO_COMMA_LINE_PARA_DECL)) == NULL)
      return FAL_OBJECT_NOT_FOUND;

   // get the address of the object sub-index
   if(u8_subIndex == 0)
   {
      p_obj_addr = CO_READ_ODP(pCurObj->pObj);
   }
   else
   {
      p_obj_addr = (unsigned char *)getSubIndexAddr(pCurObj, u8_subIndex);
   }

    // allocate security mechanism for object dictionary consistency
   CO_APPL_PART_ALLOC(CO_LINE_PARA);

    // copy data to the object dictionary
   if((pCurObj->pValDesc[u8_subIndex].attribute & CO_NUM_VAL) != 0)
      memcpy_ram((void*)p_obj_addr, (void*)p_data, s16_size);
   else
      CO_UNPACK_MEMCPY((UNSIGNED8*)p_obj_addr, (UNSIGNED8*)p_data, FB_objects_size_array[BGVAR(u16_fb_curr_obj_id)], co_num);

    // release security mechanism for object dictionary consistency
   CO_APPL_PART_RELEASE(CO_LINE_PARA);

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalInitDownloadRespond
// Description:
//          This function is responsible to initiate a download response message to Fiedlbus
//
// Author: Anatoly Odler
// Algorithm:
// Revisions: nitsan: add second SDO channel. added sdoNr: 1 is first (default) SDO channel, 2: second SOD cannel
//***********************************************************
int FalInitDownloadRespond(int s16_id, int s16_msg_id)
{
   if(s16_msg_id == 0xFFFF) // Buzilla 5830 FLT 69 Fieldbus target command lost
      return FAL_SUCCESS;

   if((s16_id < 1) || (s16_id > ((int)SDO_LAST_OBJECT - 1)))
   {
      return FAL_INTERNAL_FW_FAULT; // inform the caller of internal FW fault
   }

   if(IS_CAN_DRIVE)
   {
#ifdef CONFIG_SPLIT_INDICATION
         //finishSdoWrInd(1, CO_OK CO_COMMA_LINE_PARA_DECL);// split indication final response
         finishSdoWrInd(BACKGROUND_CONTEXT, s16_CAN_SDO_Channel_Number, CO_OK CO_COMMA_LINE_PARA_DECL);// split indication final response
#endif /* CONFIG_SPLIT_INDICATION */
   }
   else if (IS_EC_DRIVE || IS_PN_DRIVE)
   {
         InitDownloadRespond(s16_id, s16_msg_id, INIT_DOWNLOAD_RESPOND_MSG);
   }
   return FAL_SUCCESS;
}


//**********************************************************
// Function Name: FalInitUploadRespond
// Description:
//          This function is responsible to initiate an download response message to Fiedlbus
//
// Author: Anatoly Odler
// Algorithm:
// Revisions: nitsan: add second SDO channel. added sdoNr: 1 is first (default) SDO channel, 2: second SOD cannel
//***********************************************************
int FalInitUploadRespond(int s16_id, int s16_msg_id, int s16_size, int* p_data)
{

   if(s16_msg_id == 0xFFFF) // Buzilla 5830 FLT 69 Fieldbus target command lost
      return FAL_SUCCESS;

   if((s16_id < 1) || (s16_id > ((int)SDO_LAST_OBJECT - 1)))
   {
      return FAL_INTERNAL_FW_FAULT;// inform the caller of an internal FW fault
   }

   if(p_data == NULL)
   {
      return FAL_INTERNAL_FW_FAULT;// inform the caller of an internal FW fault
   }

   if(IS_CAN_DRIVE)
   {
#ifdef CONFIG_SPLIT_INDICATION
       // send correct SDO channel

      //finishSdoRdInd(1, CO_OK CO_COMMA_LINE_PARA_DECL);// split indication final response
      finishSdoRdInd(BACKGROUND_CONTEXT, s16_CAN_SDO_Channel_Number, CO_OK CO_COMMA_LINE_PARA_DECL);// split indication final response
#endif /* CONFIG_SPLIT_INDICATION */
   }
   else if (IS_EC_DRIVE || IS_PN_DRIVE)
   {
        InitUploadRespond(s16_id, s16_msg_id, INIT_UPLOAD_RESPOND_MSG, 1, 0, 0, s16_size,(int*)p_data);
   }
   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalAbortRequest
// Description:
//          This function is responsible to send an abort request to Fieldbus
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int FalAbortRequest(int s16_trd_context, int s16_id, int s16_msg_id, unsigned int u16_operation, unsigned long u32_err_code)
{
   if(s16_msg_id == 0xFFFF) // Buzilla 5830 FLT 69 Fieldbus target command lost
      return FAL_SUCCESS;

   if(u32_err_code == (unsigned long)CO_OK || u32_err_code == E_SDO_NO_ERROR)
      return FAL_SUCCESS;

   if((s16_id < 1) || (s16_id > ((int)SDO_LAST_OBJECT - 1)))
   {
      u32_err_code = FalConvertErrorCode(FAL_INTERNAL_FW_FAULT);// inform the caller of an internal FW fault
   }

   if(IS_CAN_DRIVE)
   {
#ifdef CONFIG_SPLIT_INDICATION
      if(u16_operation == 1)// write operation
      {
         //finishSdoWrInd(1, (RET_T)u32_err_code CO_COMMA_LINE_PARA_DECL);// split indication final write response
         finishSdoWrInd(s16_trd_context, s16_CAN_SDO_Channel_Number, (RET_T)u32_err_code CO_COMMA_LINE_PARA_DECL);// split indication final write response

      }
      else// read operation
      {
         //finishSdoRdInd(1, (RET_T)u32_err_code CO_COMMA_LINE_PARA_DECL);// split indication final read response
         finishSdoRdInd(s16_trd_context, s16_CAN_SDO_Channel_Number, (RET_T)u32_err_code CO_COMMA_LINE_PARA_DECL);// split indication final read response
      }
#endif /* CONFIG_SPLIT_INDICATION */

   }
   else if (IS_EC_DRIVE || IS_PN_DRIVE)
   {
      AbortRequest(s16_id, s16_msg_id, ABORT_REQUEST_MSG, u32_err_code);
   }

   return FAL_SUCCESS;
}

//**********************************************************
// Function Name: FalConvertErrorCode
// Description:
//          This function is responsible to convert an internal drive error code to corresponding Field bus error code
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
unsigned long FalConvertErrorCode(int s16_drive_err_code)
{
   int i = 0;
   unsigned long ret_val = (unsigned long)CO_OK;

   for(i = 0; (i < FB_ERROR_CODES_CONV_TABLE_SIZE) && (FB_ErrorCodesConv_Table[i].u16_drive_err_code != (unsigned int)s16_drive_err_code); i++);

   if(i != FB_ERROR_CODES_CONV_TABLE_SIZE)// on success
   {
      if(IS_CAN_DRIVE)
         ret_val =  (unsigned long)FB_ErrorCodesConv_Table[i].u16_canbus_err_code;
      else if (IS_EC_DRIVE)
      {
         // If general error - set the error code to manu_spec_u32_Drive_Error_Code object 0x4FA6
         if (FB_ErrorCodesConv_Table[i].u32_ethercat_err_code == E_SDO_OTHER)
         {
            manu_spec_u32_Drive_Error_Code = (unsigned long)s16_drive_err_code;
         }
         else // if not general error set the manu_spec_u32_Drive_Error_Code to 0
         {
            manu_spec_u32_Drive_Error_Code = 0;
         }
         ret_val = (unsigned long)FB_ErrorCodesConv_Table[i].u32_ethercat_err_code;
      }
   }
   else// on fail
   {
      // search for intrnal_fw_error and and return it as general error
      for(i = 0; (i < FB_ERROR_CODES_CONV_TABLE_SIZE) && (FB_ErrorCodesConv_Table[i].u16_drive_err_code != (unsigned int)FAL_INTERNAL_FW_FAULT); i++);

      if(i != FB_ERROR_CODES_CONV_TABLE_SIZE)// if FAL_INTERNAL_FW_FAULT is found
      {
         if(IS_CAN_DRIVE)
            ret_val =  (unsigned long)FB_ErrorCodesConv_Table[i].u16_canbus_err_code;
         else if (IS_EC_DRIVE)
            ret_val = (unsigned long)FB_ErrorCodesConv_Table[i].u32_ethercat_err_code;
      }
      else  // if also FAL_INTERNAL_FW_FAULT is not found
      {
         // we should not get into this section. return the last error as default
         if(IS_CAN_DRIVE)
            ret_val =  (unsigned long)FB_ErrorCodesConv_Table[FB_ERROR_CODES_CONV_TABLE_SIZE-1].u16_canbus_err_code;
         else if (IS_EC_DRIVE)
            ret_val = (unsigned long)FB_ErrorCodesConv_Table[FB_ERROR_CODES_CONV_TABLE_SIZE-1].u32_ethercat_err_code;
      }
   }

   return ret_val;
}

//**********************************************************
// Function Name: FalConvertUnitIndex
// Description:
//          This function is responsible to convert an internal drive units index to corresponding Field bus unist index
//       from the units conversion table
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalConvertUnitIndex(int s16_drive_unit_index, int* p_fb_unit_index)
{
   int i = 0, ret_val = FAL_SUCCESS;

   for(i = 0; (i < FB_UNIT_INDEX_CONV_TABLE_SIZE) && (FB_UnitIndex_Conversion_Table[i].s16_drive != s16_drive_unit_index); i++);

   if(i != FB_UNIT_INDEX_CONV_TABLE_SIZE)// on success
   {
      *p_fb_unit_index = FB_UnitIndex_Conversion_Table[i].s16_fieldbus;
   }
   else// on fail
   {
      ret_val = FAL_INTERNAL_FW_FAULT;
   }

   return ret_val;
}

//**********************************************************
// Function Name: FalGetDriveTableIndex
// Description:
//          This function is responsible to get the drive table index by the corresponding to the object index
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
int FalGetDriveTableIndex(unsigned int u16_object_id, unsigned int* p_drive_table_index)
{
   int i = 0;
//   unsigned int u16_pParam, u16_p_param_index;


/*       The search sequence handles 4 cases:
      1) FB object index refers to one ASCII command and reversely
      2) FB object sub-indexes refer to different ASCII commands
      3) Different FB objects indexes refer to the same ASCII command
      4) Different FB object sub-indexes refers to the same ASCII command */

   // search for the position in the drive table according to the object id
   for(i = 0; (i < COMMANDS_TABLE_SIZE) && (Commands_Table[i].u16_fildbus1_index != u16_object_id); i++);

   if(i != COMMANDS_TABLE_SIZE) //position found: case 1 or case 2
   {
      *p_drive_table_index = i;
      return FAL_SUCCESS;
   }
   else // position not found: case 3 or case 4 or fail
   {

      // case 3
//nitsan      for(i = 0; (i < FB_MULT_REF_OBJECTS_TABLE_SIZE) && (FB_MultASCIIRefObject_Table[i].u16_object_id_1 != u16_object_id); i++);
//      if(i != FB_MULT_REF_OBJECTS_TABLE_SIZE)
//         return(FalGetDriveTableIndex(FB_MultASCIIRefObject_Table[i].u16_object_id_2, p_drive_table_index));

      // case 4
      // compute the id of the first subindex of the object
      u16_object_id -= FB_objects_array[u16_object_id].u8_sub_index - 1;

      // search for the position in the drive table according to the computed id
      for(i = 0; (i < COMMANDS_TABLE_SIZE) && (Commands_Table[i].u16_fildbus1_index != u16_object_id); i++);

      if(i == COMMANDS_TABLE_SIZE)// on fail - return FW fault
      {
         return FAL_INTERNAL_FW_FAULT;

       }

      *p_drive_table_index = i;
      return FAL_SUCCESS;
   }
}


// convert can open index to p param index for schnieder drive
// convertion removes the highest byte and convert the dec digits to hex digits
// e.g can open index 0x4A12 is converted to P10-18 (1018)
// returns the p param value
unsigned int ConvertCanOpenIndexToPparam(unsigned int canOpenIndex)
{
   unsigned int u16_group, u16_param;

   u16_group = (canOpenIndex >> 8) & 0xF;  // get p param group value.
   u16_param = canOpenIndex & 0xFF;
   u16_param += (u16_group * 100);         // get the full p-param value
   return u16_param;
}


//**********************************************************
// Function Name: InitUploadRespond
// Description:
//          This function writes an object in BG TX buffer.
//
//   entry in control buffer
//   ------------------------
//   ____________________________
//  |    Object ID               |  16 bits
//  |____________________________|
//  |    Offset in data buffer   |  16 bits
//  |____________________________|
//  |    Message properties      |  16 bits - 8 bits: message number ID, 4 bits: message type
//  |____________________________|            1 bit: is last, 1 bit: is size indication, 2 bits: reserved
//  |    Reserved                |  16 bits
//  |____________________________|
//
//
//
//   entry in data buffer
//   ------------------------
//   ____________________________
//  |                            |  32 bits
//  |    Total message size      |
//  |     (for segmented)        |
//  |____________________________|
//  |    Packet size             |  16 bits
//  |____________________________|
//  |    Data - first            |  16 bits
//  |____________________________|
//  |    Data - second           |  16 bits
//  |____________________________|
//  |          ...               |
//  |          ...               |
//  |          ...               |
//  | ___________________________|
//  |    Data - Nth              |  16 bits
//  |____________________________|
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void InitUploadRespond(int s16_id, int s16_msg_id, int s16_msg_type, int s16_is_last, int s16_is_size_ind, unsigned long u32_complete_size, int s16_size, int* p_data)
{
   int i;
   unsigned int u16_msg_prop = 0; //message properties (id,type,is last, is size indication)

   unsigned int u16_local_bg_tx_ctrl_in_index = 0;
   unsigned int u16_local_bg_tx_data_in_index = 0;
   unsigned int u16_tmp_bg_tx_data_in_index = 0; //pointer to begining of data for later use

   p_data += 0; //For Remark Avoidance
   u32_complete_size += 0;//For Remark Avoidance

   SET_DPRAM_FPGA_MUX_REGISTER(2);
   // Read the value in DPR address 0x1702 (TOMI memory space).
   // This is an offset for the "LP TX FIFO CTRL" register.
   u16_local_bg_tx_ctrl_in_index = *(int*)p_u16_bg_tx_ctrl_in_index;
   // Read the value in DPR address 0x1703 (TOMI memory space).
   // This is an offset for the "LP TX FIFO DATA" register.
   u16_local_bg_tx_data_in_index = *(int*)p_u16_bg_tx_data_in_index;

   //data information header and data must not be written on a buffer rollover -make sure we have 7 available places
   if(u16_local_bg_tx_data_in_index > (FIELDBUS_BG_DATA_BUFFER_SIZE - 7))
      u16_local_bg_tx_data_in_index = 0;

   // Save the start-offset (in "LP TX FIFO DATA") of the new message in an temporary variable
   u16_tmp_bg_tx_data_in_index = u16_local_bg_tx_data_in_index;

   SET_DPRAM_FPGA_MUX_REGISTER(3);

   //write 32 bits of complete size - TBD for now
   *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = 0;
   u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = 0;
   u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //write current packet size in 16 bits
   *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = s16_size;
   u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //write data
   for(i=0;i<s16_size;i++)
   {
       *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = p_data[i];

       //increament pointer
       u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
   }

   //update pointer in register 0x1703 (TOMI)
   SET_DPRAM_FPGA_MUX_REGISTER(2);
   *p_u16_bg_tx_data_in_index = u16_local_bg_tx_data_in_index;

   //write index id to "LP TX FIFO CTRL"
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = s16_id;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the address of the data to "LP TX FIFO CTRL"
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = u16_tmp_bg_tx_data_in_index;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   // set message id
   u16_msg_prop = s16_msg_id;

   // set message type
   u16_msg_prop |= (s16_msg_type << 8);

   //set "is last" bit
   if(s16_is_last)
      u16_msg_prop |= 0x1000;

   //set "is size ind" bit
   if(s16_is_size_ind)
      u16_msg_prop |= 0x2000;

   //set the properties word to "LP TX FIFO CTRL"
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = u16_msg_prop;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the reserved word to "LP TX FIFO CTRL"
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = 0;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //update pointer in 0x1702 (TOMI)
   *p_u16_bg_tx_ctrl_in_index = u16_local_bg_tx_ctrl_in_index;

   SET_DPRAM_FPGA_MUX_REGISTER(0);

}

//**********************************************************
// Function Name: InitDownloadRespond
// Description:
//          This function writes an object in BG TX buffer.
//
//   entry in control buffer
//   ------------------------
//   ____________________________
//  |    Object ID               |  16 bits
//  |____________________________|
//  |    Offset in data buffer   |  16 bits
//  |____________________________|
//  |    Message properties      |  16 bits - 8 bits: message number ID, 4 bits: message type
//  |____________________________|            4 bits reserved
//  |    Reserved                |  16 bits
//  |____________________________|
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void InitDownloadRespond(int s16_id, int s16_msg_id, int s16_msg_type)
{
   unsigned int u16_msg_prop = 0; //message properties (id,type,is last, is size indication)

   unsigned int u16_local_bg_tx_ctrl_in_index = 0;

   SET_DPRAM_FPGA_MUX_REGISTER(2);
   u16_local_bg_tx_ctrl_in_index = *(int*)p_u16_bg_tx_ctrl_in_index;

   //write index id
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = s16_id;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //disregard address word
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = 0;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   // set message id
   u16_msg_prop = s16_msg_id;

   // set message type
   u16_msg_prop |= (s16_msg_type << 8);


   //set the properties word
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = u16_msg_prop;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the reserved word
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = 0;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //update pointer
   *p_u16_bg_tx_ctrl_in_index = u16_local_bg_tx_ctrl_in_index;


   SET_DPRAM_FPGA_MUX_REGISTER(0);
}

//**********************************************************
// Function Name: InitDownloadRequest
// Description:
//          This function writes a write object in BG RX buffer.
//
//
//    entry in control buffer
//   ------------------------
//   ____________________________
//  |    Object ID               |  16 bits
//  |____________________________|
//  |    Offset in data buffer   |  16 bits
//  |____________________________|
//  |    Message properties      |  16 bits - 8 bits: message number ID, 4 bits: message type
//  |____________________________|            1 bit: is last, 1 bit: is size indication, 2 bits: reserved
//  |    Reserved                |  16 bits
//  |____________________________|
//
//
//
//   entry in data buffer
//   ------------------------
//   ____________________________
//  |                            |  32 bits
//  |    Total message size      |
//  |     (for segmented)        |
//  |____________________________|
//  |    Packet size             |  16 bits
//  |____________________________|
//  |    Data - first            |  16 bits
//  |____________________________|
//  |    Data - second           |  16 bits
//  |____________________________|
//  |          ...               |
//  |          ...               |
//  |          ...               |
//  | ___________________________|
//  |    Data - Nth              |  16 bits
//  |____________________________|
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void InitDownloadRequest(int s16_id, int s16_msg_type, int s16_is_last, int s16_is_size_ind, unsigned long u32_complete_size, int s16_size, int* p_data )
{
   int i;
   unsigned int u16_msg_prop = 0; //message properties (id,type,is last, is size indication)


   p_data += 0; //For Remark Avoidance
   u32_complete_size += 0;//For Remark Avoidance
   //write index id
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = s16_id;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the address of the data
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = *p_u16_bg_rx_data_in_index;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   // set message id
   u16_msg_prop = u16_bg_msg_id;
   u16_bg_msg_id = ((u16_bg_msg_id +1) & 0xFF);

   // set message type
   u16_msg_prop |= (s16_msg_type << 8);

   //set "is last" bit
   if(s16_is_last)
      u16_msg_prop |= 0x1000;

   //set "is size ind" bit
   if(s16_is_size_ind)
      u16_msg_prop |= 0x2000;

   //set the properties word
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = u16_msg_prop;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the reserved word
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = 0;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //data information header must not be written on a buffer rollover -make sure we have 3 available places
   if(*p_u16_bg_rx_data_in_index > (FIELDBUS_BG_DATA_BUFFER_SIZE - 3))
      *p_u16_bg_rx_data_in_index = 0;

   //write 32 bits of complete size - TBD for now
   *(int*)(p_bg_rx_data_buffer + *p_u16_bg_rx_data_in_index) = 0;
   *p_u16_bg_rx_data_in_index = ((*p_u16_bg_rx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   *(int*)(p_bg_rx_data_buffer + *p_u16_bg_rx_data_in_index) = 0;
   *p_u16_bg_rx_data_in_index = ((*p_u16_bg_rx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //write current packet size in 16 bits
   *(int*)(p_bg_rx_data_buffer + *p_u16_bg_rx_data_in_index) = s16_size;
   *p_u16_bg_rx_data_in_index = ((*p_u16_bg_rx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //write data
   for(i=0;i<s16_size;i++)
   {
       *(int*)(p_bg_rx_data_buffer + *p_u16_bg_rx_data_in_index) = p_data[i];

       //increament pointer
       *p_u16_bg_rx_data_in_index = ((*p_u16_bg_rx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
   }
}


//**********************************************************
// Function Name: InitUploadRequest
// Description:
//          This function writes a READ object in BG RX buffer.
//
//
//    entry in control buffer
//   ------------------------
//   ____________________________
//  |    Object ID               |  16 bits
//  |____________________________|
//  |    Offset in data buffer   |  16 bits
//  |____________________________|
//  |    Message properties      |  16 bits - 8 bits: message number ID, 4 bits: message type
//  |____________________________|            1 bit: is last, 1 bit: is size indication, 2 bits: reserved
//  |    Reserved                |  16 bits
//  |____________________________|
//
//
//
//   entry in data buffer
//   ------------------------
//   ____________________________
//  |    Packet size             |  16 bits
//  |____________________________|
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
void InitUploadRequest(int s16_id, int s16_size, int s16_msg_type)
{
   unsigned int u16_msg_prop = 0; //message properties (id,type,is last, is size indication)

   //write index id
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = s16_id;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the address of the data
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = *p_u16_bg_rx_data_in_index;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   // set message id
   u16_msg_prop = u16_bg_msg_id;
   u16_bg_msg_id = ((u16_bg_msg_id +1) & 0xFF);

   // set message type
   u16_msg_prop |= (s16_msg_type << 8);

   //set the properties word
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = u16_msg_prop;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the reserved word
   *(int*)(p_bg_rx_ctrl_buffer + *p_u16_bg_rx_ctrl_in_index) = 0;
   *p_u16_bg_rx_ctrl_in_index = ((*p_u16_bg_rx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the size
   *(int*)(p_bg_rx_data_buffer + *p_u16_bg_rx_data_in_index) = s16_size;
   *p_u16_bg_rx_data_in_index = ((*p_u16_bg_rx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));
}

//***********************************************************
// Function Name: AbortRequest
// Description:
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//**********************************************************
void AbortRequest(int s16_id, int s16_msg_id, int s16_msg_type, unsigned long u32_err_code)
{
   unsigned int u16_msg_prop = 0; //message properties (id,type,is last, is size indication)

   unsigned int u16_local_bg_tx_ctrl_in_index = 0;
   unsigned int u16_local_bg_tx_data_in_index = 0;
   unsigned int u16_tmp_bg_tx_data_in_index = 0; //pointer to begining of data for later use

   SET_DPRAM_FPGA_MUX_REGISTER(2);
   u16_local_bg_tx_ctrl_in_index = *(int*)p_u16_bg_tx_ctrl_in_index;
   u16_local_bg_tx_data_in_index = *(int*)p_u16_bg_tx_data_in_index;

   //data information header must not be written on a buffer rollover -make sure we have 3 available places
   if(u16_local_bg_tx_data_in_index > (FIELDBUS_BG_DATA_BUFFER_SIZE - 3))
      u16_local_bg_tx_data_in_index = 0;

   u16_tmp_bg_tx_data_in_index = u16_local_bg_tx_data_in_index;

   SET_DPRAM_FPGA_MUX_REGISTER(3);

   //write error code's LSB word
   *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = (int)(u32_err_code & 0xFFFF);
   //increment pointer
   u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //write error code's MSB word
   *(int*)(p_bg_tx_data_buffer + u16_local_bg_tx_data_in_index) = (int)(u32_err_code >> 16);
   //increament pointer
   u16_local_bg_tx_data_in_index = ((u16_local_bg_tx_data_in_index + 1) & (FIELDBUS_BG_DATA_BUFFER_SIZE - 1));

   //update pointer
   SET_DPRAM_FPGA_MUX_REGISTER(2);
   *p_u16_bg_tx_data_in_index = u16_local_bg_tx_data_in_index;

   //write index id
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = s16_id;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the address of the data
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = u16_tmp_bg_tx_data_in_index;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   // set message id
   u16_msg_prop = s16_msg_id;

   // set message type
   u16_msg_prop |= (s16_msg_type << 8);

   //set the properties word
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = u16_msg_prop;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //set the reserved word
   *(int*)(p_bg_tx_ctrl_buffer + u16_local_bg_tx_ctrl_in_index) = 0;
   u16_local_bg_tx_ctrl_in_index = ((u16_local_bg_tx_ctrl_in_index + 1) & (FIELDBUS_BG_CTRL_BUFFER_SIZE - 1));

   //update pointer
   *p_u16_bg_tx_ctrl_in_index = u16_local_bg_tx_ctrl_in_index;

   SET_DPRAM_FPGA_MUX_REGISTER(0);
}


//***********************************************************
// Function Name: FalTargetPositionCommandRt (0x607a)
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTargetPositionCommandRt, "ramcan");
int  FalTargetPositionCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static long long s64_abs_target_pos_prev = 0LL;
   long s32_tmp_pos = 0L, s32_delta;
   long long s64_abs_target_pos = 0LL;
   unsigned long u32_temp_pfb_lo = 0L;
   long          s32_temp_pfb_hi = 0L;
   long long s64_prev_target_abs = 0LL;
   int s16_sign = 0;
   int drive = 0;
   unsigned int u16_temp_time;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // We can take the data as 32-bit since it is ensured that the array to which
   // the pointer points to is aligned to even addresses. Look at the pragma instruction
   // for the "p_s16_rt_rx_data" array.
   s32_tmp_pos = *(unsigned long *)&p_data[0];

   manu_spec_u16_Debug_Position_Command = s32_tmp_pos; // For Debug, can be read via PCMDFBRAW

   //in order to update the value when it will be read via SDO
   p402_target_position = s32_tmp_pos;
   if(((p402_modes_of_operation_display != INTRPOLATED_POSITION_MODE) && (p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_POSITION_MODE) && (p402_modes_of_operation_display != PROFILE_POSITION_MODE)) || (!VAR(AX0_s16_Active_Indication)))

   //since we run the interpolation regardless of opmode and enable/disable state - We need to copy PFB to PCMD
 /*  if(((p402_modes_of_operation_display != INTRPOLATED_POSITION_MODE) &&
       (p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_POSITION_MODE) &&
       (p402_modes_of_operation_display != PROFILE_POSITION_MODE)) || (!VAR(AX0_s16_Active_Indication) ||(((VAR(AX0_s16_PhaseFindRTBits) & PHASE_FIND_PROCESS_ACTIVE_MASK) !=0)) ||
       (((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK) == 0))))*/
   {//make the memory elements equal to PFB

      //get PFB
      // take pfb
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            u32_temp_pfb_lo = LVAR(AX0_u32_PFB_DF_Lo);//LVAR(AX0_u32_Pfb_Internal_After_Mod_Lo);
            s32_temp_pfb_hi = LVAR(AX0_s32_PFB_DF_Hi);//(AX0_s32_Pfb_Internal_After_Mod_Hi);
         } while (u16_temp_time != Cntr_3125);

      //units conversion
      s32_tmp_pos =
      (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_temp_pfb_lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

      s64_Prev_Target_Position_Command = (long long)s32_tmp_pos + ((long long)LVAR(AX0_u32_FB_One_Rev) * (long long)s32_temp_pfb_hi);

      s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_temp_pfb_hi);


      s32_Prev_Target_Position_Command = s32_tmp_pos;
   }

   // convert 32 bit command to 64 bit to avoid 32 bit wrap around
   s32_delta =  s32_tmp_pos - s32_Prev_Target_Position_Command;

   //limits end
   s64_Prev_Target_Position_Command += (long long)s32_delta;
   s32_Prev_Target_Position_Command = s32_tmp_pos;

   LLVAR(AX0_u32_Tmp_Pos_Lo) = s64_Prev_Target_Position_Command;
   // If conversion factor is a integer value use it to calculate num or rev + fraction, this is to allow better accuracy when using CONV_FB_TARGET_POSITION_TO_INTERNAL
   if (BGVAR(u16_Scaling_Is_Integer))
   {
      s64_prev_target_abs = s64_Prev_Target_Position_Command;
      if (s64_prev_target_abs < 0LL) // To use positive math
      {
         s64_prev_target_abs = -s64_prev_target_abs;
         s16_sign = -1;
      }
      s32_tmp_pos = (long)(s64_prev_target_abs / BGVAR(s64_Scaling_Denominator));
      LLVAR(AX0_u32_Tmp_Pos_Lo) = (s64_prev_target_abs % BGVAR(s64_Scaling_Denominator));
      CONV_FB_POSITION_UNITS_TO_INTERNAL();
      LVAR(AX0_s32_Tmp_Pos_Hi) = s32_tmp_pos;

      if (s16_sign)
         LLVAR(AX0_u32_Tmp_Pos_Lo) = -LLVAR(AX0_u32_Tmp_Pos_Lo);

      POSITION_UNITS_TO_INTERNAL_ADD_OFFSET_AND_POLARITY();
   }
   else
   {
      CONV_FB_POSITION_UNITS_TO_INTERNAL();
      POSITION_UNITS_TO_INTERNAL_ADD_OFFSET_AND_POLARITY();
   }

   s64_abs_target_pos = LLVAR(AX0_u32_Tmp_Pos_Lo);

   // nitsan: if schinder drive, check access rights
   if (((p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) && (BGVAR(s16_CAN_Flags) & CANOPEN_SYNC_PROF_POS_ACCESS_RIGHT_MASK)) || //if cyclic syncronous position mode and has access right (for =s=)
       ((p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE) && (BGVAR(s16_CAN_Flags) & CANOPEN_INTERPOLATED_ACCESS_RIGHT_MASK)))            // or interpolated position mode and has access right (for =s=)
   {
      // this is private case for bug IPR 569: if master request to disable drive but drive is still not disabled (disable process is in background), ignore the position (use prev position)
      // =S= master sends controlword=0 and targetPcmd=0 in the same CAN packet and expects the drive to disable before processing targetPcmd.
      if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         if (((p402_statusword & STATUSWORD_STATE) == OPERATION_ENABLED) && (p402_controlword == 0) && (p402_target_position == 0L))
         {
            s64_abs_target_pos = s64_abs_target_pos_prev;
         }

         // IPR 1350: Parameter P1-42 (ON Delay Time of Holding Brake) has no effect.
         // if drive is enabled but motion is not allowed (brake delay time has not elapsed yet),
         // then we should avoid following mode (from master) and keep same position to avoid drift on Z axis.
         if ((p402_statusword & STATUSWORD_STATE) == SWITCHED_ON)
         {
            s64_abs_target_pos = s64_abs_target_pos_prev;
         }
      }

      //for limit checking function
      BGVAR(s64_Temp_Pos_LS) = s64_abs_target_pos;

//   if (s32_Enc_Sim_Res < 0)
//      BGVAR(s64_Temp_Pos_LS) = -BGVAR(s64_Temp_Pos_LS);

      // If an alignment of the previous temp position is required
      if(BGVAR(u8_Align_Prev_Temp_Pos_LS))
      {
         // Align one time the variable needed for limit switch handling
         BGVAR(s64_Previous_Temp_Pos_LS) = BGVAR(s64_Temp_Pos_LS);
         // Clear the indication
         BGVAR(u8_Align_Prev_Temp_Pos_LS) = 0;
      }
      //BGVAR(s64_Temp_Pos_LS) = LLVAR(AX0_u32_FieldBus_Int_Target_Lo);
      BGVAR(s16_Move_Abs_Issued) = 1;

      if(p402_interpolation_sub_mode_select == LINEAR_INTERPOLATION)
          //pass position command for next Sync cycle - Linear interpolation
          FieldBusLinearInterpolationSetTarget(drive, s64_abs_target_pos);

      else if (p402_interpolation_sub_mode_select != LINEAR_INTERPOLATION)
         //pass position command for next Sync cycle - Cubic interpolation
         FieldBusCubicInterpolationSetTarget(drive, s64_abs_target_pos, 0L, POS_VALUE);
   }

   else if (p402_modes_of_operation_display == PROFILE_POSITION_MODE)  //if profile position mode
   {
      BGVAR(s64_Fb_Target_Position) = s64_abs_target_pos;
   }

   s64_abs_target_pos_prev = s64_abs_target_pos;

   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalProfileVelocityPpModeCommandRt
// Description:   Since profile position mode is actually a BG mode we allow using bBG parameters (0x6081)
// Author: Itai
// Algorithm:
// Revisions: nitsan: limit vel according to vmax
//**********************************************************
#pragma CODE_SECTION(FalProfileVelocityPpModeCommandRt, "ramcan");
int  FalProfileVelocityPpModeCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_vel = 0L;
   int drive = 0;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //The below 2 lines were added to save execution time in RT.
   //Might need to remove in future since there will be a need to use them for all operatio modes.
// nitsan: i remove this condition because it may lead to bugs. e.g. when changing the velocity while opmode == vel_mode and then going to opmode = pos_mode and excpecting the value to be already set.
//   if((p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_POSITION_MODE) && (p402_modes_of_operation_display != PROFILE_POSITION_MODE) && (p402_modes_of_operation_display != INTRPOLATED_POSITION_MODE))
//      return (FAL_SUCCESS);

   // We can take the data as 32-bit since it is ensured that the array to which
   // the pointer points to is aligned to even addresses. Look at the pragma instruction
   // for the "p_s16_rt_rx_data" array.
   s32_tmp_vel = *(unsigned long *)&p_data[0];


      //in order to update the value when it will be read via SDO
   p402_profile_velocity = s32_tmp_vel;

   //units conversion
   s32_tmp_vel = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_Out_Loop_To_Int_Fix))) >> VAR(AX0_u16_Velocity_Out_Loop_To_Int_Shr));

   // bug 3662: limit velocity according to vmax (object 607f)
   if (s32_tmp_vel > BGVAR(s32_V_Lim_Design))
   {
      s32_tmp_vel = BGVAR(s32_V_Lim_Design);
   }

   s32_tmp_vel *= VAR(AX0_s16_Vel_Polarity);

   if (((p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) || (p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE)) && (p402_interpolation_sub_mode_select == CUBIC_INTERPOLATION))  //if syclic syncronous position mode + Cubic position and velocity interpolation
   {
      //pass position command for next Sync cycle - Cubic interpolation
     FieldBusCubicInterpolationSetTarget(drive,0LL, s32_tmp_vel,VEL_VALUE);
   }
   else if (p402_modes_of_operation_display == PROFILE_POSITION_MODE)  //if profile position mode
   {
      BGVAR(s32_Fb_Target_Velocity_For_Pos) = s32_tmp_vel;
   }


   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalProfileVelocityCommandRt (0x60ff)
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalProfileVelocityCommandRt, "ramcan");
int  FalProfileVelocityCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static long s32_tmp_vel_prev = 0L;
   long s32_tmp_vel = 0L;
   int drive = 0;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // We can take the data as 32-bit since it is ensured that the array to which
   // the pointer points to is aligned to even addresses. Look at the pragma instruction
   // for the "p_s16_rt_rx_data" array.
   s32_tmp_vel = *(unsigned long *)&p_data[0];
   // in order to save RT time, if drive is not enabld, or in a different opmode, only process modified values (do nothing if same value is sent again).
   // fix bugzilla 3855: PDO Target Torque (object 0x6071)- Updated only after a drive is in Enable.
   if ((!VAR(AX0_s16_Active_Indication)) || ((p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_VELOCITY_MODE) && (p402_modes_of_operation_display != PROFILE_VELOCITY_MODE)))
   {
      if (s32_tmp_vel_prev == (unsigned long)s32_tmp_vel)
      {
         return (FAL_SUCCESS);
      }
   }

   s32_tmp_vel_prev = s32_tmp_vel;

   //in order to update the value when it will be read via SDO
   p402_target_velocity = s32_tmp_vel;

   //save the raw data of the velocity command for later test in BG against "Target Reached"
   BGVAR(s16_CAN_Flags) |= CANOPEN_PROFILE_VELOCITY_CMD_RCV;
   BGVAR(s32_Fb_Target_Velocity_For_Vel_Raw_Data) = s32_tmp_vel;

   //units conversion
   s32_tmp_vel = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_In_Loop_Fix)) + LLVAR(AX0_u32_Vel_In_Loop_Half_For_Round_Lo)) >> VAR(AX0_u16_Velocity_In_Loop_Shr));

   //Saturation only for positive (negative in set by polarity object)
   if(s32_tmp_vel > 22750)
      s32_tmp_vel = 22750;

   s32_tmp_vel *= VAR(AX0_s16_Vel_Polarity);

   // bugzilla 5311: Unexpected movement in velocity operation mode.
   // nitsan: if schinder drive, check access rights (for CDHD drive, this condition is always true)
   if (BGVAR(s16_CAN_Flags) & CANOPEN_SYNC_PROF_POS_ACCESS_RIGHT_MASK)    //if cyclic syncronous position mode and has access right (for =s=)
   {
      if (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_VELOCITY_MODE)  //if syclic syncronous velocity mode
      {
         //pass position command for next Sync cycle
         FieldBusLinearInterpolationSetTarget(drive,(long long)s32_tmp_vel);
      }

      // fix bugzilla 3855
      //else if (p402_modes_of_operation_display == PROFILE_VELOCITY_MODE)  //if profile velocity mode
      else
      {
         if (TestJogCommand((long long)s32_tmp_vel) == FAL_SUCCESS)
         {
            VAR(AX0_s16_Serial_Vel_Cmnd) = (int)s32_tmp_vel;
         }
      }
   }

   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalVelocityOffsetCommandRt 0x60B1
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalVelocityOffsetCommandRt, "ramcan");
int  FalVelocityOffsetCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_vel = 0L;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp_vel =  (long)p_data[1];
   s32_tmp_vel <<= 16;
   s32_tmp_vel |= ((long)p_data[0] & 0xFFFF);

   //data to set is the same as current data
   if(p402_velocity_offset == s32_tmp_vel)
      return FAL_SUCCESS;

   //in order to update the value when it will be read via SDO
   p402_velocity_offset = s32_tmp_vel;


   //units conversion
   s32_tmp_vel = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_In_Loop_Fix)) + LLVAR(AX0_u32_Vel_In_Loop_Half_For_Round_Lo)) >> VAR(AX0_u16_Velocity_In_Loop_Shr));

   //Saturation
   if(s32_tmp_vel > 22750)
      s32_tmp_vel = 22750;

   if(s32_tmp_vel < -22750)
      s32_tmp_vel = -22750;


    VAR(AX0_s16_Extrn_Vcmd_FF) = (int)s32_tmp_vel;


   return (FAL_SUCCESS);
}


//**********************************************************
// Function Name: FalTargetTorqueCommandRt (6071)
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTargetTorqueCommandRt, "ramcan");
int  FalTargetTorqueCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   static long s32_tmp_torque_prev = 0;
   long s32_tmp_torque = 0L;
   int drive = 0;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   if (*(int*)p_data < 0)
      s32_tmp_torque = ((*(int*)p_data) | 0xFFFF0000);
   else
      s32_tmp_torque = ((*(int*)p_data) & 0x0000FFFF);


   // in order to save RT time, if drive is not enabld, or in a different opmode, only process modified values (do nothing if same value is sent again).
   // fix bugzilla 3855: PDO Target Torque (object 0x6071)- Updated only after a drive is in Enable.
   if ((!VAR(AX0_s16_Active_Indication)) || ((p402_modes_of_operation_display != PROFILE_TORQUE_MODE) && (p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_TORQUE_MODE)))
   {
      if (s32_tmp_torque_prev == s32_tmp_torque)
      {
         return (FAL_SUCCESS);
      }
   }

   s32_tmp_torque_prev = s32_tmp_torque;

   // convert to can units to mA
//   s32_tmp_torque = (long)((((long long)(s32_tmp_torque) * (long long)BGVAR(s32_Current_To_User_mA_Fix)) + BGVAR(s64_Current_To_User_mA_Round)) >> BGVAR(u32_Current_To_User_mA_Shift));
// s32_tmp_torque = (long)((((long long)(s32_tmp_torque) * (long long)fixer) + rounder >> shifter);

   //in order to update the value when it will be read via SDO
   p402_target_torque = (int)s32_tmp_torque;


   //check whether we're in a halt mode
   if(p402_controlword & HALT_MOVE)
   {
         s32_tmp_torque = 0L;
   }

   //units conversion
   s32_tmp_torque = (long)((((long long)(s32_tmp_torque) * (long long)LVAR(AX0_s32_Current_To_Internal_Fix)) + LLVAR(AX0_u32_Current_To_Internal_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_Internal_Shr));

   //Saturation
   if(s32_tmp_torque > 26214)
      s32_tmp_torque = 26214;

   if(s32_tmp_torque < -26214)
      s32_tmp_torque = -26214;

   // fix bugzilla 3855
   //if (p402_modes_of_operation_display == PROFILE_TORQUE_MODE)  //if profile torque mode
   {
      BGVAR(s16_Fb_Target_Torque) = (int)s32_tmp_torque;
      // nitsan: let 402fsa.c handle the command for this opmode
      // VAR(AX0_s16_Serial_Crrnt_Cmnd) = (int)s32_tmp_torque;
   }

   if (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE)  //if cyclic syncronous torque mode
   {
      //pass position command for next Sync cycle
      FieldBusLinearInterpolationSetTarget(drive,(long long)s32_tmp_torque);
   }

   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalTorqueOffsetCommandRt 0x60B2
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTorqueOffsetCommandRt, "ramcan");
int  FalTorqueOffsetCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_torque = 0L;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   //take data only 1 int
   if (*(int*)p_data < 0)
      s32_tmp_torque = ((*(int*)p_data) | 0xFFFF0000);
   else
      s32_tmp_torque = ((*(int*)p_data) & 0x0000FFFF);

   //data to set is the same as current data
   if(p402_torque_offset == (int)s32_tmp_torque)
      return FAL_SUCCESS;

   //in order to update the value when it will be read via SDO
   p402_torque_offset = (int)s32_tmp_torque;

   //units conversion
   s32_tmp_torque = (long)((((long long)(s32_tmp_torque) * (long long)LVAR(AX0_s32_Current_To_Internal_Fix)) + LLVAR(AX0_u32_Current_To_Internal_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_Internal_Shr));

   //Saturation
   if(s32_tmp_torque > VAR(AX0_s16_I_Sat_Hi)) s32_tmp_torque = VAR(AX0_s16_I_Sat_Hi);
   if(s32_tmp_torque < VAR(AX0_s16_I_Sat_Lo)) s32_tmp_torque = VAR(AX0_s16_I_Sat_Lo);

   VAR(AX0_s16_Extrn_Icmd_FF) = (int)s32_tmp_torque;

   return (FAL_SUCCESS);
}

//**********************************************************
// Function Name: FalAnOutCmdCommandRt 0x2134
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalAnOutCmdCommandRt, "ramcan");
int  FalAnOutCmdCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   if (BGVAR(u8_Analog_Out_Mode) != USER_COM)
      return FAL_SUCCESS;

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp =  (long)p_data[1];
   s32_tmp <<= 16;
   s32_tmp |= ((long)p_data[0] & 0xFFFF);

   //data to set is the same as current data
   if(s32_tmp == manu_spec_AnOutCommand)
      return FAL_SUCCESS;

   //Saturation
   if (s32_tmp >  (long)BGVAR(u16_Analog_Out_Volt_Lim))
     s32_tmp = (long)BGVAR(u16_Analog_Out_Volt_Lim);

   if (s32_tmp < -(long)BGVAR(u16_Analog_Out_Volt_Lim))
        s32_tmp = (long)((long)BGVAR(u16_Analog_Out_Volt_Lim) * (-1));


   //in order to update the value when it will be read via SDO
   manu_spec_AnOutCommand = s32_tmp;

   BGVAR(s32_Analog_Out_Volt_Cmd) = s32_tmp;

   return (FAL_SUCCESS);
}



//**********************************************************
// Function Name: FalInterpolationDataRecord1CommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalInterpolationDataRecord1CommandRt, "ramcan");
int FalInterpolationDataRecord1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_return = 0;
   long s32_tmp;


   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   p402_interpolation_data_record[1] = s32_tmp;

   if(p402_modes_of_operation_display == INTRPOLATED_POSITION_MODE)
   {
      //We look at interpolated position mode as sync position
      s16_return = FalTargetPositionCommandRt(p_data, s16_msg_id,u16_operation);
   }

   return s16_return;
}


//**********************************************************
// Function Name: FalInterpolationDataRecord2CommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalInterpolationDataRecord2CommandRt, "ramcan");
int  FalInterpolationDataRecord2CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long s32_tmp;

   s16_msg_id += 0;
   drive += 0;
   u16_operation += 0;

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   p402_interpolation_data_record[2] = s32_tmp;
   return 0;
}


//**********************************************************
// Function Name: FalInterpolationDataRecord3CommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalInterpolationDataRecord3CommandRt, "ramcan");
int  FalInterpolationDataRecord3CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long s32_tmp;

   s16_msg_id += 0;
   drive += 0;
   u16_operation += 0;

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   p402_interpolation_data_record[3] = s32_tmp;
   return 0;
}


//**********************************************************
// Function Name: FalInterpolationDataRecord4CommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalInterpolationDataRecord4CommandRt, "ramcan");
int  FalInterpolationDataRecord4CommandRt(int* p_data , int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   long s32_tmp;

   s16_msg_id += 0;
   drive  += 0;
   u16_operation += 0;

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp = (long)p_data[1] & 0xFFFF;
   s32_tmp <<= 16;
   s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

   p402_interpolation_data_record[4] = s32_tmp;
   return 0;
}


//**********************************************************
// Function Name: FalInterpolationDataRecord4CommandRt
// Description:
// Author: Itai
// Algorithm: place holder for mapping display support
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDoNothingRt, "ramcan");
int  FalDoNothingRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int drive = 0;
   p_data += 0;
   s16_msg_id += 0;
   drive += 0;
   u16_operation += 0;
   return 0;
}

//***********************************************************
// Function Name: FalActualMotorPositionCommandRt (0x2189)
// Description:
// Author: Lior
// Algorithm:
//**********************************************************
#pragma CODE_SECTION(FalActualMotorPositionCommandRt, "ramcan");
int  FalActualMotorPositionCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_pos, s32_pos_Hi = 0L;
   unsigned long  u32_pos_Lo = 0L;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark


   //we need to take data int by int to prevent bad data due to odd address
   u32_pos_Lo =  p_data[1];
   u32_pos_Lo <<= 16;
   u32_pos_Lo |= ((unsigned long)p_data[0] & 0xFFFF);


   s32_pos_Hi =  p_data[3];
   s32_pos_Hi <<= 16;
   s32_pos_Hi |= ((long)p_data[2] & 0xFFFF);

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Mfb_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Mfb_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Mfb_Pos_Shr_To_User));

   //s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   manu_spec_Mfb = s32_tmp_pos;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalActualPositionCommandRt (0x6064)
// Description:
// Author: Itai
// Algorithm:
// Revisions: nitsan 3/2/14 - if dual mode, return SFB
//**********************************************************
#pragma CODE_SECTION(FalActualPositionCommandRt, "ramcan");
int  FalActualPositionCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_pos, s32_pos_Hi = 0L;
   unsigned long  u32_pos_Lo = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark


   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   u32_pos_Lo = *(unsigned long *)&p_data[0];
   s32_pos_Hi = *(long *)&p_data[2];

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   p402_position_actual_value = s32_tmp_pos;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalSecondaryFeedbackPositionActualValueCommandRt (0x2140)
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalSecondaryFeedbackPositionActualValueCommandRt, "ramcan");
int  FalSecondaryFeedbackPositionActualValueCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_pos, s32_pos_Hi = 0L;
   unsigned long  u32_pos_Lo = 0L;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark


   //we need to take data int by int to prevent bad data due to odd address
   u32_pos_Lo =  p_data[1];
   u32_pos_Lo <<= 16;
   u32_pos_Lo |= ((unsigned long)p_data[0] & 0xFFFF);


   s32_pos_Hi =  p_data[3];
   s32_pos_Hi <<= 16;
   s32_pos_Hi |= ((long)p_data[2] & 0xFFFF);

   //units conversion
   s32_tmp_pos =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)u32_pos_Lo * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo)) >> VAR(AX0_u16_Pos_Shr_To_User));

   s32_tmp_pos += (LVAR(AX0_u32_FB_One_Rev) * s32_pos_Hi);

   manu_spec_Sfb_Pfb = s32_tmp_pos;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalTouchProbesPolarityCommandRt
// Description:   TPDO handler for object 0x4523 (p5-35)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbesPolarityCommandRt, "ramcan");
int FalTouchProbesPolarityCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   manu_spec_u8_P5_35_Probes_Polarity = p_data[0] & 0x00ff;
   return FAL_SUCCESS;
}

//***********************************************************
// Function Name: FalTouchProbeStatusCommandRt (0x60B9)
// Description:
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeStatusCommandRt, "ramcan");
int  FalTouchProbeStatusCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   p402_touch_probe_status = p_data[0];
   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalTouchProbeCounter1CommandRt
// Description:   TPDO handler for object 0x4526
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeCounter1CommandRt, "ramcan");
int FalTouchProbeCounter1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   manu_spec_u16_P5_38_Probe_Cntr = p_data[0];
   return FAL_SUCCESS;
}

//***********************************************************
// Function Name: FalTouchProbeCounter2CommandRt
// Description:   TPDO handler for object 0x4526
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTouchProbeCounter2CommandRt, "ramcan");
int FalTouchProbeCounter2CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   manu_spec_u16_P5_58_Probe2_Cntr = p_data[0];
   return FAL_SUCCESS;
}


//***********************************************************
// Function Name: FalStoStatusCommandRt
// Description:   TPDO handler for object 0x4419 (STO status)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalStoStatusCommandRt, "ramcan");
int FalStoStatusCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   // value is the opposite of u16_STO_Flt_Indication_Actual
   manu_spec_u16_P4_25_STO_Stat = (p_data[0] == 0) ? 1 : 0;
   return FAL_SUCCESS;
}


//***********************************************************
// Function Name: FalDriveStatusCommandRt
// Description:   TPDO handler for object 0x4B29 (Drive Status)
//                Note, this is the same p-param as P0-46
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDriveStatusCommandRt, "ramcan");
int FalDriveStatusCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp_vel;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp_vel =  (long)p_data[1];
   s32_tmp_vel <<= 16;
   s32_tmp_vel |= ((unsigned long)p_data[0] & 0xFFFF);
   manu_spec_s32_P11_41_DrvStat = s32_tmp_vel;
   return FAL_SUCCESS;
}


//***********************************************************
// Function Name: FalSpeedInputAnalogVoltageCommandRt
// Description:   TPDO handler for object 0x4B08 (Speed input Analog-Voltage)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalSpeedInputAnalogVoltageCommandRt, "ramcan");
int FalSpeedInputAnalogVoltageCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp =  (long)p_data[1];
   s32_tmp <<= 16;
   s32_tmp |= ((unsigned long)p_data[0] & 0xFFFF);

   s32_tmp = (int)((long long)((long long)(s32_tmp) * (long long)BGVAR(u32_Fb_Analog_Input_Fix)) >> BGVAR(u16_Fb_Analog_Input_Shr));

   // Multiply the value with 0.1, since this is the way the voltage is represented by the Lexium (x.yy format)
   s32_tmp = (long)(((long long)s32_tmp * 26215) >> 18);

   manu_spec_s32_P11_08_anin1 = s32_tmp;
   return FAL_SUCCESS;
}

//***********************************************************
// Function Name: FalTorqueInputAnalogVoltageCommandRt
// Description:   TPDO handler for object 0x4B0A (Torque input Analog-Voltage)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTorqueInputAnalogVoltageCommandRt, "ramcan");
int FalTorqueInputAnalogVoltageCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp =  (long)p_data[1];
   s32_tmp <<= 16;
   s32_tmp |= ((unsigned long)p_data[0] & 0xFFFF);

   s32_tmp = (int)((long long)((long long)(s32_tmp) * (long long)BGVAR(u32_Fb_Analog_Input_Fix)) >> BGVAR(u16_Fb_Analog_Input_Shr));

   // Multiply the value with 0.1, since this is the way the voltage is represented by the Lexium (x.yy format)
   s32_tmp = (long)(((long long)s32_tmp * 26215) >> 18);

   manu_spec_s32_P11_10_anin2 = s32_tmp;
   return FAL_SUCCESS;
}


//***********************************************************
// Function Name: FalInputFreqCommandRt
// Description:   TPDO handler for object 0x4B06 (Input Frequency)
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int s16_cnt=0;
long  s32_val=0;
#pragma CODE_SECTION(FalInputFreqCommandRt, "ramcan");
int FalInputFreqCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp;

   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we need to take data int by int to prevent bad data due to odd address
   s32_tmp =  (long)p_data[1];
   s32_tmp <<= 16;
   s32_tmp |= ((unsigned long)p_data[0] & 0xFFFF);

   // Reduce the value in case that the gearing input interpolation has been activated (see GEARINMODE).
   // Since the input pulse frequency is in reality lower by the factor in the formula below.
   s32_tmp = s32_tmp >> (VAR(AX0_u16_Qep_Out_Scale) - VAR(AX0_u16_Qep_In_Scale_Design));
   manu_spec_s32_P11_06_Input_Freq = s32_tmp;
   if (manu_spec_s32_P11_06_Input_Freq != 0)
   {
      s16_cnt++;
      s32_val=manu_spec_s32_P11_06_Input_Freq;
   }
   return FAL_SUCCESS;
}


//***********************************************************
// Function Name: FalActualVelocityCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalActualVelocityCommandRt, "ramcan");
int FalActualVelocityCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   long s32_tmp_vel = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   s32_tmp_vel = *(long *)&p_data[0];

   //units conversion
   p402_velocity_actual_value = (long)((((long long)(s32_tmp_vel) * (long long)LVAR(AX0_s32_Velocity_Out_Loop_To_User_Fix)) + LLVAR(AX0_u32_Vel_Out_Loop_Half_For_Round_To_User_Lo)) >> VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr));

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalActualCurrentCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions: Nitsan: assign this function to object 0x6078 instaed of 0x6077 (renamed function)
//**********************************************************
#pragma CODE_SECTION(FalActualCurrentCommandRt, "ramcan");
int  FalActualCurrentCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   int s16_tmp_torque = 0;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_torque = *(int*)p_data;

   //units conversion
   p402_current_actual_value = (int)(((long long)((long long)(s16_tmp_torque) * (long long)LVAR(AX0_s32_Current_To_User_Fix)) + LLVAR(AX0_u32_Current_To_User_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_User_Shr));

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalActualDCurrentCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions: 
//**********************************************************
#pragma CODE_SECTION(FalActualDCurrentCommandRt, "ramcan_2");
int  FalActualDCurrentCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_tmp_torque;
   // AXIS_OFF;


   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_torque = *(int*)p_data;

   //units conversion
   manu_spec_ID_command = (int)(((long long)((long long)(s16_tmp_torque) * (long long)LVAR(AX0_s32_Current_To_User_Fix)) + LLVAR(AX0_u32_Current_To_User_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_User_Shr));

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalPcom1ContrlWordCommandRt
// Description:   RPDO handler for object 0x2191 (PCOM control)
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalPcom1ContrlWordCommandRt, "ramcan");
int FalPcom1ContrlWordCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   manu_spec_Pcom1_Control_Word = (*(unsigned int*)p_data);
   BGVAR(u16_PCom_Cntrl1) = manu_spec_Pcom1_Control_Word;
   BGVAR(u16_Execute_Pcom1_From_Pdo) = 1;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalPcom1Contr2WordCommandRt
// Description:   RPDO handler for object 0x2192 (PCOM control)
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalPcom2ContrlWordCommandRt, "ramcan");
int FalPcom2ContrlWordCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{ 
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   manu_spec_Pcom2_Control_Word = (*(unsigned int*)p_data);
   BGVAR(u16_PCom_Cntrl2) = manu_spec_Pcom2_Control_Word;
   BGVAR(u16_Execute_Pcom2_From_Pdo) = 1;

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalPcom1StatusWordCommandRt
// Description:   RPDO handler for object 0x2193 (PCOM status)
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalPcom1StatusWordCommandRt, "ramcan");
int FalPcom1StatusWordCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   p_data += 0;        //to prevent remark

   //take data only 1 int
   manu_spec_Pcom1_Status_Word = u16_PCom_Stat1;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalPcom2StatusWordCommandRt
// Description:   RPDO handler for object 0x2194 (PCOM status)
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalPcom2StatusWordCommandRt, "ramcan");
int FalPcom2StatusWordCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   p_data += 0;        //to prevent remark

   //take data only 1 int
   manu_spec_Pcom2_Status_Word = u16_PCom_Stat2;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalActualTorqueCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalActualTorqueCommandRt, "ramcan");
int FalActualTorqueCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   // AXIS_OFF;
   int s16_tmp_torque = 0;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_torque = *(int*)p_data;

   //units conversion
   p402_torque_actual_value = (int)(((long long)((long long)(s16_tmp_torque) * (long long)LVAR(s32_Torque_To_User_Fix)) + u64_Torque_To_User_Half_For_Round) >> u16_Torque_To_User_Shr);

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalTorqueDemandValueCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalTorqueDemandValueCommandRt, "ramcan");
int  FalTorqueDemandValueCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_tmp_torque = 0;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_torque = *(int*)p_data;

   //units conversion
   p402_torque_demand_value = (int)(((long long)((long long)(s16_tmp_torque) * (long long)LVAR(AX0_s32_Current_To_User_Fix)) + LLVAR(AX0_u32_Current_To_User_Half_For_Round_Lo)) >> VAR(AX0_u16_Current_To_User_Shr));

   return (FAL_SUCCESS);
}


//***********************************************************
// Function Name: FalAnalogInput1CommandRt 0x20F2
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalAnalogInput1CommandRt, "ramcan");
int FalAnalogInput1CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_tmp_value = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_value = *(int*)p_data;

   //units conversion
   manu_spec_anin1 = (int)((long long)((long long)(s16_tmp_value) * (long long)BGVAR(u32_Fb_Analog_Input_Fix)) >> BGVAR(u16_Fb_Analog_Input_Shr));

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalAnalogInput2CommandRt 0x20F9
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalAnalogInput2CommandRt, "ramcan");
int FalAnalogInput2CommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int s16_tmp_value = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //take data only 1 int
   s16_tmp_value = *(int*)p_data;

   //units conversion
   manu_spec_anin2 = (int)((long long)((long long)(s16_tmp_value) * (long long)BGVAR(u32_Fb_Analog_Input_Fix)) >> BGVAR(u16_Fb_Analog_Input_Shr));

   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalDigitalInputsCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDigitalInputsCommandRt, "ramcan");
int  FalDigitalInputsCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned long  u32_tmp = 0L;
   // AXIS_OFF;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark
   *p_data++;        //to prevent remark

   if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
   {
      // for lxm28, take inputs before limit switches xor manipulation
      u32_tmp = VAR(AX0_u16_P4_07_Read);
   }
   else
   {
      // for cdhd
      u32_tmp = VAR(AX0_u16_Input_State);
   }


   //set manufacturer specific bits in 16 MSbit
   u32_tmp = ((u32_tmp << 16) & 0xFFFF0000);

   //set ccw limit switch bit
   if ((VAR(AX0_u16_CCW_LS) & 0x03) != 0)
     u32_tmp |= 0x0001;

   //set cw limit switch bit
   if ((VAR(AX0_u16_CW_LS) & 0x03) != 0)
     u32_tmp |= 0x0002;

   //set home limit switch bit
   if (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)
       u32_tmp |= 0x0004;

   //set interlock bit (STO)
//   if (u16_Sto_state != 0)
   if (u16_STO_Flt_Indication_Actual == 1)  //  Udi Feb 16, 2014
       u32_tmp |= 0x0008;

   p402_digital_inputs = u32_tmp;
   return (FAL_SUCCESS);
}

//***********************************************************
// Function Name: FalMachineHwPextCommandRt
// Description:
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalMachineHwPextCommandRt, "ramcan");
int  FalMachineHwPextCommandRt(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;

   s16_msg_id += 0;    //to prevent remark
   u16_operation += 0; //to prevent remark

   //we can take the data as 32-bit since the S16 pointer points to a S64 variable
   s32_tmp = *(long *)&p_data[0];

   manu_spec_Machine_HW_Position_External_command = s32_tmp;
   return (FAL_SUCCESS);
}
//**********************************************************
// Function Name: WriteRtObjectInRxBuffer
// Description:
//          This function writes an object in RT RX buffer.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(WriteRtObjectInRxBuffer, "ramcan");
void WriteRtObjectInRxBuffer(int s16_id, int s16_size, int* p_data)
{
   int i;
   unsigned int tmp_in_index = 0;

   tmp_in_index = u16_rt_rx_data_in_index;
   //write size in 16 bits
   *(int*)(rt_rx_data_buffer + u16_rt_rx_data_in_index) = s16_size;
   u16_rt_rx_data_in_index = ((u16_rt_rx_data_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));

   //write data
   for(i=0;i<s16_size;i++)
   {
      *(int*)(rt_rx_data_buffer + u16_rt_rx_data_in_index) = p_data[i];
      u16_rt_rx_data_in_index = ((u16_rt_rx_data_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));
   }

   //write index id in data buffer
   *(int*)(rt_rx_ctrl_buffer + u16_rt_rx_ctrl_in_index) = s16_id;

   //set the address of the data
   *(int*)(rt_rx_ctrl_buffer + u16_rt_rx_ctrl_in_index + 1) = tmp_in_index;

   //increament pointer by 2
   u16_rt_rx_ctrl_in_index = ((u16_rt_rx_ctrl_in_index + 2) & (FIELDBUS_RT_BUFFER_SIZE - 1));
}

//**********************************************************
// Function Name: WriteRtObjectInTxBuffer
// Description:
//          This function writes an object in RT TX buffer.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(WriteRtObjectInTxBuffer, "ramcan");
void WriteRtObjectInTxBuffer(int s16_id, int s16_size, int* p_data)
{
   int i;

   //write index id
   *(int*)(rt_tx_ctrl_buffer + u16_rt_tx_ctrl_in_index) = s16_id;
   u16_rt_tx_ctrl_in_index = ((u16_rt_tx_ctrl_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));

   //set the address of the data
   *(int*)(rt_tx_ctrl_buffer + u16_rt_tx_ctrl_in_index) = u16_rt_tx_data_in_index;
   u16_rt_tx_ctrl_in_index = ((u16_rt_tx_ctrl_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));

   //write size in 16 bits
   *(int*)(rt_tx_data_buffer + u16_rt_tx_data_in_index) = s16_size;
   u16_rt_tx_data_in_index = ((u16_rt_tx_data_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));

   //write data
   for(i=0;i<s16_size;i++)
   {
       *(int*)(rt_tx_data_buffer + u16_rt_tx_data_in_index) = p_data[i];

       //increament pointer
       u16_rt_tx_data_in_index = ((u16_rt_tx_data_in_index + 1) & (FIELDBUS_RT_BUFFER_SIZE - 1));
   }
}

int s16_dbg_must_be_0 = 0;
//**********************************************************
// Function Name: FalDoNothingObject
// Description:
//          This function does nothing. It used in order to not lead to a call to NULL.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FalDoNothingObject, "ramcan");
int FalDoNothingObject(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   s16_msg_id += 0;
   p_data += 0;
   u16_operation += 0;

   // This function should never be called
   // if this code is reached it means we have a mistake
   s16_dbg_must_be_0++;
   return FAL_INTERNAL_FW_FAULT;
}


// in params: index - canopen index
//            subIndex - canopen sub index
// return -1 if not found or line number in table FB_objects_array
int FindFbObjectArrayLine(unsigned int index, unsigned char subIndex)
{
   int i, can_open_table_index;

   if (subIndex != 0)
   {
       // all manufacturer specific P params have sub inex zero
       return -1;
   }

   for(i = 0; i < COMMANDS_TABLE_SIZE; i++)
   {
       // find canopen index of p_param in command table (in p param canopen col).
       if (Commands_Table[i].u16_can_object_number == index)
       {
            break;
       }
   }

   if (i >= COMMANDS_TABLE_SIZE)
   {
       return -1;
   }

   can_open_table_index = Commands_Table[i].u16_fildbus1_index;


   for (i=1; i<SDO_LAST_OBJECT; i++)
   {
       if (FB_objects_array[i].u16_id == can_open_table_index)
       {
            return i;
       }
   }

   return -1;
}


int s16_Can_TpdoSender_retval[4]={0,0,0,0};
#pragma CODE_SECTION(Can_TpdoSender, "ramfunc_3");
void Can_TpdoSender(int s16_trd_context, int drive)
{
   int iii,i,j, sendPdo;
   int retVal;
   int s16_pdoTableIndex;

   unsigned char u8_userData[8];
   int s16_userDataIndex;

   PDO_T   *pPdo;    /* pointer to actual pdo structure */
   int s16_can_enter_nmt_operational;
   unsigned long long u64_tmp_val = 0;
   static int s16_prev_state = UNKNOWN;


   REFERENCE_TO_DRIVE;     // Defeat compilation error

   EXIT_IF_NOT_FIELDBUS;

   // entering nmt operational flag.
   // to send PDOs when entering nmt operational regardless to values change event
   s16_can_enter_nmt_operational = ((s16_prev_state != OPERATIONAL) && (GL_ARRAY(co_Node).eState == OPERATIONAL)) ? 1 : 0;
   // save current nmt state for next cycle
   s16_prev_state = GL_ARRAY(co_Node).eState;

   // pdos should be sent only in operational state
   if (GL_ARRAY(co_Node).eState != OPERATIONAL) return;

   // go over all TPDOs
   for (i=0; i<4; i++)
   {
       pPdo = pdoExist(i+1, TRANSMIT_PDO CO_COMMA_LINE_PARA);
       // if pdo exsists and its transmission type is not synchronous and
       // inhibit time has expired and cobId is not zero (i.e. pdo is enabled), handle it here
       if ((pPdo != NULL) && ((pPdo->flags & (PDOFLAG_CYCLIC | PDOFLAG_ONLY_RTR)) == 0) && (pPdo->inhibit.ticks == 0) && (pPdo->pCOB->cobId))
       {
            sendPdo = 0;
            s16_userDataIndex = 0;
            memset(u8_userData,0, sizeof(u8_userData));

            // check if value changed in tpdo #i
            for (j=0; j<4; j++)
            {
                 // find line index of array FB_Pdos_Array
                 s16_pdoTableIndex = BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[i][j];
                 if (s16_pdoTableIndex >= 0)
                 {
                      UpdateTpdoObjectValue(s16_pdoTableIndex);

                      if (!Can_BuildPdoUserData(FB_Pdos_Array[s16_pdoTableIndex].u16_var_size & 0xff,
                                               FB_Pdos_Array[s16_pdoTableIndex].p_canopen_var_addr,
                                               u8_userData,
                                               &s16_userDataIndex))
                      {
                           // error building the user data array.
                           // means that pdo mapping is corrupted.
                           return;
                      }
                 }
            }    // end of for (j=0; j<4; j++)


            // convert u8 array into u64 value
            u64_tmp_val = 0;
            for (iii = 0; iii< s16_userDataIndex; iii++)
            {
                 u64_tmp_val |= ( ((unsigned long long)u8_userData[iii]) & 0xff ) << (56 - (8 * iii));
            }



            if (s16_can_enter_nmt_operational ||
                 ((u64_tmp_val - BGVAR(u64_CAN_Tpdo_Arr_Prev_Val)[i]) & BGVAR(u64_Pdo_Event_Mask_Val)[i]) )    // compare u8 array and u64 prev val cnsiderign pdo event mask
            {
                 BGVAR(u64_CAN_Tpdo_Arr_Prev_Val)[i] = u64_tmp_val;    // copy u8 array to u64 prev val
                 sendPdo++;
            }

            // if one of the params has changed need to send tpdo
            if (sendPdo)
            {
                 retVal = writePdoReq(s16_trd_context, pPdo, u8_userData, i+1);
                 s16_Can_TpdoSender_retval[i] = retVal;
            }
       }
   }   // end of for (i=0; i<4; i++)
}


//**********************************************************
// Function Name: UpdateTpdoObjectValue
// Description:
//          update the port lib variable for an object.
//          for building TPDO data, the port variables which are mapped to the objects must
//          be updated to the most recent value (and peform unit convertion where necessary).
//          This funtion will accept the index of the canopen object in BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index) array
//          and will update the canopen object.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(UpdateTpdoObjectValue, "ramfunc_3");
void UpdateTpdoObjectValue(int s16_pdoTableIndex)
{
   int s16_bufValue[4];
   int s16_internalParamSize;
   unsigned long long u64_tmp_val = 0;


   s16_internalParamSize = (FB_Pdos_Array[s16_pdoTableIndex].u16_var_size>>8) & 0xff;

   // switch size of internal param and get the internal value
   if (s16_internalParamSize == 1)
   {
      u64_tmp_val = *(int*)(FB_Pdos_Array[s16_pdoTableIndex].p_internal_var_addr);
     u64_tmp_val &= 0x00FFULL;
   }
   else if (s16_internalParamSize == 2)
   {
      u64_tmp_val = *(int*)(FB_Pdos_Array[s16_pdoTableIndex].p_internal_var_addr);
   }
   else if (s16_internalParamSize == 4)
   {
      u64_tmp_val = *(long*)(FB_Pdos_Array[s16_pdoTableIndex].p_internal_var_addr);
   }
   else if (s16_internalParamSize == 8)
   {
      // fix bug 3780
      u64_tmp_val = *(long long*)(FB_Pdos_Array[s16_pdoTableIndex].p_internal_var_addr);
   }

   // Fix IPR 1314: Actual position on TPDO does not match to the SDO request value
   // always use 64 bit in array. the pdo RT function (called in FB_Pdos_Array[s16_pdoTableIndex].fp_rt()) will use the part
   // of buffer according to the object that is handled
   s16_bufValue[0] =(int)(u64_tmp_val & 0xffff);
   s16_bufValue[1] =(int)((u64_tmp_val >> 16) & 0xffff);
   s16_bufValue[2] =(int)((u64_tmp_val >> 32) & 0xffff);
   s16_bufValue[3] =(int)((u64_tmp_val >> 48) & 0xffff);

   // call the canopen RT fal function with the appropriate buffer
   FB_Pdos_Array[s16_pdoTableIndex].fp_rt(s16_bufValue, 0, 0);
}

// 1 for success
#pragma CODE_SECTION(Can_BuildPdoUserData, "ramfunc_3");
int Can_BuildPdoUserData(int s16_canopenParamSize, void* p_value, unsigned char u8_userData[], int* s16_userDataIndex)
{
   //unsigned int u16_value;
   //unsigned long u32_value;
   //unsigned long long u64_value;
   int i;
   unsigned int buff[8];
   int tmp, wordIndex;

   // convert to words for memcpy command
   tmp = s16_canopenParamSize;
   // handle u8 objects
   if (tmp > 1)
       tmp = tmp >> 1;
   memcpy(buff, p_value, tmp);

   wordIndex = 0;
   for (i=0; i < s16_canopenParamSize; i++)
   {
       if (*s16_userDataIndex < 8)
       {
            // copy the upper/lower byte from the word
            if ((i & 1) == 0)
                 u8_userData[(*s16_userDataIndex)++] = (buff[wordIndex]) & 0xff;
            else
            {
                 u8_userData[(*s16_userDataIndex)++] = (buff[wordIndex]>>8) & 0xff;
                 wordIndex++;
            }
       }
   }

   return (*s16_userDataIndex <= 8);
}

void Can_UpdatePdoTable(int drive)
{
   int i,j, y,z;
   unsigned int subIndex, index, pdoIndex, numMappedObjects;
   unsigned int pdoIndexBase = 0x1A00;
   RET_T retVal;
   UNSIGNED8 *pData;
   UNSIGNED32 size, u32_mappedObj;
//   int s16_mappingMaskIndex;
//   unsigned long long u64_tmp;
//   int iii;


   REFERENCE_TO_DRIVE;     // Defeat compilation error

   EXIT_IF_NOT_FIELDBUS;

   if (BGVAR(u16_CAN_Tpdo_Need_Update) == 0) return;

   // update the mapping table
   for (i=0; i<4; i++)
   {
       // check if pdo #i requires update
       if (BGVAR(u16_CAN_Tpdo_Need_Update) & (1<<i))
       {
            // reset tpdo mapping
            for (j=0; j<4; j++)
            {
                 BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[i][j] = -1;
            }

            // actual index of tpdo
            pdoIndex = pdoIndexBase + i;


            // get number of mapped objects in this pdo
            retVal = getObjAddr(pdoIndex, 0, &pData, &size);
            memcpy(&numMappedObjects, pData, size);

            // init pdo event mask variable
            BGVAR(u64_Pdo_Event_Mask_Val)[i] = 0;
     //       s16_mappingMaskIndex = 8;

            j=0;
            while (j<4 && j<numMappedObjects)
            {
                 // get address and size of pdo mapped object
                 retVal = getObjAddr(pdoIndex, j+1, &pData, &size);
                 if (retVal == CO_OK)
                 {
                      memcpy(&u32_mappedObj, pData, size);

                      index = (u32_mappedObj >> 16) & 0xffff;
                      subIndex = (u32_mappedObj >> 8) & 0xff;


                      // find object in objects table
                      for(z = 1; z < SDO_LAST_OBJECT; z++)
                      {
                           if ((FB_objects_array[z].u16_index == index) && (FB_objects_array[z].u8_sub_index == subIndex))
                           {
                                break;
                           }
                      }
                      if (z < SDO_LAST_OBJECT)
                      {
                           // find in pdo table
                           for(y = 0; y < NUM_OF_PDO_MAPABLE_OBJS; y++)
                           {
                                if (FB_Pdos_Array[y].u16_id == FB_objects_array[z].u16_id)
                                {
                                     break;
                                }
                           }
                           if (y < NUM_OF_PDO_MAPABLE_OBJS)
                           {
                                /*
                                // convert size in bytes to TI words.
                                if (((FB_Pdos_Array[y].u16_var_size>>4) & 0xff) == 8)
                                     size = 4;
                                else
                                     size = (((FB_Pdos_Array[y].u16_var_size>>4) & 0xff) == 4) ? 2 : 1;
                                */

                                // set new mapping
                                BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[i][j] = y;
                                BGVAR(u64_CAN_Tpdo_Arr_Prev_Val)[i] = 0;
                                //memcpy(&(BGVAR(u64_CAN_Tpdo_Arr_Prev_Val)[i]), FB_Pdos_Array[y].p_internal_var_addr, size);

                                /*
                                u64_tmp = 0;
                                // go over canopen param size and set masking
                                for (iii=0; iii < (FB_Pdos_Array[BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[i][j]].u16_var_size & 0xff); iii++)
                                {
                                     s16_mappingMaskIndex--;
                                     if (BGVAR(u16_P3_18_Pdo_Event_Mask)[i] & (1 << j))
                                          u64_tmp |= (0xffLL << (8 * s16_mappingMaskIndex));
                                }

                                // masking for array
                                BGVAR(u64_Pdo_Event_Mask_Val)[i] |= u64_tmp;
                                */
                           }
                      }
                 }

                 j++;
            }    // end of while

            // fix pdo event mask
            STORE_EXECUTION_PARAMS_0_1
            s16_Number_Of_Parameters = 2;
            s64_Execution_Parameter[0] = i;
            s64_Execution_Parameter[1] = BGVAR(u16_P3_18_Pdo_Event_Mask)[i];
            //SalWritePdoEventMaskCommand(drive);
            RESTORE_EXECUTION_PARAMS_0_1
       }
   }


   BGVAR(u16_CAN_Tpdo_Need_Update) = 0;
}

//**********************************************************
// Function Name: FalIsInOperationMode
// Description:
//          This function determines if we are currentlly in operation mode (CAN or EtherrCAT).
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int FalIsInOperationMode()
{
   return FalIsInOperationModeAndPdo(1);
}

//**********************************************************
// Function Name: FalIsInOperationModeAndPdo
// Description:
//          Refactor FalIsInOperationMode() to allow getting NMT state regardless if pdo arrived or not
//
//  input param:
//     u16_consider_pdo: if zero, return true if drive in NMT state OPERATIONAL
//                       otherwise, return true if drive in NMT state OPERATIONAL and at least one RPDO was recieved by the drive
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int FalIsInOperationModeAndPdo(unsigned int u16_consider_pdo)
{
   int s16_return = 0;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(IS_EC_DRIVE_AND_COMMODE_1) //EC drive
   {
      SET_DPRAM_FPGA_MUX_REGISTER(0);
     if ((*(unsigned int*)p_u16_tx_nmt_state) == EC_NMTSTATE_OP) //if we are in EC operational
     {
         if ((u16_consider_pdo == 0) || (u16_Is_Pdo_After_Operation_Mode))  //if already had 1 or more PDOs
         s16_return = 1;
     }
   }
   else if(IS_CAN_DRIVE_AND_COMMODE_1) //CAN drive
   {
      if ((GL_ARRAY(co_Node).eState) == OPERATIONAL) //if we are in CAN operational
     {
         if ((u16_consider_pdo == 0) || (u16_Is_Pdo_After_Operation_Mode))  //if already had 1 or more PDOs
         s16_return = 1;
     }
   }
   //else not fieldbus drive so not operational
   return s16_return;
}


//**********************************************************
// Function Name: FalVoltageStateBg
// Description:
//          This function retrieve the automatic calibration process status
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageStateBg(int* p_data,int s16_msg_id ,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
    p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO215BS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_u16_Voltage_Correct_State_Command =  BGVAR(u16_Voltage_Correct_State);
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_u16_Voltage_Correct_State_Command;
       ret_val = FalInitUploadRespond((int)SDO215BS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return ret_val;
}


//**********************************************************
// Function Name: FalVoltageNvStateBg
// Description:
//          This function retrieve calibration process status
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageNvStateBg(int* p_data,int s16_msg_id ,unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
    p_data += 0;
      ret_val = FalInitDownloadRespond((int)SDO215CS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_u16_Voltage_Correct_NvSave_Status_Command = BGVAR(u16_Voltage_Correct_NvSave_Status);
      p_fieldbus_bg_tx_data[0] = (int)manu_spec_u16_Voltage_Correct_NvSave_Status_Command;
       ret_val = FalInitUploadRespond((int)SDO215CS0, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }

   return ret_val;
}


//**********************************************************
// Function Name: FalSfbVoltageBacklashPositionBg
// Description:
//          This function is called in response object 0x215D from Fieldbus.
//
//
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int FalSfbVoltageBacklashPositionBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   long s32_tmp=0;
   long long s64_tmp = 0LL;

   p_data += 0; //For Remark Avoidance
   if(u16_operation == 1)// write operation
   {
        //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

      // bugzilla 4057: fix erorr code for value 0xffffffff
         s64_tmp = ((long long)s32_tmp) & 0x00000000ffffffff; // zero upper 32 bit

         s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

       BGVAR(s64_Voltage_Correct_Backlash_Position) = s64_tmp;

         ret_val = FalInitDownloadRespond((int)SDO215DS0, s16_msg_id);
   }
   else// read operation
   {
      s64_tmp = BGVAR(s64_Voltage_Correct_Backlash_Position);

       manu_spec_s64_Voltage_Backlash_Position_Command = (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);
       p_fieldbus_bg_tx_data[0] = (manu_spec_s64_Voltage_Backlash_Position_Command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_s64_Voltage_Backlash_Position_Command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO215DS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalVoltageFastSpeedCommandBg
// Description:
//          This function is called in response object 0x215F from Fieldbus.
//
//
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageFastSpeedCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int ret_val = FAL_SUCCESS;
   long long s64_tmp=0;

   if(u16_operation == 1)// write operation
   {
        //we need to take data int by int to prevent bad data due to odd address
        s32_tmp = (long)(p_data[1] & 0xFFFF);
        s32_tmp <<= 16;
        s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

        // bugzilla 4057: fix erorr code for value 0xffffffff
        s64_tmp = ((long long)s32_tmp) & 0x00000000ffffffff; // zero upper 32 bit

        s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

        BGVAR(s32_Voltage_Correct_Fast_Speed) = (unsigned long)(s64_tmp & 0x00000000FFFFFFFF);
        ret_val = FalInitDownloadRespond((int)SDO215FS0, s16_msg_id);
   }
   else// read operation
   {
         s64_tmp = BGVAR(s32_Voltage_Correct_Fast_Speed);

       manu_spec_s32_Voltage_Correct_Fast_Speed_Command =  (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

         p_fieldbus_bg_tx_data[0] = (manu_spec_s32_Voltage_Correct_Fast_Speed_Command & 0xFFFF); //set 16 LSBits
         p_fieldbus_bg_tx_data[1] = (manu_spec_s32_Voltage_Correct_Fast_Speed_Command >> 16);    //set 16 MSBits

         ret_val = FalInitUploadRespond(SDO215FS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}



//**********************************************************
// Function Name: FalVoltageSlowSpeedCommandBg
// Description:
//          This function is called in response object 0x2160 from Fieldbus.
//
//
// Author: Meital
// Algorithm:
// Revisions:
//**********************************************************
int FalVoltageSlowSpeedCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int ret_val = FAL_SUCCESS;
   long long s64_tmp=0;

   if(u16_operation == 1)// write operation
   {
        //we need to take data int by int to prevent bad data due to odd address
        s32_tmp = (long)(p_data[1] & 0xFFFF);
        s32_tmp <<= 16;
        s32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

        // bugzilla 4057: fix erorr code for value 0xffffffff
        s64_tmp = ((long long)s32_tmp) & 0x00000000ffffffff; // zero upper 32 bit

        s64_tmp = MultS64ByFixS64ToS64(s64_tmp,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                    BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

        BGVAR(s32_Voltage_Correct_Slow_Speed) = (unsigned long)(s64_tmp & 0x00000000FFFFFFFF);
        ret_val = FalInitDownloadRespond((int)SDO2160S0, s16_msg_id);
   }
   else// read operation
   {
      s64_tmp = BGVAR(s32_Voltage_Correct_Slow_Speed);

      manu_spec_s32_Voltage_Correct_Slow_Speed_Command =  (long)MultS64ByFixS64ToS64(s64_tmp,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

       p_fieldbus_bg_tx_data[0] = (manu_spec_s32_Voltage_Correct_Slow_Speed_Command & 0xFFFF); //set 16 LSBits
       p_fieldbus_bg_tx_data[1] = (manu_spec_s32_Voltage_Correct_Slow_Speed_Command >> 16);    //set 16 MSBits

       ret_val = FalInitUploadRespond(SDO2160S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: SalWritePdoEventMaskCommand
// Description:
//    This function writes the param P3-18 to P3-21
//    array starts from index 0
//    These params will mask the transmit pdo object when tpdo is set to be event tpdo.
//    i.e. if masked, a change in the object value will not cause the pdo to be transmitted
//    each bit masked a different object (bit zero mask object #1 in the pdo, bit one mask object #2 in pdo and so on).
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePdoEventMaskCommand(int drive)
{
   long long index = s64_Execution_Parameter[0];
   unsigned long long value = (unsigned long long)s64_Execution_Parameter[1];
   int s16_mappingMaskIndex;
   unsigned long long u64_tmp;
   int j, iii;
   unsigned int numMappedObjects;
   UNSIGNED8 *pData;
   UNSIGNED32 size;
   int retVal;
   unsigned int var_size, pdoIndexBase = 0x1A00;


   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 3LL))
       return (VALUE_OUT_OF_RANGE);

   if (value > 0xFLL)
       return (VALUE_TOO_HIGH);

   // get number of mapped objects in this pdo
   retVal = getObjAddr(pdoIndexBase + index, 0, &pData, &size);
   if (retVal != CO_OK)
   {
       return retVal;
   }

   memcpy(&numMappedObjects, pData, size);

   // init pdo event mask variable
   BGVAR(u64_Pdo_Event_Mask_Val)[index] = 0;
   s16_mappingMaskIndex = 8;

   j=0;
   while (j<4 && j<numMappedObjects)
   {
       u64_tmp = 0;
       var_size = FB_Pdos_Array[BGVAR(s16_CAN_Tpdo_Arr_Pdo_Table_Index)[index][j]].u16_var_size & 0xff;

       // go over canopen param size and set masking
       for (iii=0; iii < var_size; iii++)
       {
            s16_mappingMaskIndex--;
            if (value & (1 << j))
                 u64_tmp |= (0xffLL << (8 * s16_mappingMaskIndex));
       }

       // masking for array
       BGVAR(u64_Pdo_Event_Mask_Val)[index] |= u64_tmp;
       j++;
   }

   //    array starts from index 0
   BGVAR(u16_P3_18_Pdo_Event_Mask)[index] = (unsigned int)value;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPdoEventMaskCommand
// Description:
//    This function reads the param P3-18 to P3-21
//    array starts from index 0
//    These params will mask the transmit pdo object when tpdo is set to be event tpdo.
//    i.e. if masked, a change in the object value will not cause the pdo to be transmitted
//    each bit masked a different object (bit zero mask object #1 in the pdo, bit one mask object #2 in pdo and so on).
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPdoEventMaskCommand(long long *data, int drive)
{
   long long index = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 3LL))
       return (VALUE_OUT_OF_RANGE);

   *data = BGVAR(u16_P3_18_Pdo_Event_Mask)[index];
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalInternalLimitSourceCommand
// Description:
//    This function writes the param P8-40
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalInternalLimitSourceCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((param <= 4) || (param == 9) || (param == 11))
   {
      BGVAR(u16_P3_30_Intern_Lim_Source) = (unsigned int)param;
      return SAL_SUCCESS;
   }
   else
      return (VALUE_OUT_OF_RANGE);
}

//**********************************************************
// Function Name: UpdateCanStrings
// Description:
//    This function update the const strings in the canopen objects dictionary
//    lxm28 and cdhd have different strings (product name, manufacturer nam and etc')
//    this function updates these strings on startup.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void UpdateCanStrings(void* pObject_descriptor, char* pObject_str, char* pNew_str)
{
   int len = strlen(pNew_str);
   unsigned char* ptr = ((VALUE_DESC_T*)pObject_descriptor)[0].pDefaultVal;

   // copy string nto object disctinary and update actual-len and max-len fields
   strcpy(pObject_str, pNew_str);
   *ptr = len;
   ptr += sizeof(long);
   *ptr = len;
}

//**********************************************************
// Function Name: SetCanOpenIdentityObject
// Description:
//    This function update the const object: identity object (0x1018) in the canopen objects dictionary
//    lxm28 and cdhd have different valus (vendor id, prodcut number and etc')
//    this function updates these values on startup and in case of reset comm and reset node nmt commands
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void SetCanOpenIdentityObject(int drive)
{
   //long long s64_prodNumber;
   unsigned long xx, yy, zz;
   int i = 0;
   drive += 0;
   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
       p301_identity.vendorId = 0x0800005A;
       //SalReadSchneiderProgramNumber(&s64_prodNumber, drive);
       //p301_identity.productCode = s64_prodNumber & 0xffffFFFF;


       // version string is: PxxxxyyVxxyyzz
       // revision number is 0x00xxyyzz of the version string
       // e.g. if version string is "P096010V010799", then revision number is 0x00010799
       xx = (p_s8_Shndr_Software_Version[8]- '0') * 0x10 + (p_s8_Shndr_Software_Version[9]- '0');
       yy = (p_s8_Shndr_Software_Version[10]- '0') * 0x10 + (p_s8_Shndr_Software_Version[11]- '0');
       //zz = (p_s8_Shndr_Software_Version[12]- '0') * 0x10 + (p_s8_Shndr_Software_Version[13]- '0');


       // nitsan 10/3/2014: by schnieder request, zz is set to zero always for object 0x1018sub3
       zz = 0;
       p301_identity.revisionNumber = (xx << 16) | (yy << 8) | zz;
   }
   else// CDHD
   {
      p301_identity.revisionNumber = 0; 

       //1. High Most significant is one digit
      if(p_s8_CDHD_Drive_Version[i+1] == '.')
      {
         p301_identity.revisionNumber = ((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') << 24);
         i+=2;//move pointer after '.'
      }
      else//Hi Most significant is two digits
      {
         p301_identity.revisionNumber = (((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') * 0x10 + (unsigned long)(p_s8_CDHD_Drive_Version[i+1] - '0')) << 24);
         i+=3;//move pointer after '.'
      }

      //2. High Least significant is one digit
      if(p_s8_CDHD_Drive_Version[i+1] == '.')
      {
         p301_identity.revisionNumber |= ((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') << 16);
         i+=2;//move pointer after '.'
      }
      else//Hi Most significant is two digits
      {
         p301_identity.revisionNumber |= (((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') * 0x10 + (unsigned long)(p_s8_CDHD_Drive_Version[i+1] - '0')) << 16);
         i+=3;//move pointer after '.'
      }

      //3. Low Most significant is one digit
      if((p_s8_CDHD_Drive_Version[i+1] == 'a') || (p_s8_CDHD_Drive_Version[i+1] == 0))
      {
         p301_identity.revisionNumber |= ((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') << 8);
         i+=6;//move pointer after '.'
      }
      else//Low Least significant is two digits
      {
         p301_identity.revisionNumber |= (((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') * 0x10 + (unsigned long)(p_s8_CDHD_Drive_Version[i+1] - '0')) << 8);
         i+=7;//move pointer after '.'
      }

      //4. 
      if(p_s8_CDHD_Drive_Version[i] != 0)//unofficial version (long)
      {
         if(p_s8_CDHD_Drive_Version[i+1] == 0) //Low least significant is one digit
         {
            p301_identity.revisionNumber |= (unsigned long)(p_s8_CDHD_Drive_Version[i] - '0');
         }
         else//low least significant is two digits
         {
            p301_identity.revisionNumber |= ((unsigned long)(p_s8_CDHD_Drive_Version[i] - '0') * 0x10 + (unsigned long)(p_s8_CDHD_Drive_Version[i+1] - '0'));

         }
      }
      //else official version (short) - Do nothing
   }

   //CAN device id
   p301_identity.serialNumber =
                          ((s8_Product_Serial_Number[11] & 0xff) - 0x30) +
                         (((s8_Product_Serial_Number[10] & 0xff) - 0x30) * 10) +
                         (((s8_Product_Serial_Number[9] & 0xff) - 0x30)  * 100) +
                         (((s8_Product_Serial_Number[8] & 0xff) - 0x30)  * 1000) +
                         (((s8_Product_Serial_Number[7] & 0xff) - 0x30)  * 10000) +
                         (((s8_Product_Serial_Number[6] & 0xff) - 0x30)  * 100000) +
                         (((s8_Product_Serial_Number[5] & 0xff) - 0x30)  * 1000000);

   if(IS_CAN_DRIVE)
      p301_identity.productCode = CAN_PRODUCT_CODE;
   else if (IS_EC_DRIVE)
      p301_identity.productCode = EC_PRODUCT_CODE;
}




//**********************************************************
// Function Name: CanInitSdos
// Description:
//                init SDOs values for lexium 28
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CanInitSdos(int drive)
{
   drive += 0;// for remark avoidance

   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      // fix velocity window according to lxm28 value (in object 0x4328 instead of 0x606D)
      BGVAR(s32_Velocity_Window)  = (long)MultS64ByFixS64ToS64((long long)u32_Lxm28_Velocity_Window,
                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

      // fix velocity threshold according to lxm28 value (in object 0x4329 instead of 0x606F)
      BGVAR(s32_Velocity_Threshold)  = (long)MultS64ByFixS64ToS64((long long)u32_Lxm28_Velocity_Threshold,
                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);
   }
}




//**********************************************************
// Function Name: EtherCATDebugChannel
// Description:
//    This function is called in order to cummunicate between
//    the TI DSP and the uBlaze processor. This function initiates
//    certain debug requests to the uBlaze and provides the response
//    in certain variables. See fieldbus documentation for further details.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int EtherCATDebugChannel(void)
{
   int drive = 0;
   int s16_returnValue = SAL_NOT_FINISHED;

   // Load debug pointers with memory locations reserved for debug channel
   DebugFunctionalityPointer *p_ecat_debug_read_ptr  = (DebugFunctionalityPointer *)DPRAM_RX_ECAT_DBG_CHANNEL_ADDR;
   DebugFunctionalityPointer *p_ecat_debug_write_ptr = (DebugFunctionalityPointer *)DPRAM_TX_ECAT_DBG_CHANNEL_ADDR;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(!IS_EC_DRIVE) // If no EtherCAT Drive
   {
      // Return a specific error number
      s16_returnValue = 20;
      return(s16_returnValue);
   }

   switch(BGVAR(u16_Ecat_Debug_State_Machine))
   {
      case(ECAT_DBG_IDLE):
         // Jump to next state
         BGVAR(u16_Ecat_Debug_State_Machine) = ECAT_DBG_INITIATE_REQUEST;
      break;

      case(ECAT_DBG_INITIATE_REQUEST):
         // Set address-line multiplex register to address-range 0x0000...0x07FF
         SET_DPRAM_FPGA_MUX_REGISTER(0);
         if(p_ecat_debug_read_ptr->u16_Acknowledge_Functionality != 0) // If the uBlaze is busy
         {
            // Return a specific error number
            s16_returnValue = 21;
         }
         else
         {
            // Set address-line multiplex register to address-range 0x1000...0x17FF
            SET_DPRAM_FPGA_MUX_REGISTER(2);

            // Write the request to the DPR.
            p_ecat_debug_write_ptr->s64_Value = BGVAR(s64_Ecat_Debug_Value);              // First write value
            p_ecat_debug_write_ptr->u16_Functionality = BGVAR(u16_Ecat_Debug_Function);   // Now write the functionality type
            p_ecat_debug_write_ptr->u16_Execute_Functionality = 1;                        // At the end triggger the action

            // Capture time for timeout handling
            BGVAR(s32_Ecat_Debug_Function_Timeout) = Cntr_1mS;

            // Jump to next state
            BGVAR(u16_Ecat_Debug_State_Machine) = ECAT_DBG_WAIT_FOR_ACKNOWLEDGE;
         }
      break;

      case(ECAT_DBG_WAIT_FOR_ACKNOWLEDGE):
         // Set address-line multiplex register to address-range 0x0000...0x07FF
         SET_DPRAM_FPGA_MUX_REGISTER(0);

         // Here wait until uBlaze delivers the answer or wait for timeout
         if(PassedTimeMS(700L,BGVAR(s32_Ecat_Debug_Function_Timeout)))
         {
            // Return a specific error number
            s16_returnValue = 22;
         }
         else if(p_ecat_debug_read_ptr->u16_Acknowledge_Functionality != 0) // Acknowledge from the uBlaze arrived
         {
            // Read the response from the uBlaze
            BGVAR(s64_Ecat_Debug_Value_Returned) = p_ecat_debug_read_ptr->s64_Value;

            if(p_ecat_debug_read_ptr->u16_Acknowledge_Functionality == 1) // If uBlaze returns OK
            {
               // Sate that everything is OK
               s16_returnValue = SAL_SUCCESS;
            }
            else
            {
               // Return a specific error number
               s16_returnValue = 23;
            }
         }
      break;
   }

   // Check if the function is finished
   if(s16_returnValue != SAL_NOT_FINISHED)
   {
      // Set address-line multiplex register to address-range 0x1000...0x17FF
      SET_DPRAM_FPGA_MUX_REGISTER(2);
      // Reset request from TI
      p_ecat_debug_write_ptr->u16_Execute_Functionality = 0; // Terminate the action

      // Reset state machine
      BGVAR(u16_Ecat_Debug_State_Machine) = ECAT_DBG_IDLE;
   }

   // Set DPR address-mux register to a default value (=0) when leaving this function
   SET_DPRAM_FPGA_MUX_REGISTER(0);

   return(s16_returnValue);
}

//**********************************************************
// Function Name: SalEtherCATDebugChannel
// Description:
//    This function is a wrapper for the "EtherCATDebugChannel" function. This
//    function just calls "EtherCATDebugChannel". If "EtherCATDebugChannel" returns
//    SAL_SUCCESS, then the value read from the TI is printed out over the RS232
//    in a way that the user will be able to interpret the returned value.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalEtherCATDebugChannel(void)
{
   int drive = 0;
   int s16_temp = 0;
   int s16_returnValue;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Eventually consider here if the output buffer is not empty. This depends
   // on how many characters are supposed to be printed to the RS232 channel.
   if (u8_Output_Buffer_Free_Space < 100)
   {
      return SAL_NOT_FINISHED;
   }

    // Call the debug channel function
   s16_returnValue = EtherCATDebugChannel();

   // If the debug channel function returns success
   if (s16_returnValue == SAL_SUCCESS)
   {
      // Here determine which debug function was triggered and represent the result in a proper manner.
      switch(BGVAR(u16_Ecat_Debug_Function))
      {
         case (0):  // This debug request means: 16-bit read data from ESC memory area
         case (1):  // This debug request means: 16-bit write data to ESC memory area
         case (5):  // This debug request means: 8-Bit read from any uBlaze address
         case (6):  // This debug request means: 16-Bit read from any uBlaze address
         case (7):  // This debug request means: 32-Bit read from any uBlaze address
         case (8):  // This debug request means: 8-Bit write to any uBlaze address
         case (9):  // This debug request means: 16-Bit write to any uBlaze address
         case (10): // This debug request means: 32-Bit write to any uBlaze address
            if((BGVAR(u16_Ecat_Debug_Function) == 0) || (BGVAR(u16_Ecat_Debug_Function) == 5) ||
               (BGVAR(u16_Ecat_Debug_Function) == 6) || (BGVAR(u16_Ecat_Debug_Function) == 7))
            {
               PrintString("Reading ",0);
            }
            else
            {
               PrintString("Writing ",0);
            }
            PrintString("address: ",0);

            // For 8-Bit read/write access
            if ((BGVAR(u16_Ecat_Debug_Function) == 5) || (BGVAR(u16_Ecat_Debug_Function) == 8))
            {
               s16_temp = 2; // Show 2 hexadecimal digits
            }
            // For 16-bit read/write access
            else if ((BGVAR(u16_Ecat_Debug_Function) == 0) || (BGVAR(u16_Ecat_Debug_Function) == 1) ||
                     (BGVAR(u16_Ecat_Debug_Function) == 6) || (BGVAR(u16_Ecat_Debug_Function) == 9))
            {
               s16_temp = 4; // Show 4 hexadecimal digits
            }
            // For 32-Bit read/write access
            else
            {
               s16_temp = 8; // Show 8 hexadecimal digits
            }

            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned) & 0x00000000FFFFFFFFLL,8);         // Write lower 32-bits
            PrintString(" = ",0);
            PrintDecInAsciiHex((BGVAR(s64_Ecat_Debug_Value_Returned) >> 32) & 0x00000000FFFFFFFFLL,s16_temp); // Write upper 32-bits
            PrintCrLf();
         break;

         case (2):
            if(BGVAR(s64_Ecat_Debug_Value_Returned) == 0)
            {
               PrintStringCrLf("Mailbox capture buffer cleared.",0);
            }
            else
            {
               // Print byte per byte via the RS232
               for(s16_temp=0; s16_temp<8; s16_temp++)
               {
                  // Write 1 byte, starting from byte 0 to byte 7
                  PrintDecInAsciiHex((BGVAR(s64_Ecat_Debug_Value_Returned) >> (s16_temp<<3)) & 0x00000000000000FFLL,2);
                  PrintString("  ",0);
               }
               PrintCrLf();
            }
         break;

         case (3):
            PrintStringCrLf("SDO-Index filter variable set.",0);
         break;

         case (4):
            PrintString("SDO-Index filter variable = ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned) & 0x00000000FFFFFFFFLL,8);         // Write lower 32-bits
            PrintCrLf();
         break;

         case (65534):
            PrintString("General purpose debug variable 1 set to: ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned),8);
            PrintCrLf();
            PrintString("General purpose debug variable 2 set to: ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned)>>32,8);
            PrintCrLf();
         break;

         case (65535):
            PrintString("General purpose debug variable 1 = ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned),8);
            PrintCrLf();
            PrintString("General purpose debug variable 2 = ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned)>>32,8);
            PrintCrLf();
         break;

         default: // Unknown action
            PrintStringCrLf("This debug function is not available, please correct ECDEBUGFCT!",0);
         break;
      }
   }
   else if (s16_returnValue != SAL_NOT_FINISHED)
   {
      /* Here the function 'EtherCATDebugChannel' returned a specific error-code. */
      /* Print the proper error message. */
      switch (s16_returnValue)
      {
         case(20):
            PrintStringCrLf("No EtherCAT drive!",0);
         break;

         case(21):
            PrintStringCrLf("uBlaze debug channel is busy!",0);
         break;

         case(22):
            PrintStringCrLf("uBlaze timeout!",0);
         break;

         default: // return value = 23 --> uBlaze reports an error
            PrintString("uBlaze returned error-code: ",0);
            PrintDecInAsciiHex(BGVAR(s64_Ecat_Debug_Value_Returned) & 0x00000000FFFFFFFFLL,8);
            PrintCrLf();
         break;
      }
      // Set return value to SAL_SUCCESS in order to let the parser continue his state-machine.
      // The correct error message has already been printed out.
      s16_returnValue = SAL_SUCCESS;
   }

   return (s16_returnValue);
}


//nitsan - boot up message (cobid 0x700+nodeid) is sent from background if necessary in order to allow sending it also after NMT RESET APP
// fix IPR 520: EMCY message is missing
// send boot up message (cobid 0x700+nodeid)
// also sends EMCY packets for existings faults (and lim switch/ SW limits warnings).
// s16_bootup_only: 1 to only send bootup and no EMCY packets, 0 to send both
void CAN_SendBootUpMsg(int s16_trd_context, int drive, int s16_bootup_only)
{
   unsigned char bData = 0;
   int i, s16_wrn_exists, s16_fault_set;
   long long s64_sys_not_ok;
   int retVal;
   int s16_allow_flash_loading = 1;

   // if not a CAN drive or can communication is off, do nothing
   if (!IS_CAN_DRIVE_AND_COMMODE_1)
   {
      BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_IDLE;
      u16_Can_Reset_Received = 0;
      return;
   }

   // if nodeID of the drive is invalid, just rset the state machine
   if ((GL_ARRAY(coNodeId) > 127) || (GL_ARRAY(coNodeId) <= 0))
   {
      BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_IDLE;
      BGVAR(u16_CAN_BootUp_Requests_Counter) = 0;
   }
   else
   {
      switch (BGVAR(s16_CAN_BootUp_State))
      {
         case CAN_BOOTUP_IDLE:
            if (BGVAR(u16_CAN_BootUp_Requests_Counter) > 0)
            {
               BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_INIT;  // to send bootup message
            }
            break;

         case CAN_BOOTUP_INIT:
            // set defaults for lxm28 and send bootup messgae for canopen
            SetCanOpenIdentityObject(drive);
            CanInitSdos(drive);

/* nitsan 22/11/2015: request from =s= to revert to previous behavior where nvram parameters are not loaded on nmt reset command.
 * this is related to IPRs: 1370, 1403, 1407
            if (u16_Can_Nmt_Cmd_Type == NMT_NO_CMD)
            {
               // do nothing
            }
            else if ((u16_Can_Nmt_Cmd_Type & NMT_RESET_APP_BIT) && (s16_Factory_Restore_On_Power_Up == 0))
            {
               // Fix IPR 1377: Drive display stuck when sending CANopen requests during the booting of the LXM28
               // Scenario: if CANopen/modbus master sends SDOs/requests on startup, the MTP read process is stil not finished (slow down due to load on BG loop).
               //    meanwhile, the canopen bootup process starts to read params from FLASH (due to nmt reset request from master).
               //    the result is that u16_Read_Mtp_State is not being set to MTP_INIT_INIT state
               //    and LexiumBackgroundHandler() is not called. this leads to the display not being refreshed.
               // Fix: verify that the MTP read process is not active and then load data from FLASH.
               //    If MTP process is still active, exit this loop and perform the same state in the next BG loop (i.e. load data from FLASH).
               if (BGVAR(u16_Read_Mtp_State) == MTP_INIT_INIT)
               {
                  // load all parameters
                  do
                  {
                     retVal = LoadFromFlashStateMachine(drive);
                  } while (retVal == SAL_NOT_FINISHED);
               }
               else
               {
                  // signal to exit this loop and continue from same state on next background loop.
                  s16_allow_flash_loading = 0;
               }
            }
            else if (u16_Can_Nmt_Cmd_Type & NMT_RESET_COMM_BIT)
            {
               if (BGVAR(u16_Read_Mtp_State) == MTP_INIT_INIT)
               {
                  // load only communication segment parameters
                  do
                  {
                     retVal = LoadFromFlashStateMachineInternal(drive, LOAD_CAN_COMM_PARAMS);
                  } while (retVal == SAL_NOT_FINISHED);
               }
               else
               {
                  // signal to exit this loop and continue from same state on next background loop.
                  s16_allow_flash_loading = 0;
               }
            }
*/

            if (s16_Factory_Restore_On_Power_Up == 1)
            {
               // for lxm28 only, if restore object 0x1011 was previously requested,
               // need to load factory restore when RESET NODE NMT command is called (which is here)
               s16_Factory_Restore_On_Power_Up = 0;
               BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_FACTORY_RESTORE;
               u16_Can_Nmt_Cmd_Type = NMT_NO_CMD;
               break;
               //  BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_FACTORY_RESTORE;
            }

            if (s16_allow_flash_loading == 1)
            {
               u16_Can_Nmt_Cmd_Type = NMT_NO_CMD;
               BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_UPDATE_COMM_SEG;
            }

            break;

          case CAN_BOOTUP_FACTORY_RESTORE:
            // if no operation on nvram is in progress
            if ((BGVAR(u16_Lexium_Bg_Action_Applied) == 0) && (s16_CAN_EEprom_Operation_Type == 0))
            {
               BGVAR(u16_Factory_Restore_Erase_Param_Only) = 1;  // Indicate to SalRestoreFactorySettingsCommand() to keep SINPARAM and fault log
               retVal = SalRestoreFactorySettingsCommand(drive);
               if (retVal == SAL_SUCCESS)
               {
                  // go to next state
                   BGVAR(s16_CAN_BootUp_State)++;
               }
               else if (retVal != SAL_NOT_FINISHED)
               {
                  // error - how to report ???
                   BGVAR(s16_CAN_BootUp_State)++;
               }
            }

            break;
         case CAN_BOOTUP_UPDATE_COMM_SEG:

            // copy drive variables into port variables
            CanCopyFlashImgToPortLib();

            ResetCommMsg_BG(s16_trd_context);

            // reset heartbeat (HB) consumers counters in case after NMT reset node/ reset comm.
            // this functions clear the internal HB timers if HB was enabled and then NMT reset node was performed.
            setCommPar(BACKGROUND_CONTEXT, 0x1016, 1);
            setCommPar(BACKGROUND_CONTEXT, 0x1016, 2);
            setCommPar(BACKGROUND_CONTEXT, 0x1016, 3);

            BGVAR(s16_CAN_BootUp_State)++;
            break;

        case CAN_BOOTUP_SEND_BOOT_MSG:

            TRANSMIT_COB(s16_trd_context, GL_ARRAY(co_Node).pGuard_COB, &bData);

            // reset last EMCY packet code, so a new packet will be send as a result of reset comm/reset app
            u16_Last_ErrCode = 0x00FF;

            // allow rtr to be handled after bootup message was sent
            u16_Can_Reset_Received = 0;

            if (s16_bootup_only)
            {
               // send EMCY packet with code zero to signal faults are cleared.
               writeEmcyReq(s16_trd_context, 0,(UNSIGNED8 *)"\0\0\0\0\0" CO_COMMA_LINE_PARA);
               BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_FINISH;
               return;
            }

            // all TPDOs need update (each bit is pdo flag)
            BGVAR(u16_CAN_Tpdo_Need_Update) = 0xf;

            // set num of elements in error fields to zero (object 0x1003 sub 0)
            p301_pre_defined_error_field[0] = 0;

            // check lim sw seperatly since they are warnings that fire EMCY
            u64_Warning_Latched = 0;
            s16_QuickStop_Handled = 0;
            s16_wrn_exists = CAN_Emcy_QuickStop(s16_trd_context, drive);
            if ((BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0) && (s16_wrn_exists == 0))
            {
               // send EMCY packet with code zero to signal faults are cleared.
               writeEmcyReq(s16_trd_context, 0,(UNSIGNED8 *)"\0\0\0\0\0" CO_COMMA_LINE_PARA);
            }

            BGVAR(s16_CAN_BootUp_Index) = 0;
            BGVAR(s16_CAN_BootUp_State)++;
            break;

         case CAN_BOOTUP_SET_0:
         case CAN_BOOTUP_SET_1:
         // debug only
            while (BGVAR(s16_CAN_BootUp_State) != CAN_BOOTUP_FINISH)
            {
               if (BGVAR(s16_CAN_BootUp_State) == CAN_BOOTUP_SET_0)
               {
                  s64_sys_not_ok = BGVAR(s64_SysNotOk);
                  s16_fault_set = 0;

                  s64_sys_not_ok = BGVAR(s64_SysNotOk_2);
                  s16_fault_set = 1;
               }
               else
               {
                  s64_sys_not_ok = BGVAR(s64_SysNotOk_2);
                  s16_fault_set = 1;

                  s64_sys_not_ok = BGVAR(s64_SysNotOk);
                  s16_fault_set = 0;
               }

               // go over 4 bits of errors in one SysNotOk set at a BG cycle
               for (i=0; i<4; i++)
               {
                  CAN_BootUp_Emcy_Send(drive, s16_trd_context, s64_sys_not_ok, s16_fault_set, BGVAR(s16_CAN_BootUp_Index));
                  BGVAR(s16_CAN_BootUp_Index)++;
               }

               if (BGVAR(s16_CAN_BootUp_Index) >= 64)
               {
                  BGVAR(s16_CAN_BootUp_Index) = 0;
                  BGVAR(s16_CAN_BootUp_State)++;
               }
            }

            break;

         case CAN_BOOTUP_FINISH:
            BGVAR(u16_CAN_BootUp_Requests_Counter)--;
            BGVAR(s16_CAN_BootUp_State) = CAN_BOOTUP_IDLE;
            break;
      }


   }
}

//**********************************************************
// Function Name: CAN_BootUp_Emcy_Send
// Description:
//    This function send EMCY packet for a specific fault (if exists).
//
//    s64_sys_not_ok: can be either s64_SysNotOk or s64_SysNotOk_2
//    s16_bootup_index: holds the bit for the specific fault
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CAN_BootUp_Emcy_Send(int drive, int s16_trd_context, long long s64_sys_not_ok, int s16_fault_set, int s16_bootup_index)
{
   UNSIGNED8 manuFltCode[] = {0,0,0,0,0};
   unsigned long long u64_flt_bit;

   if (s64_sys_not_ok && (s16_bootup_index <= 64))
   {
      u64_flt_bit = s64_sys_not_ok & (1ULL << s16_bootup_index);
      if (u64_flt_bit)
      {
         // is using s64_SysNotOk_2, need to jump to the next 64 faults
         if (s16_fault_set == 1)
            s16_bootup_index += 64;

         // if the fault has a canopen error code (not zero).
         if (u16_Lex_CanOpen_Fault_Number_Array[s16_bootup_index])
         {
            p402_error_code = u16_Lex_CanOpen_Fault_Number_Array[s16_bootup_index];
            GetEmcyManufcturerCode(drive, u64_flt_bit, s16_fault_set, manuFltCode);
            writeEmcyReq(s16_trd_context, p402_error_code, manuFltCode CO_COMMA_LINE_PARA);
         }
      }
   }
}

//**********************************************************
// Function Name: ClearBusOff
// Description:
//    This function tries to clear the bus off state and restore CAN communication.
//    it is called from background loop and will atttempt every 16ms to clear the busoff state (if exists).
//    For Lxm28, Only enable CAN if drive has a valid CANopen address
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void ClearBusOff()
{
   unsigned int u16_enable_can_on_fpga = 0;
   // check can driver state
   if ((coCanDriverState & CANFLAG_BUSOFF) || (BGVAR(u16_Disable_Can_Req) > 10))
   {
      switch (BGVAR(u16_Can_Busoff_Clr_State))
      {
         case 0:
            BGVAR(s32_Can_Busoff_Clr_Time) = Cntr_1mS;
            BGVAR(u16_Can_Busoff_Clr_State)++;
            break;

         case 1:
            if (PassedTimeMS(16L, BGVAR(s32_Can_Busoff_Clr_Time)))
            {
               Clear_busoff();
               BGVAR(u16_Can_Busoff_Clr_State) = 0;

               BGVAR(u16_Disable_Can_Req) = 0;

               if (BGVAR(u8_Comm_Mode) == 1)
               {
                  u16_enable_can_on_fpga = 1;
               }

               if (u16_enable_can_on_fpga == 1)
               {
                  SET_FPGA_NORMAL_MODE;
               }
            }

            break;
      }
   }
}



int FalShowDisplay1(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       u16_HMI_Display[1] = BGVAR(u16_Local_Hmi_Display_Array)[0];
       p_fieldbus_bg_tx_data[0] = u16_HMI_Display[1];
       ret_val = FalInitUploadRespond(SDO2188S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

int FalShowDisplay2(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       u16_HMI_Display[2] = BGVAR(u16_Local_Hmi_Display_Array)[1];
       p_fieldbus_bg_tx_data[0] = u16_HMI_Display[2];
       ret_val = FalInitUploadRespond(SDO2188S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

int FalShowDisplay3(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       u16_HMI_Display[3] = BGVAR(u16_Local_Hmi_Display_Array)[2];
       p_fieldbus_bg_tx_data[0] = u16_HMI_Display[3];
       ret_val = FalInitUploadRespond(SDO2188S3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
int FalShowDisplay4(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       u16_HMI_Display[4] = BGVAR(u16_Local_Hmi_Display_Array)[3];
       p_fieldbus_bg_tx_data[0] = u16_HMI_Display[4];
       ret_val = FalInitUploadRespond(SDO2188S4, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}
int FalShowDisplay5(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       u16_HMI_Display[5] = BGVAR(u16_Local_Hmi_Display_Array)[4];
       p_fieldbus_bg_tx_data[0] = u16_HMI_Display[5];
       ret_val = FalInitUploadRespond(SDO2188S5, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

int FalHmiMnemonicAddr(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   //p_data += 0;
   s16_msg_id +=0;
   if(u16_operation == 1)// write operation
   {
     manu_spec_HMI_Mnemonic_Ascii[1] = *p_data;
     ret_val = FalInitDownloadRespond((int)SDO2189S1, s16_msg_id);
   }
   else
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_HMI_Mnemonic_Ascii[1];
      ret_val = FalInitUploadRespond(SDO2189S1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

int FalHmiMnemonicVal(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   int index = GetCurrentIndexForExec();

   unsigned long mnem_code1,mnem_code2,mnem_code3;
   char mnem_let = 0;
   char mnem_str[15];
   int j,i = 0;
   
   mnem_let += 0;
   p_data += 0;
   
   mnem_code1  = Commands_Table[index].u32_mnemonic_code1;
   mnem_code2  = Commands_Table[index].u32_mnemonic_code2;
   mnem_code3  = Commands_Table[index].u32_mnemonic_code3;

   //build string from mnemonic code
   j = 0;
   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code1 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) mnem_str[j] = (mnem_let + 47);
         else  mnem_str[j] = ' ';
      ++j;
   }
   j = 5;
   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code2 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) mnem_str[j] = (mnem_let + 47);
         else  mnem_str[j] = ' ';
      j++;
   }
   j = 10;
   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code3 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) mnem_str[j] = (mnem_let + 47);
         else  mnem_str[j] = ' ';
      j++;
   }

   if(u16_operation != 1)// read operation
   {
       manu_spec_HMI_Mnemonic_Ascii[2] = mnem_str[manu_spec_HMI_Mnemonic_Ascii[1]];
       p_fieldbus_bg_tx_data[0] = manu_spec_HMI_Mnemonic_Ascii[2];
       ret_val = FalInitUploadRespond(SDO2189S2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}
int FalHmiBlinkDisplay(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data += 0;
   if(u16_operation != 1)// read operation
   {
       HMI_Blink_Segment = BGVAR(u8_Local_Hmi_Blink_Display);
       p_fieldbus_bg_tx_data[0] = HMI_Blink_Segment;
       ret_val = FalInitUploadRespond(SDO2190S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1ContrlWordCommandBg
// Description:
//          This function is called in response object 0x2191 index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1ContrlWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)(p_data[0] & 0xFFFF);

      ret_val = SalWritePcomCntrl1Command((long long)u16_tmp,drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO2191S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom1_Control_Word = u16_PCom_Cntrl1;
      p_fieldbus_bg_tx_data[0]     = u16_PCom_Cntrl1;
      ret_val = FalInitUploadRespond(SDO2191S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1Contr2WordCommandBg
// Description:
//          This function is called in response object 0x2192 index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2ContrlWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   unsigned int u16_tmp = 0;
   int drive = 0, ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
      u16_tmp = (unsigned int)(p_data[0] & 0xFFFF);

      ret_val = SalWritePcomCntrl2Command((long long)u16_tmp,drive);
      if(ret_val == FAL_SUCCESS)
         ret_val = FalInitDownloadRespond((int)SDO2192S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom2_Control_Word = u16_PCom_Cntrl2;
      p_fieldbus_bg_tx_data[0]     = u16_PCom_Cntrl2;
      ret_val = FalInitUploadRespond(SDO2192S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1StatusWordCommandBg
// Description:
//          This function is called in response object 0x2193 index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1StatusWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read only
   {
      manu_spec_Pcom1_Status_Word = u16_PCom_Stat1;
      p_fieldbus_bg_tx_data[0]     = u16_PCom_Stat1;
      ret_val = FalInitUploadRespond(SDO2193S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2StatusWordCommandBg
// Description:
//          This function is called in response object 0x2194 index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2StatusWordCommandBg(int* p_data, int s16_msg_id, unsigned int u16_operation)
{
   int ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read only
   {
      manu_spec_Pcom2_Status_Word = u16_PCom_Stat2;
      p_fieldbus_bg_tx_data[0]     = u16_PCom_Stat2;
      ret_val = FalInitUploadRespond(SDO2194S0, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1PeriodicStartCommandBg
// Description:
//          This function is called in response object 0x219B index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1PeriodicStartCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_Start1 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO219BS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom1_Periodic_Start = (long)MultS64ByFixS64ToS64((s64_PCom_Start1),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom1_Periodic_Start & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom1_Periodic_Start >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO219BS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2PeriodicStartCommandBg
// Description:
//          This function is called in response object 0x219C index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2PeriodicStartCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_Start2 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO219CS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom2_Periodic_Start = (long)MultS64ByFixS64ToS64((s64_PCom_Start2),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom2_Periodic_Start & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom2_Periodic_Start >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO219CS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1PeriodicEndCommandBg
// Description:
//          This function is called in response object 0x219D index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1PeriodicEndCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_End1 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO219DS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom1_Periodic_End = (long)MultS64ByFixS64ToS64((s64_PCom_End1),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom1_Periodic_End & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom1_Periodic_End >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO219DS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2PeriodicEndCommandBg
// Description:
//          This function is called in response object 0x219E index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2PeriodicEndCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_End2 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO219ES0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom2_Periodic_End = (long)MultS64ByFixS64ToS64((s64_PCom_End2),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom2_Periodic_End & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom2_Periodic_End >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO219ES0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1PeriodicGapCommandBg
// Description:
//          This function is called in response object 0x219F index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1PeriodicGapCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_Peirod1 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO219FS0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom1_Periodic_Gap = (long)MultS64ByFixS64ToS64((s64_PCom_Peirod1),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom1_Periodic_Gap & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom1_Periodic_Gap >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO219FS0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2PeriodicGapCommandBg
// Description:
//          This function is called in response object 0x21A0 index 0 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2PeriodicGapCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   long s32_tmp = 0L;
   int  ret_val = FAL_SUCCESS;

   if(u16_operation == 1)// write operation
   {
         //we need to take data int by int to prevent bad data due to odd address
         s32_tmp = (long)p_data[1] & 0xFFFF;
         s32_tmp <<= 16;
         s32_tmp |= (unsigned long)p_data[0] & 0xFFFF;

         s64_PCom_Peirod2 = MultS64ByFixS64ToS64((long long)s32_tmp,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
         BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         ret_val = FalInitDownloadRespond((int)SDO21A0S0, s16_msg_id);
   }
   else// read operation
   {
      manu_spec_Pcom2_Periodic_Gap = (long)MultS64ByFixS64ToS64((s64_PCom_Peirod2),
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0] = (manu_spec_Pcom2_Periodic_Gap & 0xFFFF); //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = (manu_spec_Pcom2_Periodic_Gap >> 16);    //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO21A0S0, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom1TableEntryAddressCommandBg
// Description:
//          This function is called in response object 0x21A1 sub index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1TableEntryAddressCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      manu_spec_Pcom1_Table_Entry[1] = u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO21A1S1, s16_msg_id);
   }   
   else// read operation
   {
      p_fieldbus_bg_tx_data[0]   = (manu_spec_Pcom1_Table_Entry[1] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (manu_spec_Pcom1_Table_Entry[1] >> 16);       //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO21A1S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalPcom1TableEntryValueCommandBg
// Description:
//          This function is called in response object 0x21A1 sub index 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1TableEntryValueCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      manu_spec_Pcom1_Table_Entry[2] = u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO21A1S2, s16_msg_id);
   }   
   else// read operation
   {

      //Units conversion to FB units
       manu_spec_Pcom1_Table_Entry[2] = (long)MultS64ByFixS64ToS64(s64_Pcom_Table1_Arr[manu_spec_Pcom1_Table_Entry[1] - 1],
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0]   = (manu_spec_Pcom1_Table_Entry[2] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (manu_spec_Pcom1_Table_Entry[2] >> 16);       //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO21A1S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}



//**********************************************************
// Function Name: FalPcom1TableEntryCommandBg
// Description:
//          This function is called in response object 0x21A1 index 3 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom1TableEntryCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation == 1)// write operation
   {
         manu_spec_Pcom1_Table_Entry[3] = 0;

         //do not allow to change point when PCOM feature is enabled
         if(u16_PCom_Stat1 & 0x0001)
            return NOT_ALLOWED_ON_PCOM_EN;
      
         else if((manu_spec_Pcom1_Table_Entry[1] < 0) || (manu_spec_Pcom1_Table_Entry[1] > 255))
            return VALUE_OUT_OF_RANGE;

         else
         {
            //Units conversion + update array
            s64_Pcom_Table1_Arr[manu_spec_Pcom1_Table_Entry[1] - 1]  = MultS64ByFixS64ToS64((long long)manu_spec_Pcom1_Table_Entry[2],
            BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
            BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         }
         ret_val = FalInitDownloadRespond((int)SDO21A1S3, s16_msg_id);
   }
   else// read operation
   {

      p_fieldbus_bg_tx_data[0] = 0; //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0; //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO21A1S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2TableEntryAddressCommandBg
// Description:
//          This function is called in response object 0x21A2 sub index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2TableEntryAddressCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      manu_spec_Pcom2_Table_Entry[1] = u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO21A2S1, s16_msg_id);
   }   
   else// read operation
   {
      p_fieldbus_bg_tx_data[0]   = (manu_spec_Pcom2_Table_Entry[1] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (manu_spec_Pcom2_Table_Entry[1] >> 16);       //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO21A2S1, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}


//**********************************************************
// Function Name: FalPcom2TableEntryValueCommandBg
// Description:
//          This function is called in response object 0x21A2 sub index 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2TableEntryValueCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   unsigned long u32_tmp = 0L;
   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      u32_tmp = (unsigned long)p_data[1] & 0xFFFF;
      u32_tmp <<= 16;
      u32_tmp |= (unsigned long)(p_data[0] & 0xFFFF);

      manu_spec_Pcom2_Table_Entry[2] = u32_tmp;
      ret_val = FalInitDownloadRespond((int)SDO21A2S2, s16_msg_id);
   }   
   else// read operation
   {

      //Units conversion to FB units
       manu_spec_Pcom2_Table_Entry[2] = (long)MultS64ByFixS64ToS64(s64_Pcom_Table2_Arr[manu_spec_Pcom2_Table_Entry[1] - 1],
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix,
                                  BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr);

      p_fieldbus_bg_tx_data[0]   = (manu_spec_Pcom2_Table_Entry[2] & 0xFFFF);    //set 16 LSBits
      p_fieldbus_bg_tx_data[1]   = (manu_spec_Pcom2_Table_Entry[2] >> 16);       //set 16 MSBits
      ret_val = FalInitUploadRespond(SDO21A2S2, s16_msg_id, 2, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalPcom2TableEntryCommandBg
// Description:
//          This function is called in response object 0x21A2 index 3 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalPcom2TableEntryCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation == 1)// write operation
   {
         manu_spec_Pcom2_Table_Entry[3] = 0;

         //do not allow to change point when PCOM feature is enabled
         if(u16_PCom_Stat2 & 0x0001)
            return NOT_ALLOWED_ON_PCOM_EN;
      
         else if((manu_spec_Pcom2_Table_Entry[1] < 0) || (manu_spec_Pcom2_Table_Entry[1] > 255))
            return VALUE_OUT_OF_RANGE;

         else
         {
            //Units conversion + update array
            s64_Pcom_Table2_Arr[manu_spec_Pcom2_Table_Entry[1] - 1]  = MultS64ByFixS64ToS64((long long)manu_spec_Pcom2_Table_Entry[2],
            BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
            BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);
         }
         ret_val = FalInitDownloadRespond((int)SDO21A2S3, s16_msg_id);
   }
   else// read operation
   {

      p_fieldbus_bg_tx_data[0] = 0; //set 16 LSBits
      p_fieldbus_bg_tx_data[1] = 0; //set 16 MSBits

      ret_val = FalInitUploadRespond(SDO21A2S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalDiffPortModeAddressCommandBg
// Description:
//          This function is called in response object 0x21A3 sub index 1 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalDiffPortModeAddressCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{

   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      manu_spec_Diff_Port_Mode_Entry[1] = (unsigned int)(p_data[0] & 0xFFFF);
      ret_val = FalInitDownloadRespond((int)SDO21A3S1, s16_msg_id);
   }   
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = manu_spec_Diff_Port_Mode_Entry[1];
      ret_val = FalInitUploadRespond(SDO21A3S1, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalDiffPortModeValueCommandBg
// Description:
//          This function is called in response object 0x21A3 sub index 2 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalDiffPortModeValueCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{

   int drive = 0, ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(u16_operation == 1)// write operation
   {
      manu_spec_Diff_Port_Mode_Entry[2] = (unsigned int)(p_data[0] & 0xFFFF);
      ret_val = FalInitDownloadRespond((int)SDO21A3S2, s16_msg_id);
   }   
   else// read operation
   {

      p_fieldbus_bg_tx_data[0] = manu_spec_Diff_Port_Mode_Entry[2];
      ret_val = FalInitUploadRespond(SDO21A3S2, s16_msg_id, 1, (int*)p_fieldbus_bg_tx_data);
   }
   return (ret_val);
}

//**********************************************************
// Function Name: FalDiffPortModeEntryCommandBg
// Description:
//          This function is called in response object 0x21A3 index 3 from Fieldbus.
//
//
// Author: Itai
// Algorithm:
//**********************************************************
int FalDiffPortModeEntryCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation == 1)// write operation
   {
         manu_spec_Diff_Port_Mode_Entry[3] = 0;

         s64_Execution_Parameter[0] = manu_spec_Diff_Port_Mode_Entry[1];
         s64_Execution_Parameter[1] = manu_spec_Diff_Port_Mode_Entry[2];

         ret_val = SalDifPortModeCommand();
         if(ret_val != SAL_SUCCESS)
            return ret_val;
    
         ret_val = FalInitDownloadRespond((int)SDO21A3S3, s16_msg_id);
   }
   else// read operation
   {
      p_fieldbus_bg_tx_data[0] = 0; //set 16 LSBits
      ret_val = FalInitUploadRespond(SDO21A3S3, s16_msg_id,2,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalParamsFoeFirstParsingErrorCommandBg
// Description:
//          This function is called in response object 0x21Ad subindex 1 from Fieldbus.
//          this object used for params over FOE
//          zero if all params in file were parsed successfully
//          SAL error value of the first line that parsing failed on
//
// Author: Nitsan
// Algorithm:
//**********************************************************
int FalParamsFoeFirstParsingErrorCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read operation
   {
      manu_spec_Params_Foe_Results_command[1] = u16_Foe_ParamsFile_First_Cmd_Err;
      p_fieldbus_bg_tx_data[0] = manu_spec_Params_Foe_Results_command[1];
      ret_val = FalInitUploadRespond(SDO21ADS1, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}


//**********************************************************
// Function Name: FalParamsFoeIndexFirstFailedCmdCommandBg
// Description:
//          This function is called in response object 0x21AD subindex 2 from Fieldbus.
//          this object used for params over FOE
//          zero if all params in file were parsed successfully
//          number of first command that failed in file
//
// Author: Nitsan
// Algorithm:
//**********************************************************
int FalParamsFoeIndexFirstFailedCmdCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read operation
   {
      manu_spec_Params_Foe_Results_command[2] = u16_Foe_ParamsFile_First_Cmd_Err_Number; 
      p_fieldbus_bg_tx_data[0] = manu_spec_Params_Foe_Results_command[2];
      ret_val = FalInitUploadRespond(SDO21ADS2, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalParamsFoeTotalNumParsingErrorsCommandBg
// Description:
//          This function is called in response object 0x21AD subindex 3 from Fieldbus.
//          this object used for params over FOE
//          zero if all params in file were parsed successfully
//          total number of commands that failed in file
//
// Author: Nitsan
// Algorithm:
//**********************************************************
int FalParamsFoeTotalNumParsingErrorsCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read operation
   {
      manu_spec_Params_Foe_Results_command[3] = u16_Foe_ParamsFile_Total_Parsing_Error; 
      p_fieldbus_bg_tx_data[0] = manu_spec_Params_Foe_Results_command[3];
      ret_val = FalInitUploadRespond(SDO21ADS3, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

//**********************************************************
// Function Name: FalParamsFoeTotalNumParsedCmdsCommandBg
// Description:
//          This function is called in response object 0x21AD subindex 4 from Fieldbus.
//          this object used for params over FOE
//          total number of commands that were parsed in file
//
// Author: Nitsan
// Algorithm:
//**********************************************************
int FalParamsFoeTotalNumParsedCmdsCommandBg(int* p_data,int s16_msg_id,unsigned int u16_operation)
{
   int  ret_val = FAL_SUCCESS;
   p_data+=0;

   if(u16_operation != 1)// read operation
   {
      manu_spec_Params_Foe_Results_command[4] = u16_Foe_ParamsFile_Cmd_Number;
      p_fieldbus_bg_tx_data[0] = manu_spec_Params_Foe_Results_command[4];
      ret_val = FalInitUploadRespond(SDO21ADS4, s16_msg_id,1,(int*)p_fieldbus_bg_tx_data);
   }

   return (ret_val);
}

// state machine for params file download via FOE
void FoeParamsFileDownload()
{
   unsigned int* ptr = 0;
   unsigned int u16_str_len;
   unsigned int u16_local_bg_rx_ctrl_in_index = 0;
   int i;
   
   if (u16_Foe_ParamsFile_State != FOE_PARAMS_FILE_DWLD_IDLE)
   {
      // EtherCAT - cable disconnected, reset state machine
      if (u16_EC_commands_register & EC_CWORD_CABLE_DISCONNECTED_MASK)
      {
         u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_IDLE;
         *p_u16_tx_foe_active = 0;	// remove signature
      }
   }
   
   switch (u16_Foe_ParamsFile_State)
   {
      case FOE_PARAMS_FILE_DWLD_IDLE:
         break;
         
      case FOE_PARAMS_FILE_DWLD_INIT:
         // init the error objects when starting the file downlaod process.
         // dont reset them in idle state because we need to allow read these values via SDO after the download is complete.
         u16_Foe_ParamsFile_Cmd_Number = 0;
         u16_Foe_ParamsFile_First_Cmd_Err_Number = 0;
         u16_Foe_ParamsFile_First_Cmd_Err = 0;
         u16_Foe_ParamsFile_Total_Parsing_Error = 0;
         u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_STR_START;
         break;
         
      case FOE_PARAMS_FILE_DWLD_STR_START:
         // Read the value in DPR address 0x1707 (TOMI memory space).
         // This is an offset for the "LP RX FIFO CTRL" register.
         SET_DPRAM_FPGA_MUX_REGISTER(0);
         u16_local_bg_rx_ctrl_in_index = *(unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_CTRL_ADDR;

         // uB signals that there is error in file, terminate the process.
         if (u16_local_bg_rx_ctrl_in_index & PARAMS_OVER_FOE_FILE_ERROR)
         {
            u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_IDLE;
            *(unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_CTRL_ADDR = 0;
         }
         else if (u16_local_bg_rx_ctrl_in_index & PARAMS_OVER_FOE_CMD_READY)
         {
            // set to len of string (lower byte)
            u16_str_len = u16_local_bg_rx_ctrl_in_index & 0xff;
            if (u16_str_len == 0)
            {
               // send command with len = 0
               u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_PARSE_START;
               break;
            }
            
            ptr = (unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_DATA_ADDR;
            for (i=0; i < u16_str_len; i++)
            {
                u8_Message_Buffer[i] = ptr[i];
            }
            
            // mark end of line
            u8_Message_Buffer[i] = 0;
            OmmitUnits();
            OmmitChecksum();
            u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_PARSE_START;
         }
            
         break;
           
      case FOE_PARAMS_FILE_DWLD_PARSE_START:
      
         // reset result value
         u16_Foe_ParamsFile_Parse_Result = SAL_SUCCESS;
         
         // start parser
         p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
         u8_Parser_State = PARSE_MNEMONIC;
         s16_Comms_Processor_State = PARSING;
         u16_Foe_ParamsFile_Cmd_Number = *(unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_LINE_NUM_ADDR;
         u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_WAIT_PARSE_FINISH;
         break;
         
      case FOE_PARAMS_FILE_DWLD_WAIT_PARSE_FINISH:
         // wait until comm processor is finished
         if (s16_Comms_Processor_State == PRE_PROCESSOR)
         {
            // need to understand what is the result of the command 
            if (u16_Foe_ParamsFile_Parse_Result != SAL_SUCCESS)
            {
               if (u16_Foe_ParamsFile_First_Cmd_Err_Number == 0)
               {
                  u16_Foe_ParamsFile_First_Cmd_Err_Number = u16_Foe_ParamsFile_Cmd_Number;
                  u16_Foe_ParamsFile_First_Cmd_Err = u16_Foe_ParamsFile_Parse_Result;
               }
               
               u16_Foe_ParamsFile_Total_Parsing_Error++;
            }
            
            // report uBlaze to send the next command
            //ptr = (unsigned int*)DPRAM_RX_PARAMS_OVER_FOE_STAT_ADDR;
            //*ptr = 0x5555;
            
            // check if this is the last command in file
            SET_DPRAM_FPGA_MUX_REGISTER(0);
            u16_local_bg_rx_ctrl_in_index = *(unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_CTRL_ADDR;
            //SET_DPRAM_FPGA_MUX_REGISTER(0);
            if (u16_local_bg_rx_ctrl_in_index & PARAMS_OVER_FOE_LAST_CMD)
            {
               // this was the last command so go to idle state (process finished)
               u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_IDLE;
            }
            else
            {
               // prepare for next command
               u16_Foe_ParamsFile_State = FOE_PARAMS_FILE_DWLD_STR_START;
            }
         
            *(unsigned int*)DPRAM_TX_PARAMS_OVER_FOE_CTRL_ADDR = 0;
         }
         
         break;
         
      case FOE_PARAMS_FILE_DWLD_ERROR:
         break;
   }
}
