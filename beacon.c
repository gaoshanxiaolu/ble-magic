/******************************************************************************
 *  Copyright (C) Cambridge Silicon Radio Limited, 2014
 *
 *  FILE
 *      beacon.c
 *
 *  DESCRIPTION
 *      This file defines an advertising node implementation
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <gap_app_if.h>
#include <config_store.h>
#include <pio.h>
#include <random.h>
/*============================================================================*
 *  Local Header File
 *============================================================================*/

#include "beacon.h"
#include <timer.h>
#include <aio.h>
#include "byte_queue.h"
#include <uart.h>
#include <pio.h>
#include <debug.h>
#include <pio.h>
#include <nvm.h>
#include <i2c.h>
#include <panic.h>
#include <types.h>
#include <status.h>

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
#define BTM_DATA_INTEVAL_TIME           (1 * MINUTE)// (500 * MILLISECOND)
#define SLOW_INTEVAL_TIME           (1 * SECOND)
#define FAST_INTEVAL_TIME           (200 * MILLISECOND)
#define PARSE_INTEVAL_TIME           (10 * MILLISECOND)

 /* The application is required to create two buffers, one for receive, the
  * other for transmit. The buffers need to meet the alignment requirements
  * of the hardware. See the macro definition in uart.h for more details.
  */

#define RX_BUFFER_SIZE      UART_BUF_SIZE_BYTES_64
#define TX_BUFFER_SIZE      UART_BUF_SIZE_BYTES_64

#define YAW 0
#define ROLL 1
#define PATCH 2
/* Create 64-byte receive buffer for UART data */
UART_DECLARE_BUFFER(rx_buffer, RX_BUFFER_SIZE);

/* Create 64-byte transmit buffer for UART data */
UART_DECLARE_BUFFER(tx_buffer, TX_BUFFER_SIZE);



static void appSetRandomAddress(void);

static timer_id ble_data_timeout_tid=TIMER_INVALID;
static timer_id fast_blink=TIMER_INVALID;
static timer_id parse_uart_tid =TIMER_INVALID;

int led_state = 0;
static int vol_flag=0;
int up_down_state=0;
#define MAX_APP_TIMERS 6

#define BOARDCAST_UPDATE_INTERVAL (1*SECOND)
#define SIZEOF_APP_TIMER        6
static void fast_blink_led(timer_id tid);

static uint16 app_timers1[SIZEOF_APP_TIMER * MAX_APP_TIMERS];
#define BOARDCAST_INFO_LEN 16
void startAdvertising(uint8 len, uint8 *data, uint8 advInterval_time);
static uint8 boardcast_info[BOARDCAST_INFO_LEN] ;
#define ID_OFFSET 0
#define KEYS_OFFSET (ID_OFFSET + 5)
#define VOLPER_OFFSET (KEYS_OFFSET + 6)
#define BODYFLAG_OFFSET (VOLPER_OFFSET + 1)
#define PACK_SEQ_NUM   (BODYFLAG_OFFSET + 1)
#define CRC16_OFFSET (PACK_SEQ_NUM + 1)
extern void Nvm_Disable(void);
static void add_id_to_boardcast_info(void);
 extern void Nvm_Read(uint16* buffer, uint16 length, uint16 offset);
 uint16 crc_ccitt(uint8 *q, uint8 len);
static void add_id_to_boardcast_info(void)
{
	uint16 chair_id1[3] = {0,0,0};

	Nvm_Read(chair_id1,3,0);

	boardcast_info[ID_OFFSET + 0] = chair_id1[0] >> 8;
	boardcast_info[ID_OFFSET + 1] = chair_id1[0];
	boardcast_info[ID_OFFSET + 2] = chair_id1[1] >> 8;
	boardcast_info[ID_OFFSET + 3] = chair_id1[1];
	boardcast_info[ID_OFFSET + 4] = chair_id1[2];

}

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      Nvm_Disable
 *
 *  DESCRIPTION
 *      This function is used to perform things necessary to save power on NVM 
 *      once the read/write operations are done.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void Nvm_Disable(void)
{
    NvmDisable();
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
}

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
    extern void Nvm_Read(uint16* buffer, uint16 length, uint16 offset)
    {
        sys_status result;
    
        /* Read from NVM. Firmware re-enables the NVM if it is disabled */
        result = NvmRead(buffer, length, offset);
    
        /* Disable NVM to save power after read operation */
        Nvm_Disable();
    

    
    }

/*----------------------------------------------------------------------------*
 *  NAME
 *      appSetRandomAddress
 *
 *  DESCRIPTION
 *      This function generates a non-resolvable private address and sets it
 *      to the firmware.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void appSetRandomAddress(void)
{
    BD_ADDR_T addr;

    /* "completely" random MAC addresses by default: */
    for(;;)
    {
        uint32 now = TimeGet32();
        /* Random32() is just two of them, no use */
        uint32 rnd = Random16();
        addr.uap = 0xff & (rnd ^ now);
        /* No sub-part may be zero or all-1s */
        if ( 0 == addr.uap || 0xff == addr.uap ) continue;
        addr.lap = 0xffffff & ((now >> 8) ^ (73 * rnd));
        if ( 0 == addr.lap || 0xffffff == addr.lap ) continue;
        addr.nap = 0x3fff & rnd;
        if ( 0 == addr.nap || 0x3fff == addr.nap ) continue;
        break;
    }

    /* Set it to actually be an acceptable random address */
    addr.nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;
    addr.nap |=  BD_ADDR_NAP_RANDOM_TYPE_NONRESOLV;
    GapSetRandomAddress(&addr);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      startAdvertising
 *
 *  DESCRIPTION
 *      This function is called to start advertisements.
 *
 *      Advertisement packet will contain Flags AD and Manufacturer-specific
 *      AD with Manufacturer id set to CSR and payload set to the value of
 *      the User Key 0. The payload size is set by the User Key 1.
 *
 *      +--------+-------------------------------------------------+
 *      |FLAGS AD|MANUFACTURER AD                                  |
 *      +--------+-------------------------------------------------+
 *       0      2 3
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
void startAdvertising(uint8 len, uint8 *data, uint8 advInterval_time)
{
    uint8 advData[MAX_ADVERT_PACKET_SIZE];
    uint16 offset = 0;
    //uint8 filler;
    uint16 advInterval;
    uint8 advPayloadSize;
    ls_addr_type addressType = ls_addr_type_public;     /* use public address */
    
    /* initialise values from User CsKeys */
    
    /* read User key 0 for the payload filler */
    //filler = (uint8)(CSReadUserKey(0) & 0x00FF);
    
    /* read User key 1 for the payload size */
    //advPayloadSize = (uint8)(CSReadUserKey(1) & 0x00FF);
    advPayloadSize = len;
		
    /* range check */
    if((advPayloadSize < 1) || (advPayloadSize > MAX_ADVERT_PAYLOAD_SIZE))
    {
        /* revert to default payload size */
        advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;
    }
    
    /* read User key 2 for the advertising interval */
    //advInterval = CSReadUserKey(2);
	advInterval = advInterval_time;
    
    /* range check */
    if((advInterval < MIN_ADVERTISING_INTERVAL) ||
       (advInterval > MAX_ADVERTISING_INTERVAL))
    {
        /* revert to default advertising interval */
        advInterval = DEFAULT_ADVERTISING_INTERVAL;
    }
    
    /* read address type from User key 3 */
    if(CSReadUserKey(3))
    {
        /* use random address type */
        addressType = ls_addr_type_random;

        /* generate and set the random address */
        appSetRandomAddress();
    }

    /* set the GAP Broadcaster role */
    GapSetMode(gap_role_broadcaster,
               gap_mode_discover_no,
               gap_mode_connect_no,
               gap_mode_bond_no,
               gap_mode_security_none);
    
    /* clear the existing advertisement data, if any */
    LsStoreAdvScanData(0, NULL, ad_src_advertise);

    /* set the advertisement interval, API accepts the value in microseconds */
    GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);
    
    /* manufacturer-specific data */
    advData[0] = AD_TYPE_MANUF;

    /* CSR company code, little endian */
    advData[1] = 0x1b;
    advData[2] = 0x2a;
    
    /* fill in the rest of the advertisement */
    for(offset = 0; offset < advPayloadSize; offset++)
    {
        advData[3 + offset] = data[offset];
    }

    /* store the advertisement data */
    LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);
    
    /* Start broadcasting */
    LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);
}
static  uint16 ccitt_table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


uint16 crc_ccitt(uint8 *q, uint8 len)
{
	uint16 crc = 0;

	while (len-- > 0)
		crc = ccitt_table[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return ~crc;

}

static void add_crc16_to_boardcast_info(void)
{
	//uint8 i;
	uint16 sum = 0;
	
	//for(i=0;i<CRC16_OFFSET;i++)
	//	sum += boardcast_info[i];
	sum = crc_ccitt(boardcast_info,CRC16_OFFSET);
	
	boardcast_info[CRC16_OFFSET+0] = (sum & 0xff00) >> 8;
	boardcast_info[CRC16_OFFSET+1] = sum & 0x00ff;

}


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This function is called just after a power-on reset (including after
 *      a firmware panic).
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
    /* UART receive callback to receive serial commands */
    static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                     uint16  length,
                                     uint16 *p_req_data_length);
    
    /* UART transmit callback when a UART transmission has finished */
    static void uartTxDataCallback(void);
    /*----------------------------------------------------------------------------*
     *  NAME
     *      uartTxDataCallback
     *
     *  DESCRIPTION
     *      This is an internal callback function (of type uart_data_out_fn) that
     *      will be called by the UART driver when data transmission over the UART
     *      is finished. See DebugInit in the Firmware Library documentation for
     *      details.
     *
     * PARAMETERS
     *      None
     *
     * RETURNS
     *      Nothing
     *----------------------------------------------------------------------------*/
    static void uartTxDataCallback(void)
    {
        /* Send any pending data waiting to be sent */
        //sendPendingData();
    }

void AppPowerOnReset(void)
{
    /* empty */
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      uartRxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_in_fn) that
 *      will be called by the UART driver when any data is received over UART.
 *      See DebugInit in the Firmware Library documentation for details.
 *
 * PARAMETERS
 *      p_rx_buffer [in]   Pointer to the receive buffer (uint8 if 'unpacked'
 *                         or uint16 if 'packed' depending on the chosen UART
 *                         data mode - this application uses 'unpacked')
 *
 *      length [in]        Number of bytes ('unpacked') or words ('packed')
 *                         received
 *
 *      p_additional_req_data_length [out]
 *                         Number of additional bytes ('unpacked') or words
 *                         ('packed') this application wishes to receive
 *
 * RETURNS
 *      The number of bytes ('unpacked') or words ('packed') that have been
 *      processed out of the available data.
 *----------------------------------------------------------------------------*/
static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_additional_req_data_length)
{
    if ( length > 0 )
    {
        /* First copy all the bytes received into the byte queue */
        BQForceQueueBytes((const uint8 *)p_rx_buffer, length);
    }
    
    /* Send any pending data waiting to be sent */
    //sendPendingData();
    //UartWriteBlocking(boardcast_info,1);
    /* Inform the UART driver that we'd like to receive another byte when it
     * becomes available
     */
    *p_additional_req_data_length = (uint16)1;
    
    /* Return the number of bytes that have been processed */
    return length;
}

bool find_sync_head(uint8 *data, uint8 len);

int skip_nbyte_find_head;

bool find_sync_head(uint8 *data, uint8 len)
{
	uint8 i;

	for(i=0;i<len-1;i++)
	{
		if(data[i] == 0x5A && data[i+1] == 0x5A)
		{
			skip_nbyte_find_head = i;
			return TRUE;
		}
	}

	return FALSE;
}

#define DEFINE_FRAME_SIZE 12
static unsigned char buf_frame[64];
void parse_uart_data(timer_id tid);
int control_mode;
 short int oula_angle[3];//
unsigned short int siyuan_data[4];
int cnt;
int updonw_judge(void);

int updonw_judge(void)
{
    if(oula_angle[PATCH] > 10)
    {
        return 1;
    }
    else if(oula_angle[PATCH] < -10)
    {
        return 2;
    }
    else
    {
        return 0;
    }
}
int start_boardcast = 0;
int status_judge(void);
int status_judge(void)
{
    static int state = 0;
    static int tcnt=0;
    int flag;

    flag = 0;

    if(state == 0)
    {
        if(oula_angle[ROLL] > 10 )
        {
            state = 1;
            tcnt = 0;
        }

        if(oula_angle[ROLL] < -10)
        {
            
        }

    }
    else if(state == 1)
    {
        
        if(oula_angle[ROLL] < -10)
        {
            state = 2;
            tcnt = 0;
        }
        else
        {
            if(++tcnt>300)
            {
                state = 0;
            }
        }
    }
    else if(state == 2)
    {
        if(oula_angle[ROLL] > 10)
        {
            state = 3;
            tcnt = 0;
        }
        else
        {
            if(++tcnt>300)
            {
                state = 0;
            }
        }
    }
    else if(state == 3)
    {
        
        if(oula_angle[ROLL] < -10)
        {
            state = 0;
            tcnt = 0;
            flag = 1;
        }
        else
        {
            if(++tcnt>300)
            {
                state = 0;
            }
        }
    }


    return flag;

}
void parse_uart_data(timer_id tid)
{
	int len,i,sum;
	bool find_head;
	unsigned char cmd_type;
    int frame_len;
    int offset;

    if( tid == parse_uart_tid)
    {
    	len = BQGetDataSize();
    	
    	if( len >= DEFINE_FRAME_SIZE)
    	{
    		BQPeekBytes(buf_frame,len);

    		find_head = find_sync_head(buf_frame,len);

    		if(!find_head)
    		{
                //UartWriteBlocking(buf_frame,len);
    			BQPopBytes(buf_frame,len - 1);
    			goto exit;
    		}

    		if(skip_nbyte_find_head > 0)
    		{
                //UartWriteBlocking(buf_frame,len);
    			BQPopBytes(buf_frame,skip_nbyte_find_head);
    			goto exit;
    		}

            frame_len = buf_frame[3]+4;
            if(frame_len + 1 > len)
            {
                goto exit;
            }

            sum=0;
            for(i=0;i<frame_len;i++)
            {
                sum += buf_frame[i];
            }

    		if((buf_frame[frame_len] != (sum & 0x00ff)))//check sum
    		{
    			BQPopBytes(buf_frame,1);
    			goto exit;
    		}

    		cmd_type = buf_frame[2];
    		//UartWriteBlocking(&cmd_type,1);
    		//UartWriteBlocking(buf_frame,DEFINE_FRAME_SIZE);
    		#define DATA_POS 4
    		offset = 0;
    		if((cmd_type&0x01) == 1)//jia su du
    		{
    			offset += 6;
    		}

            if((cmd_type&0x02) == 2)//ci chang
            {
                offset += 6;;
            }

            if((cmd_type&0x04) == 4)//tuo luo yi
            {
                offset += 6;
            }

            if((cmd_type&0x08) == 8)//ou la jiao
            {
                oula_angle[0] = buf_frame[DATA_POS+offset] << 8 | buf_frame[DATA_POS+offset+1];
                oula_angle[1] = buf_frame[DATA_POS+offset+2] << 8 | buf_frame[DATA_POS+offset+3];
                oula_angle[2] = buf_frame[DATA_POS+offset+4] << 8 | buf_frame[DATA_POS+offset+5];

                for(i=0;i<3;i++)
                    oula_angle[i] /= 100;
                
                
                for(i=0;i<6;i++)
                    boardcast_info[KEYS_OFFSET+i] = buf_frame[DATA_POS+offset+i];
                offset += 6;
                
            }

            if((cmd_type&0x10) == 0x10)//si yuan shu
            {
                siyuan_data[0] = buf_frame[DATA_POS+offset] << 8 | buf_frame[DATA_POS+offset+1];
                siyuan_data[1] = buf_frame[DATA_POS+offset+2] << 8 | buf_frame[DATA_POS+offset+3];
                siyuan_data[2] = buf_frame[DATA_POS+offset+4] << 8 | buf_frame[DATA_POS+offset+5];
                siyuan_data[3] = buf_frame[DATA_POS+offset+6] << 8 | buf_frame[DATA_POS+offset+7];

            }

    		//BQCommitLastPeek();
            BQPopBytes(buf_frame,frame_len+1);
    		
    	}
exit:
        parse_uart_tid =	TimerCreate(PARSE_INTEVAL_TIME, TRUE, parse_uart_data);

        boardcast_info[BODYFLAG_OFFSET] = updonw_judge();///0 stop , 2 up ,1 down
        up_down_state = boardcast_info[BODYFLAG_OFFSET];
        control_mode = status_judge();;
        if(control_mode)
        {
            if(led_state == 0 || led_state == 1)
            {
                led_state = 3;
                fast_blink =	TimerCreate(FAST_INTEVAL_TIME, TRUE, fast_blink_led);
            }
            else if(led_state == 2)
            {
                led_state = 3;
            }

            
        }
        boardcast_info[PACK_SEQ_NUM] = led_state == 3;///////1 control mode , 0 idle mode
        boardcast_info[VOLPER_OFFSET] = 0x80;

        if(++cnt > 50)//1s
        {
            cnt = 0;
            add_crc16_to_boardcast_info();
            startAdvertising(BOARDCAST_INFO_LEN,boardcast_info,500);
            start_boardcast= 1;
        }

        if(boardcast_info[PACK_SEQ_NUM]==0 && start_boardcast == 1 )
        {
            LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);
            start_boardcast = 0;
        }
    }
}
static void bleDataTimer(timer_id tid);
void adc_read(void);

void adc_read(void)
{
    uint16 tmp,adc;
            tmp=AioRead(AIO0);
 	//adc = (tmp << 2)  - (tmp >> 1);
 	adc = tmp;//*100/28;
#define low_vol 1092 //3900
#define high_vol 1200//4071
#define min_vol 1036//3700

    if(adc >= high_vol)
    {
        PioSet(3, 1);
        PioSet(4, 1);

        if(led_state == 2)
        {
            led_state = 0;
        }

        //PioSet(9, 0);
    }
    else if(adc > 1100)
    {
        if(led_state == 2)
        {
            led_state = 0;
        }

    }
    else if(adc < low_vol)
    {
        PioSet(3, 0);
        PioSet(4, 0);

    }

    if(adc < min_vol)
    {
        if(led_state == 0 || led_state == 1)
        {
            led_state = 2;
            fast_blink =	TimerCreate(FAST_INTEVAL_TIME, TRUE, fast_blink_led);
        }
    }
}
static void bleDataTimer(timer_id tid)
{
    
    
    if( tid == ble_data_timeout_tid )
    {
        adc_read();


	    ble_data_timeout_tid =	TimerCreate(BTM_DATA_INTEVAL_TIME, TRUE, bleDataTimer);
    }
}

int fast_cnt=0;
void led_blink(void);
void led_blink(void)
{
        if(vol_flag)
        {
            vol_flag = 0;
            PioSet(9, 1);
        }
        else
        {
            vol_flag = 1;
            PioSet(9, 0);
        }
}
static void fast_blink_led(timer_id tid)
{
    
    if( tid == fast_blink)
    {
        switch(led_state)
        {
            case 0 : 
                PioSet(9, 1); //off led
                return;
            case 1 :
                PioSet(9, 0);//on led
                return;
            case 2 : //slow blink
            if(fast_cnt++ > 5)
            {
                fast_cnt = 0;
                led_blink();
            }
            break;
            case 3:
                led_blink();

                if(++fast_cnt > 100)
                {
                    fast_cnt = 0;
                
                    led_state = 0;

                    adc_read();
               }

                    if(up_down_state > 0)//control mode, give up exit, up down is running
                    {
                        fast_cnt = 0;
                    }

                break;
        }



        fast_blink =	TimerCreate(FAST_INTEVAL_TIME, TRUE, fast_blink_led);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This function is called after a power-on reset (including after a
 *      firmware panic) or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppInit(sleep_state last_sleep_state)
{
    /* set all PIOs to inputs and pull them down */
    PioSetModes(0xFFFFFFFFUL, pio_mode_user);
    PioSetDirs(0xFFFFFFFFUL, FALSE);
    PioSetPullModes(0xFFFFFFFFUL, pio_mode_strong_pull_down);
    
    /* disable wake up on UART RX */
    SleepWakeOnUartRX(FALSE);
    
    /* pull down the I2C lines */
    PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);
       
    /* Start advertising */
//    startAdvertising();
    add_id_to_boardcast_info();
    add_crc16_to_boardcast_info();
    

    TimerInit(MAX_APP_TIMERS, (void*)app_timers1);
    
    PioSetDir(3, TRUE/*PIO_DIRECTION_OUTPUT*/);
    PioSetMode(3, pio_mode_user);
	PioSet(3, 0);//WIFI NOT RUN

	PioSetDir(4, TRUE/*PIO_DIRECTION_OUTPUT*/);
	PioSetMode(4, pio_mode_user);
	PioSet(4, 0);//CHARGE
	
	PioSetDir(9, TRUE/*PIO_DIRECTION_OUTPUT*/);
	PioSetMode(9, pio_mode_user);
	PioSet(9, 1);//CHARGE

    /* Initialise UART and configure with
     * default baud rate and port configuration.
     */
    //DebugInit(UART_BUF_SIZE_BYTES_32, UartDataRxCallback, NULL);
    /* Initialise UART and configure with default baud rate and port
     * configuration
     */
    UartInit(uartRxDataCallback,
             uartTxDataCallback,
             rx_buffer, RX_BUFFER_SIZE,
             tx_buffer, TX_BUFFER_SIZE,
             uart_data_unpacked);

    UartConfig(0x0028,0);//9600
    //UartConfig(0x01d9,0);//115200
    UartEnable(TRUE);

    /* UART Rx threshold is set to 1,
     * so that every byte received will trigger the rx callback.
     */
    UartRead(1, 0);

    ble_data_timeout_tid =	TimerCreate(10 * MILLISECOND, TRUE, bleDataTimer);
    parse_uart_tid =	TimerCreate(PARSE_INTEVAL_TIME, TRUE, parse_uart_data);
    
    //UartWriteBlocking(boardcast_info,5);

    //startAdvertising(BOARDCAST_INFO_LEN,boardcast_info,500);
    //slow_blink =	TimerCreate(SLOW_INTEVAL_TIME, TRUE, slow_blink_led);
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppProcessSystemEvent(sys_event_id id, void *data)
{
    /* empty */
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

bool AppProcessLmEvent(lm_event_code event_code, 
                       LM_EVENT_T *p_event_data)
{
    return TRUE;
}
