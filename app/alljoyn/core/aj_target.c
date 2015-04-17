#include "aj_target.h"
#include "c_stdlib.h"
#include "c_math.h"
#include "os_type.h"
#include "alljoyn.h"
#include "aj_crypto.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "arch/sys_arch.h"

/*aj_crypto.h*/
static uint8_t seed[16];
static uint8_t key[16];

/*
 * The host has various ADC's. We are going to accumulate entropy by repeatedly
 * reading the ADC and accumulating the least significant bit or each reading.
 */
int GatherBits(uint8_t* buffer, uint32_t len)
{
    int i;
    uint32_t val;

    /*
     * Start accumulating entropy one bit at a time
     */
    for (i = 0; i < (8 * len); ++i) {
//        val = analogRead(analogPin);
        buffer[i / 8] ^= ((val & 1) << (i & 7));
    }
    return val;
}
void AJ_RandBytes(uint8_t* rand, uint32_t len)
{
   ///*
    // * On the first call we need to accumulate entropy
    // * for the seed and the key.
    // */
    //if (seed[0] == 0) {
    //    GatherBits(seed, sizeof(seed));
    //    GatherBits(key, sizeof(key));
    //}
    //AJ_AES_Enable(key);
    /*
     * This follows the NIST guidelines for using AES as a PRF
     */
    while (len) {
        *rand = os_random(256);
        len -= 1;
        rand += 1;
    }
    //AJ_AES_Disable();
}
void AJ_AES_CBC_128_ENCRYPT(const uint8_t* key, const uint8_t* in, uint8_t* out, uint32_t len, uint8_t* iv)
{
    AES_CTX *aes_ctx = (AES_CTX *)os_malloc(sizeof(AES_CTX));
    AES_set_key(aes_ctx, key, iv, AES_MODE_128);
    //if (is_decrypt)
    //{
    //    AES_convert_key(aes_ctx);
    //}
	AES_cbc_encrypt(aes_ctx, in, out, len);	
}
void AJ_AES_ECB_128_ENCRYPT(const uint8_t* key, const uint8_t* in, uint8_t* out)
{
	
}
void AJ_AES_Enable(const uint8_t* key)
{
	
}
void AJ_AES_Disable(void)
{
	
}
void AJ_AES_CTR_128(const uint8_t* key, const uint8_t* in, uint8_t* out, uint32_t len, uint8_t* ctr)
{
	
}

/*timer*/
AJ_Time AJ_ms_tick_timer;
os_timer_t ms_tick_timer;

void ICACHE_FLASH_ATTR ms_tick_timer_cb()
{
	AJ_ms_tick_timer.milliseconds ++;
	AJ_ms_tick_timer.seconds += AJ_ms_tick_timer.milliseconds / 1000;
	AJ_ms_tick_timer.milliseconds = AJ_ms_tick_timer.milliseconds % 1000;
}
void ms_tick_timer_start(void)
{
	os_timer_disarm(&ms_tick_timer);
	os_timer_setfn(&ms_tick_timer, (os_timer_func_t *)ms_tick_timer_cb, NULL);
	os_timer_arm(&ms_tick_timer,1,1);
}

/*aj_util.h*/
void AJ_InitTimer(AJ_Time* timer)
{
//    uint32_t now_msec = xTaskGetTickCount() / portTICK_RATE_MS;
//    uint32_t now_sec = now_msec / 1000;         //Get the seconds
//    now_msec = now_msec - (now_sec * 1000);     //Get the additional msec's
//    timer->seconds = now_sec;
//    timer->milliseconds = now_msec;		
    timer->seconds = ms_tick_timer.seconds;
    timer->milliseconds = ms_tick_timer.milliseconds;		
}
uint32_t AJ_GetElapsedTime(AJ_Time* timer, uint8_t cumulative)
{
	uint32 elapsed;
	AJ_Time now;
	AJ_InitTimer(&now);
	elapsed = ((now.seconds - timer->seconds) * 1000) + now.milliseconds - timer->milliseconds;
	return elapsed;
}
void AJ_Free(void* mem)
{
	os_free(mem);	 
}

void AJ_Sleep(uint32_t ms)
{
	sys_msleep(ms);
}
void* AJ_Malloc(size_t size)
{
	return (void *) os_malloc(size);
}
/*aj_wifi_ctrl.h*/
AJ_Status AJ_AcquireIPAddress(uint32_t* ip, uint32_t* mask, uint32_t* gateway, int32_t timeout)
{
	
	return AJ_OK;
}

/*aj_net.h*/
AJ_Status AJ_Net_Connect(AJ_NetSocket* netSock, uint16_t port, uint8_t addrType, const uint32_t* addr)
{
	return AJ_OK;
}

void AJ_Net_Disconnect(AJ_NetSocket* netSock)
{
	
}

AJ_Status AJ_Net_MCastUp(AJ_NetSocket* netSock)
{
	return AJ_OK;
}

void AJ_Net_MCastDown(AJ_NetSocket* netSock)
{
	
}

/*aj_target.h*/

/*
 * Message identifiers for the method calls this application implements
 */

#define APP_FLASH   AJ_APP_MESSAGE_ID(0, 0, 0)
#define APP_ON      AJ_APP_MESSAGE_ID(0, 0, 1)
#define APP_OFF     AJ_APP_MESSAGE_ID(0, 0, 2)

#define CONNECT_TIMEOUT    (1000 * 1000)
#define UNMARSHAL_TIMEOUT  (1000 * 5)

static const char ServiceName[] = "org.alljoyn.sample.ledservice";
static const char DaemonServiceName[] = "org.alljoyn.BusNode.Led";
static const uint16_t ServicePort = 24;


static const char* const testInterface[] = {
    "org.alljoyn.sample.ledcontroller",
    "?Flash msec<u",
    "?On",
    "?Off",
    NULL
};

static const AJ_InterfaceDescription testInterfaces[] = {
    testInterface,
    NULL
};

/**
 * Objects implemented by the application
 */
static const AJ_Object AppObjects[] = {
    { "/org/alljoyn/sample/ledcontroller", testInterfaces },
    { NULL }
};
static const char PWD[] = "ABCDEFGH";


static uint32_t PasswordCallback(uint8_t* buffer, uint32_t bufLen)
{
    memcpy(buffer, PWD, sizeof(PWD));
    return sizeof(PWD) - 1;
}

static void AppDoWork()
{
    /*
     * This function is called if there are no messages to unmarshal
     */
    AJ_Printf(("do work\n"));
}

static AJ_Status AppHandleFlash(AJ_Message* msg)
{
    AJ_Message reply;
    uint32_t timeout;
    AJ_UnmarshalArgs(msg, "u", &timeout);
    AJ_Printf(("AppHandleFlash(%u)\n", timeout));

 //   DUE_led_timed(timeout);


    AJ_MarshalReplyMsg(msg, &reply);
    return AJ_DeliverMsg(&reply);
}

static AJ_Status AppHandleOnOff(AJ_Message* msg, uint8_t on)
{
    AJ_Message reply;

//    AJ_Printf(("AppHandleOnOff(%u)\n", on));
//    DUE_led(on);

    AJ_MarshalReplyMsg(msg, &reply);
    return AJ_DeliverMsg(&reply);
}

int AJ_Main(void)
{
    AJ_Status status = AJ_OK;
    AJ_BusAttachment bus;
    uint8_t connected = FALSE;
    uint32_t sessionId = 0;

    /*
     * One time initialization before calling any other AllJoyn APIs
     */
    AJ_Initialize();
	ms_tick_timer_start();
//#ifndef NDEBUG
//    AJ_PrintXML(AppObjects);
//#endif	
    AJ_RegisterObjects(AppObjects, NULL);


    while (TRUE) {
        AJ_Message msg;

        if (!connected) {
            status = AJ_StartService(&bus, DaemonServiceName, CONNECT_TIMEOUT, FALSE, ServicePort, ServiceName, AJ_NAME_REQ_DO_NOT_QUEUE, NULL);
            if (status != AJ_OK) {
                continue;
            }
//            AJ_InfoPrintf(("StartService returned AJ_OK; running %s:%u\n", ServiceName, ServicePort));
            connected = TRUE;
            AJ_BusSetPasswordCallback(&bus, PasswordCallback);
        }

        status = AJ_UnmarshalMsg(&bus, &msg, UNMARSHAL_TIMEOUT);
        if (status != AJ_OK) {
            if (status == AJ_ERR_TIMEOUT) {
                AppDoWork();
                continue;
            }
        }
        if (status == AJ_OK) {
            switch (msg.msgId) {

            case AJ_METHOD_ACCEPT_SESSION:
                {
//                    AJ_InfoPrintf(("Accepting...\n"));
                    uint16_t port;
                    char* joiner;
                    AJ_UnmarshalArgs(&msg, "qus", &port, &sessionId, &joiner);
                    status = AJ_BusReplyAcceptSession(&msg, TRUE);

                    if (status == AJ_OK) {
//                        AJ_InfoPrintf(("Accepted session session_id=%u joiner=%s\n", sessionId, joiner));
                    } else {
//                        AJ_InfoPrintf(("AJ_BusReplyAcceptSession: error %d\n", status));
                    }
                }
                break;

            case APP_FLASH:
                status = AppHandleFlash(&msg);
                break;

            case APP_ON:
                AppHandleOnOff(&msg, TRUE);
                break;

            case APP_OFF:
                AppHandleOnOff(&msg, FALSE);
                break;

            case AJ_SIGNAL_SESSION_LOST_WITH_REASON:
                /*
                 * Force a disconnect
                 */
                status = AJ_ERR_READ;
                break;

            default:
                /*
                 * Pass to the built-in bus message handlers
                 */
                status = AJ_BusHandleBusMessage(&msg);
                break;
            }
        }
        /*
         * Unarshaled messages must be closed to free resources
         */
        AJ_CloseMsg(&msg);

        if (status == AJ_ERR_READ) {
//            AJ_AlwaysPrintf(("AllJoyn disconnect\n"));
            AJ_Disconnect(&bus);
            connected = FALSE;
            /*
             * Sleep a little while before trying to reconnect
             */
            AJ_Sleep(10 * 1000);
        }
    }
//    AJ_AlwaysPrintf(("svclite EXIT %d\n", status));

    return status;
}
