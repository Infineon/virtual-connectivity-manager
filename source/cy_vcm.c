/******************************************************************************
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 *
 *******************************************************************************
 * File Name:   cy_vcm.c
 *
 * Description: This file contains implementation for Virtual connectivity manager APIs.
 *
 ******************************************************************************/

#ifdef ENABLE_MULTICORE_CONN_MW
/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include <stdio.h>
#if defined (COMPONENT_MTB_HAL)
#include "mtb_ipc.h"
#else
#include "cyhal_ipc.h"
#endif
#include "cyabs_rtos.h"
#include "cy_worker_thread.h"
#include "cy_vcm.h"
#ifndef USE_VIRTUAL_API
#include "cy_wcm_internal.h"
#ifdef VCM_ENABLE_MQTT
#include "cy_mqtt_api_internal.h"
#endif  /* VCM_ENABLE_MQTT */
#endif  /* USE_VIRTUAL_API */
/*******************************************************************************
 * Macros
 ******************************************************************************/
 /* VCM log message */
#ifdef ENABLE_VCM_LOGS
#define cy_vcm_log_msg cy_log_msg
#else
#define cy_vcm_log_msg(a,b,c,...)
#endif

#ifndef CY_VCM_EVENT_QUEUE_SIZE
#define CY_VCM_EVENT_QUEUE_SIZE            (20UL)
#endif

#define ASYNC_CB_QUEUE_SIZE                (CY_VCM_EVENT_QUEUE_SIZE)   /* Size of ASYNC_CB queue */
#define REQUEST_QUEUE_SIZE                 (1UL)                       /* Size of Request queue */
#define RESPONSE_QUEUE_SIZE                (1UL)                       /* Size of Response queue */
#define INIT_QUEUE_SIZE                    (1UL)                       /* Size of Init queue */
#define DEINIT_QUEUE_SIZE                  (1UL)                       /* Size of De-Init queue */
#define DEINIT_RSP_QUEUE_SIZE              (1UL)                       /* Size of De-Init response queue */
#define FREE_QUEUE_SIZE                    (10UL)                      /* Size of free queue, used to send notifications to free a memory */
#define ASYNC_CB_QUEUE_NUM                 (1UL)        /* Async callback queue number */
#define REQUEST_QUEUE_NUM                  (2UL)        /* Request queue number */
#define RESPONSE_QUEUE_NUM                 (3UL)        /* Response queue number */
#define INIT_QUEUE_NUM                     (4UL)        /* Init queue number */
#define DEINIT_QUEUE_NUM                   (5UL)        /* De-init queue number */
#define DEINIT_RSP_QUEUE_NUM               (6UL)        /* De-init response queue number */
#define FREE_QUEUE_NUM                     (7UL)        /* free queue number */

#ifndef USE_VIRTUAL_API
#define MAX_WCM_DATA_EVENTS                (10UL)       /* Maximum WCM events that can be queued at a time */
#endif

#define CY_VCM_MAX_MUTEX_WAIT_TIME_MS      (120000)                          /* Maximum wait time for mutex operation in milliseconds */
#define CY_VCM_MAX_QUEUE_WAIT_TIME_US      (60000000)                        /* Maximum wait time for queue operation in microseconds */
#define CY_VCM_WORKER_THREAD_STACK_SIZE    (6 * 1024)                        /* VCM worker thread stack size */
#define CY_VCM_WORKER_THREAD_PRIORITY      (CY_RTOS_PRIORITY_ABOVENORMAL)    /* VCM worker thread priority */
#define VCM_UNUSED_PARAMETER(x)            ((void)(x))

#if defined(COMPONENT_PSE84)
#define IPC_PRIORITY                    1U
#define CM55_WAKEUP_IPC_CH_NUM          10
#define CM55_WAKEUP_IPC_INTR_NUM        3

#define CM55_WAKEUP_IPC_CH_MASK         CY_IPC_CH_MASK(CM55_WAKEUP_IPC_CH_NUM)
#define CM55_WAKEUP_IPC_INTR_MASK       CY_IPC_INTR_MASK(CM55_WAKEUP_IPC_INTR_NUM)
#define CM55_WAKEUP_IPC_INTR_MUX        CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM)

#define CM55_WAKEUP_IPC_CH_NUM2         11
#define CM55_WAKEUP_IPC_INTR_NUM2       4
#define CM55_WAKEUP_IPC_CH_MASK2        CY_IPC_CH_MASK(CM55_WAKEUP_IPC_CH_NUM2)
#define CM55_WAKEUP_IPC_INTR_MASK2      CY_IPC_INTR_MASK(CM55_WAKEUP_IPC_INTR_NUM2)
#define CM55_WAKEUP_IPC_INTR_MUX2       CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM2)
#endif
/*******************************************************************************
 * Global variables
 ******************************************************************************/
#ifndef USE_VIRTUAL_API
#if (CY_CPU_CORTEX_M0P)
CY_SECTION_SHAREDMEM
#elif defined(COMPONENT_PSE84)
#if (CY_CPU_CORTEX_M55)
CY_SECTION(".cy_gpu_buf")
#elif (CY_CPU_CORTEX_M33)
CY_SECTION(".cy_shared_socmem")
#endif
#endif
static cy_wcm_event_callback_params_t  wcm_event_callback_params[MAX_WCM_DATA_EVENTS]; /* Array which stores the parameters of cy_wcm_event_callback_t function each time the WCM event callback is received from WHD */

#if (CY_CPU_CORTEX_M0P)
CY_SECTION_SHAREDMEM
#elif defined(COMPONENT_PSE84)
#if (CY_CPU_CORTEX_M55)
CY_SECTION(".cy_gpu_buf")
#elif (CY_CPU_CORTEX_M33)
CY_SECTION(".cy_shared_socmem")
#endif
#endif
static cy_wcm_event_data_t           local_wcm_event_data[MAX_WCM_DATA_EVENTS];      /* Array which stores the event data received from WHD */

static cy_vcm_internal_callback_t    wcm_internal_func_ptr = NULL;     /* Store the WCM event callback function pointer */
static uint8_t                       wcm_event_index = 0;              /* Next available slot in wcm_event_callback_params to store WCM asynchronous data */
static cy_mutex_t                    vcm_wcm_mutex;                    /* WCM event mutex */

#ifdef VCM_ENABLE_MQTT
static cy_vcm_internal_callback_t    mqtt_internal_func_ptr = NULL;    /* Store the MQTT event callback function pointer */
static cy_mutex_t                    vcm_mqtt_mutex;                   /* MQTT event mutex */
#endif  /* VCM_ENABLE_MQTT */

#else  /* USE_VIRTUAL_API */
static cy_mutex_t                    vcm_async_mutex;                  /* Asynchronous callback mutex used only on the secondary core */
#endif

/* Queue objects */
#if defined(COMPONENT_PSE84)
mtb_ipc_queue_t async_cb_queue_obj, request_queue_obj, response_queue_obj, deinit_queue_obj, deinit_rsp_queue_obj, init_queue_obj, free_queue_obj;
#else
cyhal_ipc_t async_cb_queue_obj, request_queue_obj, response_queue_obj, deinit_queue_obj, deinit_rsp_queue_obj, init_queue_obj, free_queue_obj;
#endif

/* Queue pool pointers */
void *async_cb_queue_pool, * request_queue_pool, *response_queue_pool, *deinit_queue_pool, *deinit_rsp_queue_pool, *init_queue_pool, *free_queue_pool;

/* Queue handles */
#if defined(COMPONENT_PSE84)
mtb_ipc_queue_data_t* async_queue_handle, * request_queue_handle, *response_queue_handle, *deinit_queue_handle, *deinit_rsp_queue_handle, *init_queue_handle, *free_queue_handle;
#else
cyhal_ipc_queue_t* async_queue_handle, * request_queue_handle, *response_queue_handle, *deinit_queue_handle, *deinit_rsp_queue_handle, *init_queue_handle, *free_queue_handle;
#endif

static cy_vcm_hal_resource_opt_t   hal_resource_opt;             /* Should be set to CY_VCM_CREATE_HAL_RESOURCE in the core which initializes VCM first. Value should be set to CY_VCM_USE_HAL_RESOURCE in the core which initializes VCM later */
static bool                        is_vcm_initialized;           /* Global variable to indicate whether VCM in initialized or not */
static bool                        is_dual_core_vcm_initialized; /* Global variable to indicate whether VCM in initialized on both cores or not */
static bool                        is_vcm_init_complete;         /* Global variable to indicate whether CY_VCM_EVENT_INIT_COMPLETE event is received on main_app */
static cy_vcm_event_callback_t     event_cb = NULL;              /* Pointer to the VCM event callback function */
static cy_worker_thread_info_t     cy_vcm_worker_thread;         /* VCM worker thread */
static cy_mutex_t                  init_complete_mutex;          /* Mutex used to protect access to asynchronous init complete event queue */
static cy_mutex_t                  vcm_request_mutex;            /* Request mutex used on both cores while send and processing API requests */

/*******************************************************************************
* Structures
*******************************************************************************/
/**
 * Virtual-Connectivity-Manager asynchronous callback structure
 */
typedef struct
{
    void*     func_ptr;     /* Function pointer to the callback function. This will be provided from the core containing virtual implementations */
    void*     data;         /* Module specific structure cast to void* which contains all parameters needed for the module specific callback funtion */
} cy_vcm_cb_data;

/*******************************************************************************
 * Function definitions
 ******************************************************************************/

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
void wake_cm55_from_cm33_before_send_ipc(void)
{
    Cy_IPC_Drv_SetInterrupt(
            Cy_IPC_Drv_GetIntrBaseAddr(
                    CM55_WAKEUP_IPC_INTR_NUM),
            0, CM55_WAKEUP_IPC_CH_MASK );
}

void wake_cm55_from_cm33_after_send_ipc(void)
{
    Cy_IPC_Drv_SetInterrupt(
            Cy_IPC_Drv_GetIntrBaseAddr(
                    CM55_WAKEUP_IPC_INTR_NUM2),
            0, CM55_WAKEUP_IPC_CH_MASK2 );
}
#endif

#ifdef COMPONENT_CM55
const cy_stc_sysint_t ipcWakeIntConfig =
{
    .intrSrc = (IRQn_Type)CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM),
    .intrPriority = IPC_PRIORITY,
};

const cy_stc_sysint_t ipcWakeIntConfig2 =
{
    .intrSrc = (IRQn_Type)CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM2),
    .intrPriority = IPC_PRIORITY,
};

static void ipc_wake_isr(void)
{
    uint32_t shadowIntr;
    IPC_INTR_STRUCT_Type *ipcIntrPtr;

    ipcIntrPtr = Cy_IPC_Drv_GetIntrBaseAddr(CM55_WAKEUP_IPC_INTR_NUM);
    shadowIntr = Cy_IPC_Drv_GetInterruptStatusMasked(ipcIntrPtr);

    /* Check to make sure the interrupt was a notify interrupt */
    if (0UL != Cy_IPC_Drv_ExtractAcquireMask(shadowIntr))
    {
        /* Clear the notify interrupt.  */
        Cy_IPC_Drv_ClearInterrupt(ipcIntrPtr, CY_IPC_NO_NOTIFICATION, Cy_IPC_Drv_ExtractAcquireMask(shadowIntr));
    }
}

static void ipc_wake_isr2(void)
{
    uint32_t shadowIntr;
    IPC_INTR_STRUCT_Type *ipcIntrPtr;

    ipcIntrPtr = Cy_IPC_Drv_GetIntrBaseAddr(CM55_WAKEUP_IPC_INTR_NUM2);
    shadowIntr = Cy_IPC_Drv_GetInterruptStatusMasked(ipcIntrPtr);

    /* Check to make sure the interrupt was a notify interrupt */
    if (0UL != Cy_IPC_Drv_ExtractAcquireMask(shadowIntr))
    {
        /* Clear the notify interrupt.  */
        Cy_IPC_Drv_ClearInterrupt(ipcIntrPtr, CY_IPC_NO_NOTIFICATION, Cy_IPC_Drv_ExtractAcquireMask(shadowIntr));
    }
}

void setup_ws_cm55(void)
{
    Cy_SysInt_Init(&ipcWakeIntConfig, ipc_wake_isr);
    Cy_SysInt_Init(&ipcWakeIntConfig2, ipc_wake_isr2);

    Cy_IPC_Drv_SetInterruptMask(
            Cy_IPC_Drv_GetIntrBaseAddr(CM55_WAKEUP_IPC_INTR_NUM),
            CM55_WAKEUP_IPC_CH_MASK,
            CM55_WAKEUP_IPC_CH_MASK);

    NVIC_EnableIRQ((IRQn_Type)CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM));
    Cy_IPC_Drv_SetInterruptMask(
        Cy_IPC_Drv_GetIntrBaseAddr(CM55_WAKEUP_IPC_INTR_NUM2),
        CM55_WAKEUP_IPC_CH_MASK2,
        CM55_WAKEUP_IPC_CH_MASK2);

    NVIC_EnableIRQ((IRQn_Type)CY_IPC_INTR_MUX(CM55_WAKEUP_IPC_INTR_NUM2));
}
#endif

#endif /* COMPONENT_PSE84 */

#ifdef USE_VIRTUAL_API

/* Process asynchronous callback */
static void process_async_cb(void *arg)
{
    VCM_UNUSED_PARAMETER(arg);
    cy_vcm_internal_callback_t       event_cb;
    cy_vcm_cb_data                   async_cb_data;
    cy_rslt_t                        result;

    result = cy_rtos_get_mutex(&vcm_async_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }

    if((is_dual_core_vcm_initialized != true) || (is_vcm_initialized != true))
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: VCM initialization error\n", __func__, __LINE__);
        (void)cy_rtos_set_mutex(&vcm_async_mutex);
        return;
    }
#if defined(COMPONENT_PSE84)
    while(mtb_ipc_queue_count(&async_cb_queue_obj))
    {
        result = mtb_ipc_queue_get(&async_cb_queue_obj, &async_cb_data, 0);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: mtb_ipc_queue_get failed. res = 0x%x\n", __func__, __LINE__, result);
            continue;
        }
        event_cb = (cy_vcm_internal_callback_t)async_cb_data.func_ptr;

        if(event_cb != NULL)
        {
            /* Call the application callback with appropriate parameter */
            event_cb((void*)async_cb_data.data);
        }
    }
#else
    result = cyhal_ipc_queue_get(&async_cb_queue_obj, &async_cb_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cyhal_ipc_queue_get failed. res = 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&vcm_async_mutex);     
        return;
    }

    event_cb = (cy_vcm_internal_callback_t)async_cb_data.func_ptr;

    if(event_cb != NULL)
    {
        /* Call the application callback with appropriate parameter */
        event_cb((void*)async_cb_data.data);
    }

#endif
    result = cy_rtos_set_mutex(&vcm_async_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }
    return;
}

/* write event callback for async-callback-queue */
#if defined(COMPONENT_PSE84)
void async_queue_write_callback(void *callback_arg, mtb_ipc_queue_event_t event)
#else
void async_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
#endif
{
    cy_rslt_t result;
    result = cy_worker_thread_enqueue(&cy_vcm_worker_thread, process_async_cb, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_worker_thread_enqueue failed in async_queue_write_callback. res = 0x%x\n", __func__, __LINE__, result);
    }
    (void)result;
}

#else  /* !USE_VIRTUAL_API - Primary core with implementation will run the below code */

/* Process the request to free an address from free_queue */
static void process_virtual_free(void *arg)
{
    cy_rslt_t result;
    uint32_t ptr_val;

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res 0x%x\n", __func__, __LINE__, result);
        return;
    }
#if defined(COMPONENT_PSE84)
    while(mtb_ipc_queue_count(&free_queue_obj))
    {
        /* Read free queue to get the address to be freed */
        result = mtb_ipc_queue_get(&free_queue_obj, &ptr_val, 0);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to get the address to be freed from the queue. res = 0x%x\n", __func__, __LINE__, result);
            continue;
        }
        else
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_get Success. \n", __func__, __LINE__);
        }

        /* Call free on the address received from the queue */
        if((void*)ptr_val != NULL)
        {
            free((void*)(ptr_val));
        }
    }
#else
    /* Read free queue to get the address to be freed */
    result = cyhal_ipc_queue_get(&free_queue_obj, &ptr_val, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to get the address to be freed from the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }

    /* Call free on the address received from the queue */
    free((void*)(ptr_val));
#endif
    cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }
    return;
}

/* write event callback for free_queue */
#if defined(COMPONENT_PSE84)
void free_queue_write_callback(void *callback_arg, mtb_ipc_queue_event_t event)
#else
void free_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
#endif
{
    cy_rslt_t result;
    result = cy_worker_thread_enqueue(&cy_vcm_worker_thread, process_virtual_free, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_worker_thread_enqueue failed in request_queue_write_callback. res = 0x%x\n", __func__, __LINE__, result);
    }
    (void)result;
}

/* Local WCM callback event which posts the event data to the asynchronous callback queue */
void local_wcm_event_cb(cy_wcm_event_t event, cy_wcm_event_data_t *event_data)
{
    cy_vcm_cb_data   vcm_async_callback_data;
    cy_rslt_t        result;

    result = cy_rtos_get_mutex(&vcm_wcm_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }

    if((is_vcm_initialized == false) || (is_dual_core_vcm_initialized == false))
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] VCM initialization incomplete. res = 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
        return;
    }

    if(wcm_internal_func_ptr == NULL)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] WCM callback is not registered.\n", __func__, __LINE__);
        (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
        return;
    }

    if(event_data != NULL)
    {
        memset(&(local_wcm_event_data[wcm_event_index]), 0, sizeof(cy_wcm_event_data_t));
        memcpy((void*)(&(local_wcm_event_data[wcm_event_index])), (void*)event_data, sizeof(cy_wcm_event_data_t));
        wcm_event_callback_params[wcm_event_index].event_data = &(local_wcm_event_data[wcm_event_index]);
    }
    else
    {
        wcm_event_callback_params[wcm_event_index].event_data = NULL;
    }
    wcm_event_callback_params[wcm_event_index].event = event;
    vcm_async_callback_data.data = (void *)(&(wcm_event_callback_params[wcm_event_index]));
    vcm_async_callback_data.func_ptr = (void *)wcm_internal_func_ptr;

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else
    result = cyhal_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push WCM event to the asynchronous callback queue. res = 0x%x\n", __func__, __LINE__, result);
    }

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#endif

    if(wcm_event_index < (MAX_WCM_DATA_EVENTS - 1))
    {
        wcm_event_index++;
    }
    else
    {
        wcm_event_index = 0;
    }

    result = cy_rtos_set_mutex(&vcm_wcm_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    return;
}

#ifdef VCM_ENABLE_MQTT
/* Local MQTT callback event which posts the event data to the asynchronous callback queue */
void local_mqtt_event_cb(cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data)
{
#if (CY_CPU_CORTEX_M0P)
    CY_SECTION_SHAREDMEM
#elif defined(COMPONENT_PSE84)
#if (CY_CPU_CORTEX_M55)
CY_SECTION(".cy_gpu_buf")
#elif (CY_CPU_CORTEX_M33)
CY_SECTION(".cy_shared_socmem")
#endif
#endif
    static cy_mqtt_callback_params_t mqtt_callback_params;
    cy_vcm_cb_data                   vcm_async_callback_data;
    cy_rslt_t                        result;
    const char                       *payload;
    const char                       *topic;
#if defined(COMPONENT_PSE84)
    const char                       *payload_base_ptr, *topic_base_ptr;
    uint32_t                          aligned_size, alloc_size;
#endif

    result = cy_rtos_get_mutex(&vcm_mqtt_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }

    if((is_vcm_initialized == false) || (is_dual_core_vcm_initialized == false))
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] VCM initialization incomplete\n", __func__, __LINE__);
        (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }

    if(mqtt_internal_func_ptr == NULL)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] MQTT callback is not registered.\n", __func__, __LINE__);
        (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }

    memset(&mqtt_callback_params, 0x00, sizeof(cy_mqtt_callback_params_t));
    mqtt_callback_params.mqtt_handle = mqtt_handle;

    mqtt_callback_params.event = event;
    /* Store the topic and payload into a local memory before passing to the secondary core.
     * This memory will be freed in virtual_event_handler in MQTT library
     */
#if defined(COMPONENT_PSE84)
    /* Determine the minimum 32 byte aligned memory size that is needed to fit the payload_len */
    aligned_size = event.data.pub_msg.received_message.payload_len + (CY_VCM_MEMORY_BYTE_ALIGNMENT - (event.data.pub_msg.received_message.payload_len % CY_VCM_MEMORY_BYTE_ALIGNMENT));
    /* Add additional 32 bytes to facilitate moving of the start of payload pointer to a location which is 32 byte aligned */
    alloc_size = aligned_size + CY_VCM_MEMORY_BYTE_ALIGNMENT;
    payload_base_ptr = (char*)malloc(alloc_size);
    if( payload_base_ptr == NULL )
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] malloc failed. No memory.\n", __func__, __LINE__);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }
    memset((void*)payload_base_ptr, 0, alloc_size);
    /* Moving the payload pointer to a memory location which is 32 byte aligned */
    payload = (char*)((uint32_t)payload_base_ptr + ( CY_VCM_MEMORY_BYTE_ALIGNMENT - ((uint32_t)payload_base_ptr % CY_VCM_MEMORY_BYTE_ALIGNMENT)));
    memcpy((void*)payload, (void*)(event.data.pub_msg.received_message.payload), event.data.pub_msg.received_message.payload_len);
#if (CY_CPU_CORTEX_M55)
    /* Clean the D-cache so that data is written to the actual memory location */
    SCB_CleanDCache_by_Addr((void*)payload, aligned_size);
#endif

    /* Determine the minimum 32 byte aligned memory size that is needed to fit the topic_len */
    aligned_size = event.data.pub_msg.received_message.topic_len + (CY_VCM_MEMORY_BYTE_ALIGNMENT - (event.data.pub_msg.received_message.topic_len % CY_VCM_MEMORY_BYTE_ALIGNMENT));
    /* Add additional 32 bytes to facilitate moving of the start of topic pointer to a location which is 32 byte aligned */
    alloc_size = aligned_size + CY_VCM_MEMORY_BYTE_ALIGNMENT;
    topic_base_ptr = (char*)malloc( alloc_size );
    if( topic_base_ptr == NULL )
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] malloc failed. No memory.\n", __func__, __LINE__);
        free((char*)payload_base_ptr);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }
    memset((void*)topic_base_ptr, 0, alloc_size);
    /* Moving the topic pointer to a memory location which is 32 byte aligned */
    topic = (char*)((uint32_t)topic_base_ptr + ( CY_VCM_MEMORY_BYTE_ALIGNMENT - ((uint32_t)topic_base_ptr % CY_VCM_MEMORY_BYTE_ALIGNMENT)));
    memcpy((void*)topic, (void*)(event.data.pub_msg.received_message.topic), event.data.pub_msg.received_message.topic_len);
#if (CY_CPU_CORTEX_M55)
    /* Clean the D-cache so that data is written to the actual memory location */
    SCB_CleanDCache_by_Addr((void*)topic, aligned_size);
#endif
#else /* !COMPONENT_PSE84*/
    payload = (char *) malloc( event.data.pub_msg.received_message.payload_len );
    if( payload == NULL )
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] malloc failed. No memory.\n", __func__, __LINE__);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }
    topic = (char *) malloc( event.data.pub_msg.received_message.topic_len );
    if( topic == NULL )
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] malloc failed. No memory.\n", __func__, __LINE__);
        free((char*)payload);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        return;
    }
    memcpy((void*)payload, (void*)(event.data.pub_msg.received_message.payload), event.data.pub_msg.received_message.payload_len);
    memcpy((void*)topic, (void*)(event.data.pub_msg.received_message.topic), event.data.pub_msg.received_message.topic_len);
#endif /*COMPONENT_PSE84*/

    mqtt_callback_params.event.data.pub_msg.received_message.payload = payload;
    mqtt_callback_params.event.data.pub_msg.received_message.topic = topic;
#if defined(COMPONENT_PSE84)
    mqtt_callback_params.payload_base_ptr = (void*)payload_base_ptr;
    mqtt_callback_params.topic_base_ptr = (void*)topic_base_ptr;
#endif

    mqtt_callback_params.user_data = user_data;
    vcm_async_callback_data.data = (void *)(&(mqtt_callback_params));
    vcm_async_callback_data.func_ptr = (void *)mqtt_internal_func_ptr;

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /* !COMPONENT_PSE84*/
    result = cyhal_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /* COMPONENT_PSE84*/

    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push MQTT event to the asynchronous callback queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        goto exit;
    }

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#endif

    result = cy_rtos_set_mutex(&vcm_mqtt_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    return;

exit:
    free((char*)payload);
    free((char*)topic);
#if defined(COMPONENT_PSE84)
    free((char*)payload_base_ptr);
    free((char*)topic_base_ptr);
#endif
    return;
}
#endif /* VCM_ENABLE_MQTT */

/* Process API request events from request queue */
static void process_api_request(void *arg)
{
    cy_vcm_request_t   request;
    cy_vcm_response_t  response;
#if (CY_CPU_CORTEX_M0P)
    CY_SECTION_SHAREDMEM
#elif defined(COMPONENT_PSE84)
#if (CY_CPU_CORTEX_M55)
CY_SECTION(".cy_gpu_buf")
#elif (CY_CPU_CORTEX_M33)
CY_SECTION(".cy_shared_socmem")
#endif
#endif
    static cy_rslt_t   api_result = CY_RSLT_SUCCESS;
    cy_rslt_t result;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Request processing STARTS on primary application\n", __func__, __LINE__);

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }

    memset(&request, 0, sizeof(cy_vcm_request_t));
    /* Read the API request from request_queue
     * mtb_ipc_queue_get will also perform the cache operation whenever D-cache is available.
     * Hence request will contain the latest data and not stale data. */
#if defined(COMPONENT_PSE84)
    result = mtb_ipc_queue_get(&request_queue_obj, &request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else
    result = cyhal_ipc_queue_get(&request_queue_obj, &request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] IPC get queue failed. res = 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] API request received. API ID: %d\n", __func__, __LINE__, request.api_id);

    /* Switch case to identify the right API request */
    switch(request.api_id)
    {
        case CY_VCM_API_WCM_IS_CONNECTED_AP:
        {
#if (CY_CPU_CORTEX_M0P)
            CY_SECTION_SHAREDMEM
#elif defined(COMPONENT_PSE84)
#if (CY_CPU_CORTEX_M55)
CY_SECTION(".cy_gpu_buf")
#elif (CY_CPU_CORTEX_M33)
CY_SECTION(".cy_shared_socmem")
#endif
#endif
            static uint8_t connect_status = 0;
            connect_status = cy_wcm_is_connected_to_ap();
            response.result = (void*)&connect_status;
            break;
        }
        case CY_VCM_API_WCM_REG_EVENT_CB:
        {
            cy_wcm_register_event_callback_params_t *params = NULL;

            result = cy_rtos_get_mutex(&vcm_wcm_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
            if(result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
                api_result = CY_VCM_MUTEX_ERROR;
                response.result = &api_result;
                break;
            }

            params = (cy_wcm_register_event_callback_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
                break;
            }
            /* Store the WCM internal function pointer */
            wcm_internal_func_ptr = params->event_callback;

            api_result = cy_wcm_register_event_callback(&local_wcm_event_cb);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_wcm_register_event_callback failed\n", __func__, __LINE__);
                wcm_internal_func_ptr = NULL;
            }
            response.result = &api_result;
            (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
            break;
        }
        case CY_VCM_API_WCM_DEREG_EVENT_CB:
        {
            cy_wcm_deregister_event_callback_params_t *params = NULL;

            result = cy_rtos_get_mutex(&vcm_wcm_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
            if(result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
                api_result = CY_VCM_MUTEX_ERROR;
                response.result = &api_result;
                break;
            }

            params = (cy_wcm_deregister_event_callback_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
                break;
            }
            api_result = cy_wcm_deregister_event_callback(&local_wcm_event_cb);
            if((api_result == CY_RSLT_SUCCESS) && (params->event_callback == wcm_internal_func_ptr))
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_wcm_deregister_event_callback is a success\n", __func__, __LINE__);

                /* Clear the WCM internal function pointer */
                wcm_internal_func_ptr = NULL;
            }
            response.result = &api_result;
            (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
            break;
        }
#ifdef VCM_ENABLE_MQTT
        case CY_VCM_API_MQTT_SUBSCRIBE:
        {
            cy_mqtt_subscribe_params_t *params = (cy_mqtt_subscribe_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                break;
            }
#if defined(COMPONENT_CM55) && defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
            SCB_InvalidateDCache_by_Addr((void *) params->sub_info->topic, params->sub_info->topic_len);
#endif
            api_result = cy_mqtt_subscribe(params->mqtt_handle, params->sub_info, params->sub_count);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_subscribe failed\n", __func__, __LINE__);
            }
            response.result = &api_result;
            break;
        }
        case CY_VCM_API_MQTT_UNSUBSCRIBE:
        {
            cy_mqtt_unsubscribe_params_t *params = (cy_mqtt_unsubscribe_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                break;
            }
#if defined(COMPONENT_CM55) && defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
            SCB_InvalidateDCache_by_Addr((void *) params->unsub_info->topic, params->unsub_info->topic_len);
#endif
            api_result = cy_mqtt_unsubscribe(params->mqtt_handle, params->unsub_info, params->unsub_count);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_unsubscribe failed\n", __func__, __LINE__);
            }
            response.result = &api_result;
            break;
        }
        case CY_VCM_API_MQTT_PUBLISH:
        {
            cy_mqtt_publish_params_t *params = (cy_mqtt_publish_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                break;
            }

#if defined(COMPONENT_CM55) && defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
            SCB_InvalidateDCache_by_Addr((void *) params->pub_msg->topic, params->pub_msg->topic_len);
            SCB_InvalidateDCache_by_Addr((void *) params->pub_msg->payload, params->pub_msg->payload_len);
#endif
            api_result = cy_mqtt_publish(params->mqtt_handle, params->pub_msg);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_publish failed\n", __func__, __LINE__);
            }
            response.result = &api_result;
            break;
        }
        case CY_VCM_API_MQTT_GET_HANDLE:
        {
            cy_mqtt_get_handle_params_t *params = (cy_mqtt_get_handle_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                break;
            }
            api_result = cy_mqtt_get_handle(params->mqtt_handle, params->descriptor);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_get_handle failed\n", __func__, __LINE__);
            }
            response.result = &api_result;
            break;
        }
        case CY_VCM_API_MQTT_REG_EVENT_CB:
        {
            result = cy_rtos_get_mutex(&vcm_mqtt_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
            if(result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
                api_result = CY_VCM_MUTEX_ERROR;
                response.result = &api_result;
                break;
            }

            cy_mqtt_register_event_callback_params_t *params = (cy_mqtt_register_event_callback_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
                break;
            }
            /* Store the MQTT internal function pointer */
            mqtt_internal_func_ptr = params->event_callback;
            api_result = cy_mqtt_register_event_callback(params->mqtt_handle, &local_mqtt_event_cb, params->user_data );
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_register_event_callback failed\n", __func__, __LINE__);
                mqtt_internal_func_ptr = NULL;
            }
            response.result = &api_result;
            (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
            break;
        }
        case CY_VCM_API_MQTT_DEREG_EVENT_CB:
        {
            result = cy_rtos_get_mutex(&vcm_mqtt_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
            if(result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
                api_result = CY_VCM_MUTEX_ERROR;
                response.result = &api_result;
                break;
            }

            cy_mqtt_deregister_event_callback_params_t *params = (cy_mqtt_deregister_event_callback_params_t *)request.params;
            if(params == NULL)
            {
                api_result = CY_VCM_BAD_ARG;
                response.result = &api_result;
                (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
                break;
            }

            api_result = cy_mqtt_deregister_event_callback(params->mqtt_handle, &local_mqtt_event_cb);
            if(api_result != CY_RSLT_SUCCESS)
            {
                cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_mqtt_deregister_event_callback failed.\n", __func__, __LINE__);
            }
            else
            {
                mqtt_internal_func_ptr = NULL;
            }
            response.result = &api_result;
            (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
            break;
        }
#endif /* VCM_ENABLE_MQTT */
        default:
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] API ID did not match with available virtual APIs\n", __func__, __LINE__);
            response.result = NULL;
            break;
        }
    }

    /* Push the result of API call to response_queue
     * response.result always points to a shared memory.
     * Hence whenever D-cache is available on a core, cache operation is not require before sending it over IPC */
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&response_queue_obj, &response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#else /*!COMPONENT_PSE84*/
    result = cyhal_ipc_queue_put(&response_queue_obj, &response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }
#endif /*COMPONENT_PSE84*/
    result = cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] API request processed\n", __func__, __LINE__);
}

/* write event callback for request_queue */
#if defined(COMPONENT_PSE84)
void request_queue_write_callback(void *callback_arg, mtb_ipc_queue_event_t event)
#else
void request_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
#endif
{
    cy_rslt_t result;
    result = cy_worker_thread_enqueue(&cy_vcm_worker_thread, process_api_request, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_worker_thread_enqueue failed in request_queue_write_callback. res = 0x%x\n", __func__, __LINE__, result);
    }
    (void)result;
}
#endif  /* !USE_VIRTUAL_API */

/* Process init queue event */
static void init_queue_processing(void *arg)
{
    VCM_UNUSED_PARAMETER(arg);
    cy_rslt_t        result;
    cy_vcm_event_t   vcm_event;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] init checking on main_app STARTS\n", __func__, __LINE__);

    result = cy_rtos_get_mutex(&init_complete_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed\n", __func__, __LINE__);
        return;
    }

    if(is_vcm_initialized != true)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] VCM is not initialized\n", __func__, __LINE__);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return;
    }

    /* Wait for queue element to be posted by secondary core once the VCM is inited on secondary core */
#if defined(COMPONENT_PSE84)
    result = mtb_ipc_queue_get(&init_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else
    result = cyhal_ipc_queue_get(&init_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif
    if(result == CY_RSLT_SUCCESS)
    {
        if(vcm_event != CY_VCM_EVENT_INIT_COMPLETE)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Unexpected event received.Expected event CY_VCM_EVENT_INIT_COMPLETE\n", __func__, __LINE__);
            (void)cy_rtos_set_mutex(&init_complete_mutex);
            return;
        }
        /* Set a global flag */
        is_dual_core_vcm_initialized = true;

        /* Send intimation to the application */
        if(event_cb != NULL)
        {
            event_cb(CY_VCM_EVENT_INIT_COMPLETE);
        }
    }
    else
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] IPC queue get failed. res: 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return;
    }
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_enable_event(&init_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#else
    cyhal_ipc_queue_enable_event(&init_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#endif
    /* Delete the init_queue */
#if defined(COMPONENT_PSE84)
    cy_rtos_delay_milliseconds(50);
    mtb_ipc_queue_free(&init_queue_obj);
#else
    cyhal_ipc_queue_free(&init_queue_obj);
#endif
    init_queue_pool = NULL;
    init_queue_handle = NULL;

    is_vcm_init_complete = true;

    result = cy_rtos_set_mutex(&init_complete_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] init checking on main_app ENDS\n", __func__, __LINE__);
}

/* write event callback for init queue */
#if defined(COMPONENT_PSE84)
void init_queue_write_callback(void *callback_arg, mtb_ipc_queue_event_t event)
#else
void init_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
#endif
{
    cy_rslt_t result;
    result = cy_worker_thread_enqueue(&cy_vcm_worker_thread, init_queue_processing, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_worker_thread_enqueue failed in init_queue_write_callback. res = 0x%x\n", __func__, __LINE__, result);
    }
    (void)result;
}

/* Processing deinit queue */
static void deinit_queue_processing(void *arg)
{
    VCM_UNUSED_PARAMETER(arg);
    cy_vcm_event_t vcm_event;
    cy_rslt_t      result;

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }
#ifndef USE_VIRTUAL_API
    result = cy_rtos_get_mutex(&vcm_wcm_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }
#ifdef VCM_ENABLE_MQTT
    result = cy_rtos_get_mutex(&vcm_mqtt_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }
#endif
#else /* USE_VIRTUAL_API */
    result = cy_rtos_get_mutex(&vcm_async_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }
#endif

#if defined(COMPONENT_PSE84)
    result = mtb_ipc_queue_get(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else
    result = cyhal_ipc_queue_get(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] IPC queue get failed. res = 0x%x\n", __func__, __LINE__, result);
        goto exit;
    }

    /* if event received is CY_VCM_EVENT_DEINIT, inform the application that other core has de-initialized VCM */
    if(vcm_event == CY_VCM_EVENT_DEINIT)
    {
        if(event_cb != NULL)
        {
            event_cb(vcm_event);
        }
    }

    /* Clear is_dual_core_vcm_initialized as the other core has already de-initialized */
    is_dual_core_vcm_initialized = false;

    /* Acknowledge the de-init notification and send message to the core which is trying to de-initialize */
    vcm_event = CY_VCM_EVENT_DEINIT;
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#else /*!COMPONENT_PSE84*/
    result = cyhal_ipc_queue_put(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }
#endif
exit:
#ifndef USE_VIRTUAL_API
    (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
#ifdef VCM_ENABLE_MQTT
    (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
#endif
#else
    (void)cy_rtos_set_mutex(&vcm_async_mutex);
#endif
    (void)cy_rtos_set_mutex(&vcm_request_mutex);

    return;
}

/* write event callback for deinit queue */
#if defined(COMPONENT_PSE84)
void deinit_queue_write_callback(void *callback_arg, mtb_ipc_queue_event_t event)
#else
void deinit_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
#endif
{
    cy_rslt_t result;
    result = cy_worker_thread_enqueue(&cy_vcm_worker_thread, deinit_queue_processing, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d]: cy_worker_thread_enqueue failed. res = 0x%x\n", __func__, __LINE__, result);
    }
    (void)result;
}

cy_rslt_t cy_vcm_init(cy_vcm_config_t *config)
{
    cy_rslt_t                 result = CY_RSLT_SUCCESS;
    uint32_t                  channel;
#if defined(COMPONENT_PSE84)
    mtb_ipc_t                 *ipc_obj;
#endif
    cy_vcm_event_t            vcm_event;
    cy_worker_thread_params_t worker_thread_params;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager initialization STARTS\n", __func__, __LINE__);

    if(is_vcm_initialized)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager is initialized\n", __func__, __LINE__);
        return CY_RSLT_SUCCESS;
    }

    if(config == NULL)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Bad argument to cy_vcm_init\n", __func__, __LINE__);
        return CY_VCM_BAD_ARG;
    }

    if(config->event_cb == NULL)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] VCM event callback is NULL\n", __func__, __LINE__);
        return CY_VCM_BAD_ARG;
    }

    /* Store the Parameters passed to VCM init */
    hal_resource_opt = config->hal_resource_opt;
    channel          = config->channel_num;
    event_cb         = config->event_cb;
#if defined(COMPONENT_PSE84)
    ipc_obj          = config->ipc_obj;
#endif

    /* Creating the VCM worker thread */
    memset(&worker_thread_params, 0, sizeof(worker_thread_params));
    worker_thread_params.name = "VCM_worker";
    worker_thread_params.priority = CY_VCM_WORKER_THREAD_PRIORITY;
    worker_thread_params.stack = NULL;
    worker_thread_params.stack_size = CY_VCM_WORKER_THREAD_STACK_SIZE;
    worker_thread_params.num_entries = 0;
    if(cy_worker_thread_create(&cy_vcm_worker_thread, &worker_thread_params) != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to create the VCM worker thread\n", __func__, __LINE__);
        return CY_VCM_WORKER_THREAD_ERROR;
    }

    result = cy_rtos_init_mutex(&init_complete_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_init_mutex failed\n", __func__, __LINE__);
        result = CY_VCM_MUTEX_ERROR;
        goto exit1;
    }

    if (cy_rtos_init_mutex(&vcm_request_mutex) != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_init_mutex failed\n", __func__, __LINE__);
        result = CY_VCM_MUTEX_ERROR;
        goto exit2;
    }
#ifndef USE_VIRTUAL_API
    if (cy_rtos_init_mutex(&vcm_wcm_mutex) != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_init_mutex failed\n", __func__, __LINE__);
        result = CY_VCM_MUTEX_ERROR;
        goto exit3;
    }
#ifdef VCM_ENABLE_MQTT
    if (cy_rtos_init_mutex(&vcm_mqtt_mutex) != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_init_mutex failed\n", __func__, __LINE__);
        (void)cy_rtos_deinit_mutex(&vcm_wcm_mutex);
        result = CY_VCM_MUTEX_ERROR;
        goto exit3;
    }
#endif
#else  /* USE_VIRTUAL_API */
    if (cy_rtos_init_mutex(&vcm_async_mutex) != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_init_mutex failed\n", __func__, __LINE__);
        result = CY_VCM_MUTEX_ERROR;
        goto exit3;
    }
#endif

    /* Initialize the queue resources only in the core which initializes the Virtual connectivity manager first */
    if(hal_resource_opt == CY_VCM_CREATE_HAL_RESOURCE)
    {
#if defined(COMPONENT_PSE84)

        MTB_IPC_QUEUE_POOL_ALLOC(async_cb_queue_pool, ASYNC_CB_QUEUE_SIZE, sizeof(cy_vcm_cb_data));
        MTB_IPC_QUEUE_POOL_ALLOC(request_queue_pool, REQUEST_QUEUE_SIZE, sizeof(cy_vcm_request_t));
        MTB_IPC_QUEUE_POOL_ALLOC(response_queue_pool, RESPONSE_QUEUE_SIZE, sizeof(cy_vcm_response_t));
        MTB_IPC_QUEUE_POOL_ALLOC(deinit_queue_pool, DEINIT_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        MTB_IPC_QUEUE_POOL_ALLOC(deinit_rsp_queue_pool, DEINIT_RSP_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        MTB_IPC_QUEUE_POOL_ALLOC(init_queue_pool, INIT_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        MTB_IPC_QUEUE_POOL_ALLOC(free_queue_pool, FREE_QUEUE_SIZE, sizeof(void*));

        /* Defining and allocating (shared) memory for queue handles */
        MTB_IPC_QUEUE_DATA_ALLOC(async_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(request_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(response_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(deinit_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(deinit_rsp_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(init_queue_handle);
        MTB_IPC_QUEUE_DATA_ALLOC(free_queue_handle);

        /* Associating the queue handles with appropriate queue details */
        mtb_ipc_queue_config_t async_queue_config, request_queue_config, response_queue_config, deinit_queue_config, deinit_rsp_queue_config, init_queue_config, free_queue_config;
        async_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        async_queue_config.queue_num   = ASYNC_CB_QUEUE_NUM;
        async_queue_config.queue_pool  = async_cb_queue_pool;
        async_queue_config.max_num_items   = ASYNC_CB_QUEUE_SIZE;
        async_queue_config.item_size   = sizeof(cy_vcm_cb_data);
        async_queue_config.semaphore_num = 9UL;

        request_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        request_queue_config.queue_num   = REQUEST_QUEUE_NUM;
        request_queue_config.queue_pool  = request_queue_pool;
        request_queue_config.max_num_items   = REQUEST_QUEUE_SIZE;
        request_queue_config.item_size   = sizeof(cy_vcm_request_t);
        request_queue_config.semaphore_num = 10UL;

        response_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        response_queue_config.queue_num   = RESPONSE_QUEUE_NUM;
        response_queue_config.queue_pool  = response_queue_pool;
        response_queue_config.max_num_items   = RESPONSE_QUEUE_SIZE;
        response_queue_config.item_size   = sizeof(cy_vcm_response_t);
        response_queue_config.semaphore_num = 11UL;

        deinit_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        deinit_queue_config.queue_num = DEINIT_QUEUE_NUM;
        deinit_queue_config.queue_pool = deinit_queue_pool;
        deinit_queue_config.max_num_items = DEINIT_QUEUE_SIZE;
        deinit_queue_config.item_size = sizeof(cy_vcm_event_t);
        deinit_queue_config.semaphore_num = 12UL;

        deinit_rsp_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        deinit_rsp_queue_config.queue_num = DEINIT_RSP_QUEUE_NUM;
        deinit_rsp_queue_config.queue_pool = deinit_rsp_queue_pool;
        deinit_rsp_queue_config.max_num_items = DEINIT_RSP_QUEUE_SIZE;
        deinit_rsp_queue_config.item_size = sizeof(cy_vcm_event_t);
        deinit_rsp_queue_config.semaphore_num = 13UL;

        init_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        init_queue_config.queue_num = INIT_QUEUE_NUM;
        init_queue_config.queue_pool = init_queue_pool;
        init_queue_config.max_num_items = INIT_QUEUE_SIZE;
        init_queue_config.item_size = sizeof(cy_vcm_event_t);
        init_queue_config.semaphore_num = 14UL;

        free_queue_config.channel_num = (mtb_ipc_channel_t)channel;
        free_queue_config.queue_num = FREE_QUEUE_NUM;
        free_queue_config.queue_pool = free_queue_pool;
        free_queue_config.max_num_items = FREE_QUEUE_SIZE;
        free_queue_config.item_size = sizeof(void*);
        free_queue_config.semaphore_num = 15UL;

        /* Init queue */
        result = mtb_ipc_queue_init(ipc_obj, &init_queue_obj, init_queue_handle, &init_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit4;
        }
        /* Request queue */
        result = mtb_ipc_queue_init(ipc_obj, &request_queue_obj, request_queue_handle, &request_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit5;
        }
        /* Response queue */
        result = mtb_ipc_queue_init(ipc_obj, &response_queue_obj, response_queue_handle, &response_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit6;
        }
        /* Asynchronous callback queue */
        result = mtb_ipc_queue_init(ipc_obj, &async_cb_queue_obj, async_queue_handle, &async_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit7;
        }
        /* Deinit queue */
        result = mtb_ipc_queue_init(ipc_obj, &deinit_queue_obj, deinit_queue_handle, &deinit_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit8;
        }
        /* De-init response queue */
        result = mtb_ipc_queue_init(ipc_obj, &deinit_rsp_queue_obj, deinit_rsp_queue_handle, &deinit_rsp_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit9;
        }

        /* Queue used to pass address from secondary core to primary core which is to be freed; free_queue */
        result = mtb_ipc_queue_init(ipc_obj, &free_queue_obj, free_queue_handle, &free_queue_config);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit10;
        }

        /* Register callback for write operation in init queue */
        mtb_ipc_queue_register_callback(&init_queue_obj, init_queue_write_callback, NULL);
        mtb_ipc_queue_enable_event(&init_queue_obj, MTB_IPC_QUEUE_WRITE, true);
#else /*COMPONENT_PSE84*/
        CYHAL_IPC_QUEUE_POOL_ALLOC(async_cb_queue_pool, ASYNC_CB_QUEUE_SIZE, sizeof(cy_vcm_cb_data));
        CYHAL_IPC_QUEUE_POOL_ALLOC(request_queue_pool, REQUEST_QUEUE_SIZE, sizeof(cy_vcm_request_t));
        CYHAL_IPC_QUEUE_POOL_ALLOC(response_queue_pool, RESPONSE_QUEUE_SIZE, sizeof(cy_vcm_response_t));
        CYHAL_IPC_QUEUE_POOL_ALLOC(deinit_queue_pool, DEINIT_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        CYHAL_IPC_QUEUE_POOL_ALLOC(deinit_rsp_queue_pool, DEINIT_RSP_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        CYHAL_IPC_QUEUE_POOL_ALLOC(init_queue_pool, INIT_QUEUE_SIZE, sizeof(cy_vcm_event_t));
        CYHAL_IPC_QUEUE_POOL_ALLOC(free_queue_pool, FREE_QUEUE_SIZE, sizeof(void*));

        /* Defining and allocating (shared) memory for queue handles */
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(async_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(request_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(response_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(deinit_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(deinit_rsp_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(init_queue_handle);
        CYHAL_IPC_QUEUE_HANDLE_ALLOC(free_queue_handle);

        /* Associating the queue handles with appropriate queue details */
        async_queue_handle->channel_num = channel;
        async_queue_handle->queue_num   = ASYNC_CB_QUEUE_NUM;
        async_queue_handle->queue_pool  = async_cb_queue_pool;
        async_queue_handle->num_items   = ASYNC_CB_QUEUE_SIZE;
        async_queue_handle->item_size   = sizeof(cy_vcm_cb_data);

        request_queue_handle->channel_num = channel;
        request_queue_handle->queue_num   = REQUEST_QUEUE_NUM;
        request_queue_handle->queue_pool  = request_queue_pool;
        request_queue_handle->num_items   = REQUEST_QUEUE_SIZE;
        request_queue_handle->item_size   = sizeof(cy_vcm_request_t);

        response_queue_handle->channel_num = channel;
        response_queue_handle->queue_num   = RESPONSE_QUEUE_NUM;
        response_queue_handle->queue_pool  = response_queue_pool;
        response_queue_handle->num_items   = RESPONSE_QUEUE_SIZE;
        response_queue_handle->item_size   = sizeof(cy_vcm_response_t);

        deinit_queue_handle->channel_num = channel;
        deinit_queue_handle->queue_num = DEINIT_QUEUE_NUM;
        deinit_queue_handle->queue_pool = deinit_queue_pool;
        deinit_queue_handle->num_items = DEINIT_QUEUE_SIZE;
        deinit_queue_handle->item_size = sizeof(cy_vcm_event_t);

        deinit_rsp_queue_handle->channel_num = channel;
        deinit_rsp_queue_handle->queue_num = DEINIT_RSP_QUEUE_NUM;
        deinit_rsp_queue_handle->queue_pool = deinit_rsp_queue_pool;
        deinit_rsp_queue_handle->num_items = DEINIT_RSP_QUEUE_SIZE;
        deinit_rsp_queue_handle->item_size = sizeof(cy_vcm_event_t);

        init_queue_handle->channel_num = channel;
        init_queue_handle->queue_num = INIT_QUEUE_NUM;
        init_queue_handle->queue_pool = init_queue_pool;
        init_queue_handle->num_items = INIT_QUEUE_SIZE;
        init_queue_handle->item_size = sizeof(cy_vcm_event_t);

        free_queue_handle->channel_num = channel;
        free_queue_handle->queue_num = FREE_QUEUE_NUM;
        free_queue_handle->queue_pool = free_queue_pool;
        free_queue_handle->num_items = FREE_QUEUE_SIZE;
        free_queue_handle->item_size = sizeof(void*);

        /* Init queue */
        result = cyhal_ipc_queue_init(&init_queue_obj, init_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit4;
        }
        /* Request queue */
        result = cyhal_ipc_queue_init(&request_queue_obj, request_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit5;
        }
        /* Response queue */
        result = cyhal_ipc_queue_init(&response_queue_obj, response_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit6;
        }
        /* Asynchronous callback queue */
        result = cyhal_ipc_queue_init(&async_cb_queue_obj, async_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit7;
        }
        /* Deinit queue */
        result = cyhal_ipc_queue_init(&deinit_queue_obj, deinit_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit8;
        }
        /* De-init response queue */
        result = cyhal_ipc_queue_init(&deinit_rsp_queue_obj, deinit_rsp_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit9;
        }

        /* Queue used to pass address from secondary core to primary core which is to be freed; free_queue */
        result = cyhal_ipc_queue_init(&free_queue_obj, free_queue_handle);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_init failed for queue. res = 0x%x\n", __func__, __LINE__, result);
            goto exit10;
        }

        /* Register callback for write operation in init queue */
        cyhal_ipc_queue_register_callback(&init_queue_obj, init_queue_write_callback, NULL);
        cyhal_ipc_queue_enable_event(&init_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, true);
#endif /*COMPONENT_PSE84*/
    }
    else /* hal_resource_opt is CY_VCM_USE_HAL_RESOURCE */
    {
#if defined(COMPONENT_PSE84)

        /* Get handles of all the queues */
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] VCM initialization STARTS in the second core\n", __func__, __LINE__);
        mtb_ipc_queue_get_handle(ipc_obj, &init_queue_obj, channel, INIT_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &async_cb_queue_obj, channel, ASYNC_CB_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &request_queue_obj, channel, REQUEST_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &response_queue_obj, channel, RESPONSE_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &deinit_queue_obj, channel, DEINIT_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &deinit_rsp_queue_obj, channel, DEINIT_RSP_QUEUE_NUM);
        mtb_ipc_queue_get_handle(ipc_obj, &free_queue_obj, channel, FREE_QUEUE_NUM);
#else /* !COMPONENT_PSE84*/
        /* Get handles of all the queues */
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] VCM initialization STARTS in the second core\n", __func__, __LINE__);
        cyhal_ipc_queue_get_handle(&init_queue_obj, channel, INIT_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&async_cb_queue_obj, channel, ASYNC_CB_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&request_queue_obj, channel, REQUEST_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&response_queue_obj, channel, RESPONSE_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&deinit_queue_obj, channel, DEINIT_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&deinit_rsp_queue_obj, channel, DEINIT_RSP_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&free_queue_obj, channel, FREE_QUEUE_NUM);
#endif /* COMPONENT_PSE84*/
    }

#if defined(COMPONENT_PSE84)
    /* Register callback for write operation in deinit queue */
    mtb_ipc_queue_register_callback(&deinit_queue_obj, deinit_queue_write_callback, NULL);
    mtb_ipc_queue_enable_event(&deinit_queue_obj, MTB_IPC_QUEUE_WRITE, true);

#ifdef USE_VIRTUAL_API
    /* Register callback for write operation in async callback handling queue */
    mtb_ipc_queue_register_callback(&async_cb_queue_obj, async_queue_write_callback, NULL);
    mtb_ipc_queue_enable_event(&async_cb_queue_obj, MTB_IPC_QUEUE_WRITE, true);
#else /* !USE_VIRTUAL_API */
    /* Register callback for write operation in request handling queue */
    mtb_ipc_queue_register_callback(&request_queue_obj, request_queue_write_callback, NULL);
    mtb_ipc_queue_enable_event(&request_queue_obj, MTB_IPC_QUEUE_WRITE, true);

    /* Register callback for write operation in free queue */
    mtb_ipc_queue_register_callback(&free_queue_obj, free_queue_write_callback, NULL);
    mtb_ipc_queue_enable_event(&free_queue_obj, MTB_IPC_QUEUE_WRITE, true);
#endif /* USE_VIRTUAL_API */
#else /* !COMPONENT_PSE84*/
    /* Register callback for write operation in deinit queue */
    cyhal_ipc_queue_register_callback(&deinit_queue_obj, deinit_queue_write_callback, NULL);
    cyhal_ipc_queue_enable_event(&deinit_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, true);

#ifdef USE_VIRTUAL_API
    /* Register callback for write operation in async callback handling queue */
    cyhal_ipc_queue_register_callback(&async_cb_queue_obj, async_queue_write_callback, NULL);
    cyhal_ipc_queue_enable_event(&async_cb_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, true);

#else /* !USE_VIRTUAL_API */

    /* Register callback for write operation in request handling queue */
    cyhal_ipc_queue_register_callback(&request_queue_obj, request_queue_write_callback, NULL);
    cyhal_ipc_queue_enable_event(&request_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, true);

    /* Register callback for write operation in free queue */
    cyhal_ipc_queue_register_callback(&free_queue_obj, free_queue_write_callback, NULL);
    cyhal_ipc_queue_enable_event(&free_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, true);

#endif
#endif /*COMPONENT_PSE84*/
    if(hal_resource_opt == CY_VCM_USE_HAL_RESOURCE)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Initialization complete in second core. Informing the main core\n", __func__, __LINE__);
        vcm_event = CY_VCM_EVENT_INIT_COMPLETE;
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
        result = mtb_ipc_queue_put(&init_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] mtb_ipc_queue_put failed. Failed to intimate the main core. res = 0x%x\n", __func__, __LINE__, result);
#ifdef USE_VIRTUAL_API
            mtb_ipc_queue_enable_event(&async_cb_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#else
            mtb_ipc_queue_enable_event(&request_queue_obj, MTB_IPC_QUEUE_WRITE, false);
            mtb_ipc_queue_enable_event(&free_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#endif
            mtb_ipc_queue_enable_event(&deinit_queue_obj, MTB_IPC_QUEUE_WRITE, false);

            goto exit4;
        }
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#else /* !COMPONENT_PSE84*/
        result = cyhal_ipc_queue_put(&init_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
        if(result != CY_RSLT_SUCCESS)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_put failed. Failed to intimate the main core. res = 0x%x\n", __func__, __LINE__, result);
#ifdef USE_VIRTUAL_API
            cyhal_ipc_queue_enable_event(&async_cb_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#else
            cyhal_ipc_queue_enable_event(&request_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
            cyhal_ipc_queue_enable_event(&free_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#endif
            cyhal_ipc_queue_enable_event(&deinit_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);

            goto exit4;
        }
#endif /*COMPONENT_PSE84*/
        is_dual_core_vcm_initialized = true;
    }
    is_vcm_initialized = true;

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM55
    setup_ws_cm55();
#endif
#endif
    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager initialization ENDS - successfully Initialized\n", __func__, __LINE__);

    return result;

exit10:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&deinit_rsp_queue_obj);
#else
    cyhal_ipc_queue_free(&deinit_rsp_queue_obj);
#endif
    deinit_rsp_queue_pool = NULL;
    deinit_rsp_queue_handle = NULL;
exit9:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&deinit_queue_obj);
#else
    cyhal_ipc_queue_free(&deinit_queue_obj);
#endif
    deinit_queue_pool = NULL;
    deinit_queue_handle = NULL;
exit8:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&async_cb_queue_obj);
#else
    cyhal_ipc_queue_free(&async_cb_queue_obj);
#endif
    async_cb_queue_pool = NULL;
    async_queue_handle = NULL;
exit7:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&response_queue_obj);
#else
    cyhal_ipc_queue_free(&response_queue_obj);
#endif
    response_queue_pool = NULL;
    response_queue_handle = NULL;
exit6:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&request_queue_obj);
#else
    cyhal_ipc_queue_free(&request_queue_obj);
#endif
    request_queue_pool = NULL;
    request_queue_handle = NULL;
exit5:
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_free(&init_queue_obj);
#else
    cyhal_ipc_queue_free(&init_queue_obj);
#endif
    init_queue_pool = NULL;
    init_queue_handle = NULL;

exit4:
#ifndef USE_VIRTUAL_API
    cy_rtos_deinit_mutex(&vcm_wcm_mutex);
#ifdef VCM_ENABLE_MQTT
    cy_rtos_deinit_mutex(&vcm_mqtt_mutex);
#endif
#else
    cy_rtos_deinit_mutex(&vcm_async_mutex);
#endif

exit3:
    cy_rtos_deinit_mutex(&vcm_request_mutex);
exit2:
    cy_rtos_deinit_mutex(&init_complete_mutex);

exit1:
    cy_worker_thread_delete(&cy_vcm_worker_thread);

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager initialization ENDS - Initialization failed\n", __func__, __LINE__);

    return result;
}

cy_rslt_t cy_vcm_deinit()
{
    cy_rslt_t      result;
    cy_vcm_event_t vcm_event;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager De-initialization STARTS\n", __func__, __LINE__);

    if(!is_vcm_initialized)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager not initialized\n", __func__, __LINE__);
        return CY_VCM_NOT_INITIALIZED;
    }

    result = cy_rtos_get_mutex(&init_complete_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return CY_VCM_MUTEX_ERROR;
    }

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return CY_VCM_MUTEX_ERROR;
    }
#ifndef USE_VIRTUAL_API
    result = cy_rtos_get_mutex(&vcm_wcm_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return CY_VCM_MUTEX_ERROR;
    }
#ifdef VCM_ENABLE_MQTT
    result = cy_rtos_get_mutex(&vcm_mqtt_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return CY_VCM_MUTEX_ERROR;
    }
#endif
#else /* USE_VIRTUAL_API */
    result = cy_rtos_get_mutex(&vcm_async_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        (void)cy_rtos_set_mutex(&vcm_request_mutex);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return CY_VCM_MUTEX_ERROR;
    }
#endif

#ifdef USE_VIRTUAL_API
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_enable_event(&async_cb_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#else /*!COMPONENT_PSE84*/
    cyhal_ipc_queue_enable_event(&async_cb_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#endif /*COMPONENT_PSE84*/
#else
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_enable_event(&request_queue_obj, MTB_IPC_QUEUE_WRITE, false);
    mtb_ipc_queue_enable_event(&free_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#else /* !COMPONENT_PSE84*/
    cyhal_ipc_queue_enable_event(&request_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
    cyhal_ipc_queue_enable_event(&free_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#endif /*COMPONENT_PSE84*/
    wcm_internal_func_ptr = NULL;
#ifdef VCM_ENABLE_MQTT
    mqtt_internal_func_ptr = NULL;
#endif
#endif

    /* De-register the callback for write operation on deinit handling queue */
#if defined(COMPONENT_PSE84)
    mtb_ipc_queue_enable_event(&deinit_queue_obj, MTB_IPC_QUEUE_WRITE, false);
#else /*!COMPONENT_PSE84*/
    cyhal_ipc_queue_enable_event(&deinit_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#endif /*COMPONENT_PSE84*/

    if(hal_resource_opt == CY_VCM_CREATE_HAL_RESOURCE)
    {
        if(is_dual_core_vcm_initialized == true)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Informing the other core to de-initialize Virtual Connectivity Manager\n", __func__, __LINE__);

            /* Intimate the other core to de-initialize the Virtual Connectivity Manager by passing CY_VCM_EVENT_DEINIT through the de-init queue */
            vcm_event = CY_VCM_EVENT_DEINIT;
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
            result = mtb_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
            /* Wait for response from the other core on de-init_response_queue */
            result = mtb_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /* !COMPONENT_PSE84*/
            result = cyhal_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);

            /* Wait for response from the other core on de-init_response_queue */
            result = cyhal_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /*COMPONENT_PSE84*/
            CY_VCM_ERROR_CHECK(result);

            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Other core  has acknowledged de-initialization of Virtual Connectivity Manager\n", __func__, __LINE__);
        }
        else
        {
            /* Delete the init_queue resources if not already removed */
            if(is_vcm_init_complete == false)
            {
#if defined(COMPONENT_PSE84)
                mtb_ipc_queue_free(&init_queue_obj);
#else
                cyhal_ipc_queue_free(&init_queue_obj);
#endif
                init_queue_pool = NULL;
                init_queue_handle = NULL;
            }
        }

        /* Free all queues. Set the queue_handles and queue_pool pointers to NULL */
#if defined(COMPONENT_PSE84)
        mtb_ipc_queue_free(&free_queue_obj);
        mtb_ipc_queue_free(&deinit_rsp_queue_obj);
        mtb_ipc_queue_free(&deinit_queue_obj);
        mtb_ipc_queue_free(&async_cb_queue_obj);
        mtb_ipc_queue_free(&response_queue_obj);
        mtb_ipc_queue_free(&request_queue_obj);
#else
        cyhal_ipc_queue_free(&free_queue_obj);
        cyhal_ipc_queue_free(&deinit_rsp_queue_obj);
        cyhal_ipc_queue_free(&deinit_queue_obj);
        cyhal_ipc_queue_free(&async_cb_queue_obj);
        cyhal_ipc_queue_free(&response_queue_obj);
        cyhal_ipc_queue_free(&request_queue_obj);
#endif
        free_queue_pool = NULL;
        deinit_rsp_queue_pool = NULL;
        deinit_queue_pool = NULL;
        async_cb_queue_pool = NULL;
        response_queue_pool = NULL;
        request_queue_pool = NULL;

        free_queue_handle = NULL;
        deinit_rsp_queue_handle = NULL;
        deinit_queue_handle = NULL;
        async_queue_handle = NULL;
        response_queue_handle = NULL;
        request_queue_handle = NULL;
    }
    else /* hal_resource_opt = CY_VCM_USE_HAL_RESOURCE */
    {
        /* Intimate the other core to de-initialize the Virtual Connectivity Manager by passing CY_VCM_EVENT_DEINIT through the de-init queue */
        if(is_dual_core_vcm_initialized == true)
        {
            vcm_event = CY_VCM_EVENT_DEINIT;
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
            wake_cm55_from_cm33_before_send_ipc();
#endif
            result = mtb_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);
#ifdef COMPONENT_CM33
            wake_cm55_from_cm33_after_send_ipc();
#endif
            /* Wait for response from the other core on de-init_response_queue */
            result = mtb_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /* !COMPONENT_PSE84*/
            result = cyhal_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);

            /* Wait for response from the other core on de-init_response_queue */
            result = cyhal_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /* COMPONENT_PSE84*/
            CY_VCM_ERROR_CHECK(result);
        }
    }

    is_vcm_initialized = false;
    is_dual_core_vcm_initialized = false;

    cy_worker_thread_delete(&cy_vcm_worker_thread);

#ifndef USE_VIRTUAL_API
    (void)cy_rtos_set_mutex(&vcm_wcm_mutex);
#ifdef VCM_ENABLE_MQTT
    (void)cy_rtos_set_mutex(&vcm_mqtt_mutex);
#endif
#else
    (void)cy_rtos_set_mutex(&vcm_async_mutex);
#endif
    (void)cy_rtos_set_mutex(&vcm_request_mutex);

    result = cy_rtos_set_mutex(&init_complete_mutex);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    /* Deinit mutex */
#ifndef USE_VIRTUAL_API
    cy_rtos_deinit_mutex(&vcm_wcm_mutex);
#ifdef VCM_ENABLE_MQTT
    cy_rtos_deinit_mutex(&vcm_mqtt_mutex);
#endif
#else
    cy_rtos_deinit_mutex(&vcm_async_mutex);
#endif
    cy_rtos_deinit_mutex(&vcm_request_mutex);

    cy_rtos_deinit_mutex(&init_complete_mutex);

    return result;
}

#ifdef USE_VIRTUAL_API
cy_rslt_t cy_vcm_send_api_request(cy_vcm_request_t* request, cy_vcm_response_t* response)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager Send API request STARTS\n", __func__, __LINE__);

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res 0x%x\n", __func__, __LINE__, result);
        return CY_VCM_MUTEX_ERROR;
    }

    if((is_dual_core_vcm_initialized != true) || (is_vcm_initialized != true))
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Virtual Connectivity Manager is not initialized\n", __func__, __LINE__);
        response->result = NULL;
        cy_rtos_set_mutex(&vcm_request_mutex);
        return CY_VCM_INIT_ERROR;
    }

    if((request == NULL) || (response == NULL))
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Bad argument to cy_vcm_send_api_request\n", __func__, __LINE__);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return CY_VCM_BAD_ARG;
    }

    /* Sending API request */
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&request_queue_obj, request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /*!COMPONENT_PSE84*/
    result = cyhal_ipc_queue_put(&request_queue_obj, request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /*COMPONENT_PSE84*/
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push the API request to the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return CY_VCM_SEND_API_REQUEST_ERROR;
    }

    /* Receive the API response */
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
    cy_rtos_delay_milliseconds(50);
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
    result = mtb_ipc_queue_get(&response_queue_obj, response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /*!COMPONENT_PSE84*/
    result = cyhal_ipc_queue_get(&response_queue_obj, response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /*COMPONENT_PSE84*/
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to get the API response from the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return CY_VCM_API_RESPONSE_ERROR;
    }

    cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        result = CY_VCM_MUTEX_ERROR;
    }

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager Send API request ENDS\n", __func__, __LINE__);

    return result;
}

void cy_vcm_free(void* ptr)
{
    cy_rslt_t result;
    uint32_t ptr_val;

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res 0x%x\n", __func__, __LINE__, result);
        return;
    }

    /* Send IPC message with the address to primary core who has created the memory so that it can be freed */
    ptr_val = (uint32_t)ptr;

#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_before_send_ipc();
#endif
    result = mtb_ipc_queue_put(&free_queue_obj, &ptr_val, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#else /*!COMPONENT_PSE84*/
    result = cyhal_ipc_queue_put(&free_queue_obj, &ptr_val, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
#endif /*COMPONENT_PSE84*/
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push the API request to the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }
#if defined(COMPONENT_PSE84)
#ifdef COMPONENT_CM33
    wake_cm55_from_cm33_after_send_ipc();
#endif
#endif /*COMPONENT_PSE84*/

    cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    return;
}
#endif  /* USE_VIRTUAL_API */

#endif  /* ENABLE_MULTICORE_CONN_MW */
