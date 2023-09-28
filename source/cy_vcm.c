/******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
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
#include "cyhal_ipc.h"
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
/*******************************************************************************
 * Global variables
 ******************************************************************************/
#ifndef USE_VIRTUAL_API
#if (CY_CPU_CORTEX_M0P)
CY_SECTION_SHAREDMEM
#endif
static cy_wcm_event_callback_params_t  wcm_event_callback_params[MAX_WCM_DATA_EVENTS]; /* Array which stores the parameters of cy_wcm_event_callback_t function each time the WCM event callback is received from WHD */

#if (CY_CPU_CORTEX_M0P)
CY_SECTION_SHAREDMEM
#endif
static cy_wcm_event_data_t             local_wcm_event_data[MAX_WCM_DATA_EVENTS];      /* Array which stores the event data received from WHD */

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
cyhal_ipc_t async_cb_queue_obj, request_queue_obj, response_queue_obj, deinit_queue_obj, deinit_rsp_queue_obj, init_queue_obj, free_queue_obj;

/* Queue pool pointers */
void* async_cb_queue_pool, * request_queue_pool, *response_queue_pool, *deinit_queue_pool, *deinit_rsp_queue_pool, *init_queue_pool, *free_queue_pool;

/* Queue handles */
cyhal_ipc_queue_t* async_queue_handle, * request_queue_handle, *response_queue_handle, *deinit_queue_handle, *deinit_rsp_queue_handle, *init_queue_handle, *free_queue_handle;

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

    result = cy_rtos_set_mutex(&vcm_async_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }
    return;
}

/* write event callback for async-callback-queue */
void async_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
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

    cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }
    return;
}

/* write event callback for free_queue */
void free_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
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

    memcpy((void*)(&(local_wcm_event_data[wcm_event_index])), (void*)event_data, sizeof(cy_wcm_event_data_t));
    wcm_event_callback_params[wcm_event_index].event_data = &(local_wcm_event_data[wcm_event_index]);
    wcm_event_callback_params[wcm_event_index].event = event;
    vcm_async_callback_data.data = (void *)(&(wcm_event_callback_params[wcm_event_index]));
    vcm_async_callback_data.func_ptr = (void *)wcm_internal_func_ptr;
    result = cyhal_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push WCM event to the asynchronous callback queue. res = 0x%x\n", __func__, __LINE__, result);
    }
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
#endif
    static cy_mqtt_callback_params_t mqtt_callback_params;
    cy_vcm_cb_data                   vcm_async_callback_data;
    cy_rslt_t                        result;
    const char                       *payload;
    const char                       *topic;

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

    mqtt_callback_params.mqtt_handle = mqtt_handle;

    mqtt_callback_params.event = event;
    /* Store the topic and payload into a local memory before passing to the secondary core.
     * This memory will be freed in virtual_event_handler in MQTT library
     */
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
    mqtt_callback_params.event.data.pub_msg.received_message.payload = payload;
    mqtt_callback_params.event.data.pub_msg.received_message.topic = topic;

    mqtt_callback_params.user_data = user_data;
    vcm_async_callback_data.data = (void *)(&(mqtt_callback_params));
    vcm_async_callback_data.func_ptr = (void *)mqtt_internal_func_ptr;
    result = cyhal_ipc_queue_put(&async_cb_queue_obj, &vcm_async_callback_data, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push MQTT event to the asynchronous callback queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_mqtt_mutex);
        goto exit;
    }

    result = cy_rtos_set_mutex(&vcm_mqtt_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    return;

exit:
    free((char*)payload);
    free((char*)topic);
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
#endif
    static cy_rslt_t   api_result;
    cy_rslt_t result;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Request processing STARTS on primary application\n", __func__, __LINE__);

    result = cy_rtos_get_mutex(&vcm_request_mutex, CY_VCM_MAX_MUTEX_WAIT_TIME_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_get_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
        return;
    }

    /* Read the API request from request_queue */
    result = cyhal_ipc_queue_get(&request_queue_obj, &request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_get failed. res = 0x%x\n", __func__, __LINE__, result);
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
#endif
            static uint8_t connect_status = 0;
            connect_status = cy_wcm_is_connected_to_ap();
            response.result = (void*)&connect_status;
            break;
        }
        case CY_VCM_API_WCM_REG_EVENT_CB:
        {
            cy_wcm_register_event_callback_params_t *params;

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
            cy_wcm_deregister_event_callback_params_t *params;

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

    /* Push the result of API call to response_queue */
    result = cyhal_ipc_queue_put(&response_queue_obj, &response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    result = cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] API request processed\n", __func__, __LINE__);
}

/* write event callback for request_queue */
void request_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
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
    result = cyhal_ipc_queue_get(&init_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
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
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_get failed. res: 0x%x\n", __func__, __LINE__, result);
        (void)cy_rtos_set_mutex(&init_complete_mutex);
        return;
    }

    cyhal_ipc_queue_enable_event(&init_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);

    /* Delete the init_queue */
    cyhal_ipc_queue_free(&init_queue_obj);
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
void init_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
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

    result = cyhal_ipc_queue_get(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_get failed. res = 0x%x\n", __func__, __LINE__, result);
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
    result = cyhal_ipc_queue_put(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cyhal_ipc_queue_put failed. res = 0x%x\n", __func__, __LINE__, result);
    }

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
void deinit_queue_write_callback(void *callback_arg, cyhal_ipc_event_t event)
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
    }
    else /* hal_resource_opt is CY_VCM_USE_HAL_RESOURCE */
    {
        /* Get handles of all the queues */
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] VCM initialization STARTS in the second core\n", __func__, __LINE__);
        cyhal_ipc_queue_get_handle(&init_queue_obj, channel, INIT_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&async_cb_queue_obj, channel, ASYNC_CB_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&request_queue_obj, channel, REQUEST_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&response_queue_obj, channel, RESPONSE_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&deinit_queue_obj, channel, DEINIT_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&deinit_rsp_queue_obj, channel, DEINIT_RSP_QUEUE_NUM);
        cyhal_ipc_queue_get_handle(&free_queue_obj, channel, FREE_QUEUE_NUM);
    }

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

    if(hal_resource_opt == CY_VCM_USE_HAL_RESOURCE)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Initialization complete in second core. Informing the main core\n", __func__, __LINE__);
        vcm_event = CY_VCM_EVENT_INIT_COMPLETE;
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
        is_dual_core_vcm_initialized = true;
    }
    is_vcm_initialized = true;

    cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Virtual Connectivity Manager initialization ENDS - successfully Initialized\n", __func__, __LINE__);

    return result;

exit10:
    cyhal_ipc_queue_free(&deinit_rsp_queue_obj);
    deinit_rsp_queue_pool = NULL;
    deinit_rsp_queue_handle = NULL;
exit9:
    cyhal_ipc_queue_free(&deinit_queue_obj);
    deinit_queue_pool = NULL;
    deinit_queue_handle = NULL;
exit8:
    cyhal_ipc_queue_free(&async_cb_queue_obj);
    async_cb_queue_pool = NULL;
    async_queue_handle = NULL;
exit7:
    cyhal_ipc_queue_free(&response_queue_obj);
    response_queue_pool = NULL;
    response_queue_handle = NULL;
exit6:
    cyhal_ipc_queue_free(&request_queue_obj);
    request_queue_pool = NULL;
    request_queue_handle = NULL;
exit5:
    cyhal_ipc_queue_free(&init_queue_obj);
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
    cyhal_ipc_queue_enable_event(&async_cb_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
#else
    cyhal_ipc_queue_enable_event(&request_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
    cyhal_ipc_queue_enable_event(&free_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);
    wcm_internal_func_ptr = NULL;
#ifdef VCM_ENABLE_MQTT
    mqtt_internal_func_ptr = NULL;
#endif
#endif

    /* De-register the callback for write operation on deinit handling queue */
    cyhal_ipc_queue_enable_event(&deinit_queue_obj, CYHAL_IPC_QUEUE_WRITE, CYHAL_ISR_PRIORITY_DEFAULT, false);

    if(hal_resource_opt == CY_VCM_CREATE_HAL_RESOURCE)
    {
        if(is_dual_core_vcm_initialized == true)
        {
            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Informing the other core to de-initialize Virtual Connectivity Manager\n", __func__, __LINE__);

            /* Intimate the other core to de-initialize the Virtual Connectivity Manager by passing CY_VCM_EVENT_DEINIT through the de-init queue */
            vcm_event = CY_VCM_EVENT_DEINIT;
            result = cyhal_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);

            /* Wait for response from the other core on de-init_response_queue */
            result = cyhal_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);

            cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s[%d] Other core  has acknowledged de-initialization of Virtual Connectivity Manager\n", __func__, __LINE__);
        }
        else
        {
            /* Delete the init_queue resources if not already removed */
            if(is_vcm_init_complete == false)
            {
                cyhal_ipc_queue_free(&init_queue_obj);
                init_queue_pool = NULL;
                init_queue_handle = NULL;
            }
        }

        /* Free all queues. Set the queue_handles and queue_pool pointers to NULL */
        cyhal_ipc_queue_free(&free_queue_obj);
        free_queue_pool = NULL;
        free_queue_handle = NULL;
        cyhal_ipc_queue_free(&deinit_rsp_queue_obj);
        deinit_rsp_queue_pool = NULL;
        deinit_rsp_queue_handle = NULL;
        cyhal_ipc_queue_free(&deinit_queue_obj);
        deinit_queue_pool = NULL;
        deinit_queue_handle = NULL;
        cyhal_ipc_queue_free(&async_cb_queue_obj);
        async_cb_queue_pool = NULL;
        async_queue_handle = NULL;
        cyhal_ipc_queue_free(&response_queue_obj);
        response_queue_pool = NULL;
        response_queue_handle = NULL;
        cyhal_ipc_queue_free(&request_queue_obj);
        request_queue_pool = NULL;
        request_queue_handle = NULL;
    }
    else /* hal_resource_opt = CY_VCM_USE_HAL_RESOURCE */
    {
        /* Intimate the other core to de-initialize the Virtual Connectivity Manager by passing CY_VCM_EVENT_DEINIT through the de-init queue */
        if(is_dual_core_vcm_initialized == true)
        {
            vcm_event = CY_VCM_EVENT_DEINIT;
            result = cyhal_ipc_queue_put(&deinit_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
            CY_VCM_ERROR_CHECK(result);

            /* Wait for response from the other core on de-init_response_queue */
            result = cyhal_ipc_queue_get(&deinit_rsp_queue_obj, &vcm_event, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
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
    result = cyhal_ipc_queue_put(&request_queue_obj, request, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push the API request to the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return CY_VCM_SEND_API_REQUEST_ERROR;
    }

    /* Receive the API response */
    result = cyhal_ipc_queue_get(&response_queue_obj, response, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
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

    result = cyhal_ipc_queue_put(&free_queue_obj, &ptr_val, CY_VCM_MAX_QUEUE_WAIT_TIME_US);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] Failed to push the API request to the queue. res = 0x%x\n", __func__, __LINE__, result);
        cy_rtos_set_mutex(&vcm_request_mutex);
        return;
    }

    cy_rtos_set_mutex(&vcm_request_mutex);
    if(result != CY_RSLT_SUCCESS)
    {
        cy_vcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s[%d] cy_rtos_set_mutex failed. res = 0x%x\n", __func__, __LINE__, result);
    }

    return;
}
#endif  /* USE_VIRTUAL_API */

#endif  /* ENABLE_MULTICORE_CONN_MW */
