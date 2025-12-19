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
 * File Name:   cy_vcm_internal.h
 *
 * Description: This file contains definitions of internal constants, structures and APIs of
 *              virtual connectivity manager.
 *
 ******************************************************************************/
#ifndef CY_VCM_INTERNAL_H
#define CY_VCM_INTERNAL_H

#ifdef ENABLE_MULTICORE_CONN_MW
/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_result.h"
#include "cy_vcm_error.h"
#include "cy_log.h"
/*******************************************************************************
 * Macros
 *******************************************************************************/
/* Base value of the virtual API ID */
#define CY_VCM_BASE_API_ID                (0x1000)

/* WCM specific macros */
#define CY_VCM_BASE_WCM_MODULE            (0x100)
#define CY_VCM_API_WCM_IS_CONNECTED_AP    (CY_VCM_BASE_API_ID + CY_VCM_BASE_WCM_MODULE + 1)
#define CY_VCM_API_WCM_REG_EVENT_CB       (CY_VCM_BASE_API_ID + CY_VCM_BASE_WCM_MODULE + 2)
#define CY_VCM_API_WCM_DEREG_EVENT_CB     (CY_VCM_BASE_API_ID + CY_VCM_BASE_WCM_MODULE + 3)

/* MQTT specific macros */
#define CY_VCM_BASE_MQTT_MODULE           (0x200)
#define CY_VCM_API_MQTT_SUBSCRIBE         (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 1)
#define CY_VCM_API_MQTT_UNSUBSCRIBE       (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 2)
#define CY_VCM_API_MQTT_PUBLISH           (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 3)
#define CY_VCM_API_MQTT_GET_HANDLE        (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 4)
#define CY_VCM_API_MQTT_REG_EVENT_CB      (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 5)
#define CY_VCM_API_MQTT_DEREG_EVENT_CB    (CY_VCM_BASE_API_ID + CY_VCM_BASE_MQTT_MODULE + 6)

#if defined(COMPONENT_PSE84)
/* Memory byte alignment needed for shared memory */
#define CY_VCM_MEMORY_BYTE_ALIGNMENT      (32U)
#endif
/*******************************************************************************
 * Structures
 *******************************************************************************/
/**
 * Virtual-Connectivity-Manager remote API request structure
 */
typedef struct
{
    uint16_t   api_id;    /* API ID */
    void*      params;    /* API specific structure cast to void* which contains all parameters of the API corresponding to the api_id */
} cy_vcm_request_t;

/**
 * Virtual-Connectivity-Manager response structure
 */
typedef struct
{
    void*     result;     /* Pointer which holds the return value of an API call */
} cy_vcm_response_t;

/*******************************************************************************
 * Internal callback function pointer
 *******************************************************************************/
/**
 * VCM internal event callback function pointer type. This will be called from the VCM's asynchronous callback processing thread based on receiving an event from primary core.
 * @param[in] arg            : Pointer to the event specific data structure from VCM's asynchronous callback processing thread passed as a void* pointer.
 *
 * Note: The callback function will be executed in the context of the VCM.
 */
typedef void (*cy_vcm_internal_callback_t)(void* arg);

/*******************************************************************************
 * Internal Function prototype
 *******************************************************************************/
/**
 * Send API request from secondary core to the primary core containing the actual implementation and receive the response.
 *
 * @param[in]  request     Pointer to the API request structure filled by the user \ref cy_vcm_request_t
 * @param[in]  response    Pointer to the API response structure \ref cy_vcm_response_t
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t cy_vcm_send_api_request(cy_vcm_request_t* request, cy_vcm_response_t* response);

/**
 * This API allows us to send a notification to the primary core to free a particular address which was created in primary core.
 * Sends a request from secondary core to the primary core along with the address to be freed.
 *
 * @param[in]  ptr     Pointer that should be freed by primary core
 *
 */
void cy_vcm_free(void* ptr);

#endif  /* ENABLE_MULTICORE_CONN_MW */

#endif /* CY_VCM_INTERNAL_H */
/* [] END OF FILE */
