/*
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
 */

/**
 * @file cy_vcm.h
 * @brief Virtual-Connectivity-Manager (VCM) is a library that enables connectivity libraries
 * to add multi-core support through virtualization. Virtualization allows access of connectivity stack
 * running on one core from another core using Inter Process Communication (IPC). The connectivity stack 
 * can now be run on two cores, with one core containing the full connectivity stack (primary core) and
 * the other core containing the subset of virtual-only APIs (secondary core). Underneath, VCM pipes the
 * API requests over IPC to the primary core where the API is actually executed and the result is passed back
 * to the secondary core. To the secondary core application, it is as though the API was executed
 * and returned on its own core. The VCM abstracts out the implementation details and complexity of IPC from
 * both the connectivity libraries and the end application, thus making it simple to add multi-core support.
 *
 * The library provides an API to initialize the VCM library and set up the required IPC communication paths
 * between the two cores. The applications on both cores must initialize this library with appropriate compile-time
 * flags as mentioned below. The de-init API allows the user to de-initialize the VCM library and purge all the 
 * resources created during initialization. The library also notifies the application of its state change
 * through an event notification mechanism. See individual APIs for more details.
 *
 * The library currently supports the following virtual APIs:
 * WCM virtual APIs:
 * - cy_wcm_is_connected_to_ap
 * - cy_wcm_register_event_callback
 * - cy_wcm_deregister_event_callback
 * MQTT virtual APIs:
 * - cy_mqtt_get_handle
 * - cy_mqtt_register_event_callback
 * - cy_mqtt_deregister_event_callback
 * - cy_mqtt_subscribe
 * - cy_mqtt_unsubscribe
 * - cy_mqtt_publish
 *
 * In order to use this library, define the following compile-time macros in the application's Makefile:
 * DEFINES+=ENABLE_MULTICORE_CONN_MW
 * Additionally, define the following macro in the secondary core application alone:
 * DEFINES+=USE_VIRTUAL_API
 * For more details, refer to the Quick Start section of Readme.md.
 * 
 * Note: Ensure that all PDL initializations are complete before initializing Virtual Connectivity Manager.
 *
 ******************************************************************************/
#ifndef CY_VCM_H
#define CY_VCM_H

/**
 * \defgroup group_vcm_enums Enumerated Types
 * \defgroup group_vcm_typedefs Typedefs
 * \defgroup group_vcm_structures Structures
 * \defgroup group_vcm_functions Functions
 */
/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include <stdbool.h>
#include "cy_result.h"
#include "cy_vcm_internal.h"

/**
 * \addtogroup group_vcm_enums
 * \{
 */
 
/*******************************************************************************
 *                    Enumerations
 *******************************************************************************/

/**
 * Virtual-Connectivity-Manager event type
 */
 
typedef enum
{
    CY_VCM_EVENT_INIT_COMPLETE = 0,    /**< Event generated on the core which initializes VCM first when VCM initialization is complete on both cores */
    CY_VCM_EVENT_DEINIT                /**< Event generated on a core when the other core initiates VCM de-initialization */
} cy_vcm_event_t;

/**
 * Virtual-Connectivity-Manager HAL resource option type
 */

typedef enum
{
    CY_VCM_CREATE_HAL_RESOURCE = 0,    /**< Option to be set during \ref cy_vcm_init in the core which initializes VCM first. Setting this option creates all the HAL resources on that core. */
    CY_VCM_USE_HAL_RESOURCE            /**< Option to be set during \ref cy_vcm_init in the core which initializes VCM second. Setting this option will use the HAL resources created by the other core. */
} cy_vcm_hal_resource_opt_t;

/** \} group_vcm_enums */

/**
 * \addtogroup group_vcm_typedefs
 * \{
 */
 
/*******************************************************************************
 *                    Function Pointer
 *******************************************************************************/
 
/**
 * VCM event callback function pointer type
 * @param[in] event            : VCM event type.
 *
 * \note The callback function will be executed in the context of the VCM.
 * \note Refer to \ref cy_vcm_event_t for VCM events and their applicability on each core.
 */
typedef void (*cy_vcm_event_callback_t)(cy_vcm_event_t event);

/** \} group_vcm_typedefs */

/**
 * \addtogroup group_vcm_structures
 * \{
 */
 
/*******************************************************************************
*                     Structures
*******************************************************************************/
/**
 * Virtual-Connectivity-Manager config structure
 */
typedef struct
{
#if defined(COMPONENT_PSE84)
    void*                     ipc_obj;          /**< Pointer to MTB IPC object. User should initialize this object using mtb_ipc_init and pass the address to VCM. */
#endif
    cy_vcm_hal_resource_opt_t hal_resource_opt; /**< This is used to identify the core that should create the HAL resources. The core which executes first, should set this value to CY_VCM_CREATE_HAL_RESOURCE and initialize VCM before enabling the other core. The other core should initialize VCM by setting this variable to CY_VCM_USE_HAL_RESOURCE */
    uint32_t                  channel_num;      /**< IPC channel number passed by user during initialization (e.g. CYHAL_IPC_CHAN_0). The channel number which user can use are defined in cyhal_ipc_impl.h */
    cy_vcm_event_callback_t   event_cb;         /**< Callback function which receives the VCM events. It will be executed in the context of the VCM */
} cy_vcm_config_t;

/** \} group_vcm_structures */

/**
 * \addtogroup group_vcm_functions
 * \{
 * * The VCM library internally creates multiple threads; the created threads are executed with the "CY_RTOS_PRIORITY_ABOVENORMAL" priority.
 * * The definition of the CY_RTOS_PRIORITY_ABOVENORMAL macro is located at "mtb-shared/abstraction-rtos/include/COMPONENT_FREERTOS/cyabs_rtos_impl.h".
 * * The VCM APIs are thread-safe.
 */
 
/*******************************************************************************
*                     Function prototypes
*******************************************************************************/

/**
 * Initializes the virtual connectivity manager.
 * 
 * This function initializes the VCM resources, and HAL resources(queues) for the user given IPC channel. This function should be called in both cores before calling any virtual APIs.
 *
 * \note The core which executes first, should initialize VCM before enabling the other core, and should set config.hal_resource_opt to CY_VCM_CREATE_HAL_RESOURCE. The other core should initialize VCM with config.hal_resource_opt set to CY_VCM_USE_HAL_RESOURCE.
 *
 * @param[in]  config      Pointer to the config structure filled by the user \ref cy_vcm_config_t
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t cy_vcm_init(cy_vcm_config_t *config);

/**
 * De-initializes the virtual connectivity manager.
 * \note User should de-register all the virtual callbacks registered with WCM and/or MQTT before calling \ref cy_vcm_deinit.
 * \note After invoking \ref cy_vcm_deinit on either core, \ref cy_vcm_init cannot be invoked again. The expectation is that \ref cy_vcm_init and \ref cy_vcm_deinit should be invoked only once.
 *
 * @return CY_RSLT_SUCCESS if successful; failure code otherwise.
 */
cy_rslt_t cy_vcm_deinit();

/** \} group_vcm_functions */

#endif /* CY_VCM_H */
/* [] END OF FILE */
