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
 * File Name:   cy_vcm_error.h
 *
 ******************************************************************************/
#ifndef CY_VCM_ERROR_H
#define CY_VCM_ERROR_H

#ifdef ENABLE_MULTICORE_CONN_MW

#include <stdlib.h>
#include "cy_result_mw.h"

/**
 * @defgroup cy_vcm_error VCM-specific error codes
 * @ingroup group_vcm_macros
 * Cypress middleware APIs return results of type cy_rslt_t.
 *
 * It consists of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
              Module base                   Type    Library specific error code
      +-----------------------------------+------+------------------------------+

      |CY_RSLT_MODULE_VCM_BASE            | 0x2  |           Error Code         |

      +-----------------------------------+------+------------------------------+
                14-bits                    2-bits            16-bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>.
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, that is part of [GitHub connectivity-utilities] (https://github.com/infineon/connectivity-utilities)
 *              For example, Virtual Connectivity Manager (VCM) uses CY_RSLT_MODULE_VCM_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */
 
/** Base error code for VCM */
#define CY_VCM_ERROR_BASE               CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_VCM_BASE, 0)

/** VCM Error Codes */
#define CY_VCM_INIT_ERROR               (CY_VCM_ERROR_BASE + 1) /**< VCM Initialization failed.           */
#define CY_VCM_SEND_API_REQUEST_ERROR   (CY_VCM_ERROR_BASE + 2) /**< Failed to send API request over IPC. */
#define CY_VCM_API_RESPONSE_ERROR       (CY_VCM_ERROR_BASE + 3) /**< Failed to get the API response.      */
#define CY_VCM_BAD_ARG                  (CY_VCM_ERROR_BASE + 4) /**< Bad argument.                        */
#define CY_VCM_WORKER_THREAD_ERROR      (CY_VCM_ERROR_BASE + 5) /**< Failed to created the worker thread  */
#define CY_VCM_MUTEX_ERROR              (CY_VCM_ERROR_BASE + 6) /**< VCM mutex error                      */

/** Check to see if result is success or not */
#define CY_VCM_ERROR_CHECK(x)    do {                                                                            \
                                    if(x != CY_RSLT_SUCCESS)                                                     \
                                    {                                                                            \
                                        printf("%s[%d] VCM Assertion code: %ld\n", __FILE__, __LINE__, (unsigned long)x); abort();\
                                    }                                                                            \
                                 } while(0)

/** \} error codes */

#endif /* ENABLE_MULTICORE_CONN_MW */

#endif /* CY_VCM_ERROR_H */
