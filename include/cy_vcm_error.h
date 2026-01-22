/******************************************************************************
 * (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
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
#define CY_VCM_WORKER_THREAD_ERROR      (CY_VCM_ERROR_BASE + 5) /**< Failed to created the worker thread. */
#define CY_VCM_MUTEX_ERROR              (CY_VCM_ERROR_BASE + 6) /**< VCM mutex error.                     */
#define CY_VCM_NOT_INITIALIZED          (CY_VCM_ERROR_BASE + 7) /**< VCM not initialized.                 */

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
