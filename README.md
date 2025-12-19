# Virtual Connectivity Manager (VCM)

Virtual-Connectivity-Manager (VCM) is a library that enables connectivity libraries to add multi-core support through virtualization. Virtualization allows the connectivity stack running on one core to be accessed from another core using Inter-Processor Communication (IPC). With VCM, the connectivity stack can be run on two cores, where one core contains the full connectivity stack (primary core) and the other core contains a subset of virtual-only APIs (secondary core).

Internally, VCM uses IPC to send API requests from the secondary core to the primary core where the API is executed, and then the result is sent back to the secondary core. This makes it seem like the API was executed on the secondary core itself. VCM simplifies the process of adding multi-core support by hiding the details of IPC from both the connectivity libraries and the end application.

## Features and functionality

The current implementation has the following features and functionality:

- Supports the following WCM virtual APIs:
   - `cy_wcm_is_connected_to_ap`
   - `cy_wcm_register_event_callback`
   - `cy_wcm_deregister_event_callback`

- Supports the following MQTT virtual APIs:
   - `cy_mqtt_get_handle`
   - `cy_mqtt_register_event_callback`
   - `cy_mqtt_deregister_event_callback`
   - `cy_mqtt_subscribe`
   - `cy_mqtt_unsubscribe`
   - `cy_mqtt_publish`

## Supported platforms

This library and its features are supported on the following Infineon platforms:

- [PSOC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-MUR-43439M2)]( https://www.infineon.com/cms/en/product/evaluation-boards/cy8ceval-062s2/ )
- PSOC&trade; Edge E84 Evaluation Kit

## Dependent libraries

This library is not bundled as part of any other library by default.

## Quick start

- Define the following compile-time macro in the project Makefile of primary core side:
   ```
   DEFINES+=ENABLE_MULTICORE_CONN_MW
   ```
- Define the following compile-time macros in the project Makefile of secondary core side:
   ```
   DEFINES+=ENABLE_MULTICORE_CONN_MW USE_VIRTUAL_API
   ```
- To use MQTT virtual APIs, define the following compile-time macro in the project Makefile of both cores:
   ```
   DEFINES+=VCM_ENABLE_MQTT
   ```
- Call the `cy_vcm_init()` function provided by the VCM library in both the projects.

   **Note for PSOC&trade; Edge E84:** 

    1. The channel number passed to `cy_vcm_init()` must be different from the channel used in `mtb_ipc_init()`. For example, if `mtb_ipc_init()` uses `MTB_IPC_CHAN_0`, then `cy_vcm_init()` should use `MTB_IPC_CHAN_1`.

    2. IPC object should be passed to VCM during initialization:
       ```
       ///////////////////////////////////////////////////////////////////////
       // Initialization on first core (Example, CM33)
       ///////////////////////////////////////////////////////////////////////

       #include "cybsp.h"
       ...
       ...
       cy_vcm_config_t vcm_config;
       vcm_config.ipc_obj = &cybsp_cm33_ipc_instance;             // MTB IPC object created for CM33 which is initialized during BSP initialization.
       vcm_config.hal_resource_opt = CY_VCM_CREATE_HAL_RESOURCE;  // First core creates resources
       vcm_config.channel_num = MTB_IPC_CHAN_1;                   // Different from MTB IPC channel
       vcm_config.event_cb = vcm_event_callback;                  // Event callback function
       
       result = cy_vcm_init(&vcm_config);
       if (result != CY_RSLT_SUCCESS) 
       {
           printf("VCM init failed on first core: 0x%lx\n", result);
       }
       
       ///////////////////////////////////////////////////////////////////////
       // Initialization on second core (Example, CM55)
       ///////////////////////////////////////////////////////////////////////

       #include "cybsp.h"
       ...
       ...
       cy_vcm_config_t vcm_config2;
       vcm_config2.ipc_obj = &cybsp_cm55_ipc_instance;         // MTB IPC object created for CM55 which is initialized during BSP initialization.
       vcm_config2.hal_resource_opt = CY_VCM_USE_HAL_RESOURCE; // Second core uses resources
       vcm_config2.channel_num = MTB_IPC_CHAN_1;               // Same channel as first core
       vcm_config2.event_cb = vcm_event_callback;              // Event callback function
       
       result2 = cy_vcm_init(&vcm_config2);
       if (result2 != CY_RSLT_SUCCESS) 
       {
           printf("VCM init failed on second core: 0x%lx\n", result2);
       }
       ```   
       
   **Notes:**

    1. To ensure that the VCM initialization is synchronized, the project which boots first (i.e., CM0+ project in case of PSOC&trade; 62S2) must call `cy_vcm_init` before it brings up the second project (i.e., CM4 project in case of PSOC&trade; 62S2).
     
    2. The first project must initialize VCM by passing `config.hal_resource_opt` as `CY_VCM_CREATE_HAL_RESOURCE` in `cy_vcm_init`. The second project must pass `config.hal_resource_opt` as `CY_VCM_USE_HAL_RESOURCE`.

    3. VCM initialization can also be done in any order from any core, provided that the first project initializing VCM sets `config.hal_resource_opt` as `CY_VCM_CREATE_HAL_RESOURCE` in `cy_vcm_init` and the second project sets `config.hal_resource_opt` as `CY_VCM_USE_HAL_RESOURCE`. In this case synchronizing VCM initialization between the two cores will be the responsibility of the application. If not synchronized correctly, this can lead to unexpected behavior.

- VCM utilizes IPC queue resource to perform inter-core communication. Each IPC queue is associated with a *channel number* and *queue number*.

  | Channel Number | Queue Number |
  | ------- | ---------- |
  | Any IPC Channel configured by the application for VCM | 1-7 (Reserved by VCM) |

  **Note:** The application cannot use queue numbers 1-7 for the configured channel number, as they are reserved for VCM.

- VCM utilizes one of the IPC queues to send asynchronous event callback data from the primary core to the secondary core. By default, the size of this queue is set to 20.

  However, if the application expects to receive event callbacks in the secondary core in large bursts, this queue size can be modified to avoid queue overflow.

  To customize the event queue size, define and set the *CY_VCM_EVENT_QUEUE_SIZE* macro in the Makefile of the project which initializes VCM with `config.hal_resource_opt` as `CY_VCM_CREATE_HAL_RESOURCE`.
  
  For e.g., the Makefile entry for updating the queue size to 25 would be as follows:
  ```
  DEFINES+=CY_VCM_EVENT_QUEUE_SIZE=25
  ```

## Enable debug logs in VCM library

### Enable VCM log messages

The VCM library disables all the debug log messages by default. To enable log messages, the application must perform the following:

1. To enable VCM logs in a dual-core application, add the `ENABLE_VCM_LOGS` macro to the *DEFINES* in the project Makefile. Depending on where the VCM logs need to be enabled, you can add this macro to either side of the project Makefile.

   ```
   DEFINES+=ENABLE_VCM_LOGS
   ```
2. Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library.

   See [connectivity-utilities library API documentation]( https://Infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html ).

### Enable logs in dual core application

**Note:** The following instructions are specific to PSOC&trade; 62S2 platforms. For other supported platforms, consult the platform-specific documentation.

To enable debug log messages on both cores, the application needs to reserve two distinct UART ports. 

If you are using one of Infineon's BSPs for the connectivity application, you can use the on-board KitProg3 USB-UART COM port to print debug logs for one core. However, you still need another USB-UART bridge to print debug logs for the other core.

- One core can use the default CYBSP_DEBUG_UART_TX and CYBSP_DEBUG_UART_RX pins to enable the SCB block for KitProg3 using the retarget-io library.
  ```
  cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
  ```

- The other core can use any of the remaining SCB blocks to select the appropriate TX and RX pins. These TX and RX pins can be configured using the retarget-io library.

  - For example, in the CY8CEVAL-062S2-MUR-43439M2 board, the following pins can be configured for an additional UART port.
    - P13.4 and P13.5 (CYBSP_MIKROBUS_UART_RX, CYBSP_MIKROBUS_UART_TX)
    - P5.4 and P5.5 (CYBSP_D4, CYBSP_D5)
  ```
  cy_retarget_io_init(CYBSP_D5, CYBSP_D4, CY_RETARGET_IO_BAUDRATE);
  ```
  - For the second core, the selected TX and RX pins can be connected to an external USB-UART bridge (FTDI) using jumper wires, which will be emulated as a UART COM port.

## Additional information

- [Virtual Connectivity Manager RELEASE.md]( ./RELEASE.md )
- [Virtual Connectivity Manager version]( ./version.xml )
- [Virtual Connectivity Manager API documentation]( https://infineon.github.io/virtual-connectivity-manager/api_reference_manual/html/index.html )
- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos]( https://www.infineon.com/modustoolbox )
- [ModusToolbox&trade; code examples]( https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software )
