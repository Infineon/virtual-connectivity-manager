# Virtual Connectivity Manager (VCM) — Multi-Core Connectivity via IPC Virtualization

# Overview

Virtual Connectivity Manager (VCM) is a ModusToolbox&trade; middleware library that enables connectivity libraries to add multi-core support through virtualization. Virtualization allows the connectivity stack running on one core (the primary core) to be accessed from another core (the secondary core) using Inter-Processor Communication (IPC). With VCM, one core runs the full connectivity stack while the other core runs a subset of virtual-only APIs.

Internally, VCM uses IPC to forward API requests from the secondary core to the primary core where the API is executed, and then returns the result to the secondary core. To the secondary-core application, it appears as though the API executed locally. VCM hides the details of IPC from both the connectivity libraries and the end application, making it simple to add multi-core support.

For the complete API reference, see the [Virtual Connectivity Manager Reference Manual](https://infineon.github.io/virtual-connectivity-manager/api_reference_manual/html/index.html).

# Features

- Adds multi-core support to connectivity libraries through IPC-based virtualization, with no IPC details exposed to the application
- Runs the full connectivity stack on a primary core and a subset of virtual-only APIs on a secondary core
- Supports Wi-Fi Connection Manager (WCM) virtual APIs:
  - `cy_wcm_is_connected_to_ap`
  - `cy_wcm_register_event_callback`
  - `cy_wcm_deregister_event_callback`
- Supports MQTT virtual APIs:
  - `cy_mqtt_get_handle`
  - `cy_mqtt_register_event_callback`
  - `cy_mqtt_deregister_event_callback`
  - `cy_mqtt_subscribe`
  - `cy_mqtt_unsubscribe`
  - `cy_mqtt_publish`
- Delivers asynchronous event callbacks from the primary core to the secondary core through a configurable IPC event queue

# When to Use

Use VCM when building a multi-core application in which the connectivity stack (for example, Wi-Fi or MQTT) runs on one core and must be accessed from a second core without duplicating the stack. Typical scenarios include:

- Offloading the connectivity stack to a primary core while application logic on a secondary core issues connectivity requests
- Adding multi-core support to a connectivity library (WCM, MQTT) without implementing IPC in the application
- PSOC&trade; 62 dual-core (CM0+/CM4) and PSOC&trade; Edge E84 (CM33/CM55) designs that share a single connectivity stack across cores

# Prerequisites

## Hardware Requirements

VCM is a software middleware library and requires a multi-core Infineon MCU. It is supported on the following device families:

- PSOC&trade; 62 dual-core (Arm&reg; Cortex&reg;-M0+ and Cortex&reg;-M4) with Wi-Fi connectivity (for example, CY8CEVAL-062S2-MUR-43439M2)
- PSOC&trade; Edge E84 (Arm&reg; Cortex&reg;-M33 and Cortex&reg;-M55)

For the exact list of validated kits for this release, see [RELEASE.md](./RELEASE.md).

To print debug logs from both cores simultaneously, a second UART port (for example, an external USB-UART bridge such as an FTDI adapter) is required in addition to the on-board KitProg3 USB-UART COM port. See the Enable Debug Logs section for details.

## Supported software and tools

This version of the library was validated for compatibility with the following software and tools:

| Software and tools                                             | Version |
| :---                                                           | :---    |
| ModusToolbox&trade; software environment                       | 3.9     |
| ModusToolbox&trade; device configurator                        | 5.90    |
| GCC compiler                                                   | 14.2.1  |
| IAR compiler                                                   | 9.50.2  |
| Arm&reg; compiler 6                                            | 6.22    |
| LLVM Compiler                                                  | 22.1.0  |

## Read Documentation First

- [Virtual Connectivity Manager Reference Manual](https://infineon.github.io/virtual-connectivity-manager/api_reference_manual/html/index.html) - Complete API reference
- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox) - Toolchain setup and getting started

## Choose Starting Template

Create an Empty application (always prefer starting from the Empty template) for each core of your multi-core design, or start from a multi-core connectivity code example that already targets your device.

## Add Required Libraries

VCM is not bundled as part of any other library by default; add it to each core's project explicitly. Add libraries using the ModusToolbox&trade; **Library Manager** (available in the ModusToolbox&trade; for VS Code extension and the Eclipse IDE), or by creating the corresponding `.mtb` files in the application's `deps` folder and running `make getlibs`.

- [Virtual Connectivity Manager](https://github.com/Infineon/virtual-connectivity-manager) - This library. Add it to both the primary-core and secondary-core projects. To add it manually, create `deps/virtual-connectivity-manager.mtb` with:
  ```
  https://github.com/Infineon/virtual-connectivity-manager#latest-v1.X#$$ASSET_REPO$$/virtual-connectivity-manager/latest-v1.X
  ```
- [Wi-Fi Connection Manager](https://github.com/Infineon/wifi-connection-manager) and/or [MQTT](https://github.com/Infineon/mqtt) - The connectivity libraries whose virtual APIs you intend to use, added on the primary core.
- [connectivity-utilities](https://github.com/Infineon/connectivity-utilities) - Provides the `cy-log` module used to print VCM debug logs (pulled in as a dependency of the connectivity libraries).

## Configure Makefile

VCM is enabled through compile-time `DEFINES` rather than a `COMPONENT`. Add the following to each core's project Makefile:

- Primary core:
  ```
  DEFINES+=ENABLE_MULTICORE_CONN_MW
  ```
- Secondary core:
  ```
  DEFINES+=ENABLE_MULTICORE_CONN_MW USE_VIRTUAL_API
  ```
- To use MQTT virtual APIs, add the following on both cores:
  ```
  DEFINES+=VCM_ENABLE_MQTT
  ```

## Configure in the Tool

VCM does not require a dedicated Configurator. IPC channel and queue configuration is done in code through `cy_vcm_init()` (see the Multi-Core (Virtual API) Usage section).

## Add Headers

Include the VCM header in the application source on both cores:

```c
#include "cy_vcm.h"
```

# Quick Start

This quick start demonstrates initializing VCM on both cores of a multi-core device so that virtual connectivity APIs can be used from the secondary core.

**Step 1:** Create an Empty application for each core and add the required libraries (see the Add Required Libraries section).

**Step 2:** Add the compile-time `DEFINES` to each core's Makefile (see the Configure Makefile section).

**Step 3:** Include `cy_vcm.h` and call `cy_vcm_init()` on both cores (see the Multi-Core (Virtual API) Usage section for initialization order and configuration rules).

**Step 4:** Add the initialization code to your `main.c` on each core (see the Sample Code section for a complete example).

**Expected Outcome:** VCM initializes on both cores, the primary core receives the `CY_VCM_EVENT_INIT_COMPLETE` event, and the secondary core can invoke virtual connectivity APIs as if they ran locally.

## Multi-Core (Virtual API) Usage

VCM uses IPC to forward virtual API calls from the secondary core to the primary core. Observe the following rules when initializing and using VCM:

- Call `cy_vcm_init()` on both cores before invoking any virtual API.
- The first project to initialize VCM must pass `config.hal_resource_opt` as `CY_VCM_CREATE_HAL_RESOURCE`; the second project must pass `CY_VCM_USE_HAL_RESOURCE`.
- To keep initialization synchronized, the project that boots first (the CM0+ project on PSOC&trade; 62S2) should call `cy_vcm_init()` before it brings up the second project (the CM4 project on PSOC&trade; 62S2). VCM can be initialized in any order from any core as long as the create/use resource rule above is honored; if the order is not synchronized, the application is responsible for coordinating initialization, otherwise behavior is undefined.

**Note for PSOC&trade; Edge E84:**

1. The channel number passed to `cy_vcm_init()` must be different from the channel used in `mtb_ipc_init()`. For example, if `mtb_ipc_init()` uses `MTB_IPC_CHAN_0`, then `cy_vcm_init()` should use `MTB_IPC_CHAN_1`.
2. An IPC object must be passed to VCM during initialization (see the Sample Code section).

**IPC queue reservation:** VCM uses IPC queue resources for inter-core communication. Each IPC queue is associated with a channel number and a queue number.

| Channel number | Queue number |
| :--- | :--- |
| Any IPC channel configured by the application for VCM | 1-7 (reserved by VCM) |

**Note:** The application cannot use queue numbers 1-7 for the configured channel number, as they are reserved for VCM.

**Event queue size:** VCM uses one of the IPC queues to send asynchronous event-callback data from the primary core to the secondary core. By default, the size of this queue is 20. If the application expects event callbacks in large bursts on the secondary core, increase this size to avoid queue overflow. Define the `CY_VCM_EVENT_QUEUE_SIZE` macro in the Makefile of the project that initializes VCM with `config.hal_resource_opt` as `CY_VCM_CREATE_HAL_RESOURCE`. For example, to set the queue size to 25:

```
DEFINES+=CY_VCM_EVENT_QUEUE_SIZE=25
```

**Device support note:** Virtual APIs are supported on all PSOC&trade; 62 devices, but have been validated on the CY8CEVAL-062S2-MUR-43439M2 kit.

## Sample Code

The following example initializes VCM on both cores. The primary core (for example, CM0+ on PSOC&trade; 62 or CM33 on PSOC&trade; Edge E84) creates the resources and boots the secondary core; the secondary core (for example, CM4 or CM55) uses the resources.

### Primary core (main.c)

```c
#include "cybsp.h"
#include "cy_vcm.h"

/* Set when VCM initialization completes on the other core. */
static volatile bool is_vcm_init_complete = false;

/* VCM event callback registered by the primary core. */
void vcm_event_callback(cy_vcm_event_t event)
{
    switch (event)
    {
        case CY_VCM_EVENT_INIT_COMPLETE:
            is_vcm_init_complete = true;
            break;
        default:
            break;
    }
}

void primary_core_vcm_init(void)
{
    cy_rslt_t result;
    cy_vcm_config_t config;

#if defined(COMPONENT_PSE84)
    config.ipc_obj = &cybsp_cm33_ipc_instance;   /* MTB IPC object created for CM33 during BSP init. */
    config.channel_num = MTB_IPC_CHAN_1;         /* Must differ from the mtb_ipc_init() channel. */
#else
    config.channel_num = CYHAL_IPC_CHAN_0;
#endif
    config.hal_resource_opt = CY_VCM_CREATE_HAL_RESOURCE;  /* First core creates resources. */
    config.event_cb = vcm_event_callback;

    result = cy_vcm_init(&config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("cy_vcm_init failed on the primary core: 0x%lx\n", (unsigned long)result);
    }

    /* Bring up the secondary core. It must call cy_vcm_init() when it starts. */
#if defined(COMPONENT_PSE84)
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
#else
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
#endif

    /* Wait for the secondary core to finish VCM initialization. */
    while (!is_vcm_init_complete)
    {
        cy_rtos_delay_milliseconds(1000);
    }

    /* Virtual APIs can now be used. */
}
```

### Secondary core (main.c)

```c
#include "cybsp.h"
#include "cy_vcm.h"

/* VCM event callback registered by the secondary core. */
void vcm_event_callback(cy_vcm_event_t event)
{
    switch (event)
    {
        case CY_VCM_EVENT_DEINIT:
            /* Primary core initiated deinit; deinitialize VCM locally. */
            break;
        default:
            break;
    }
}

void secondary_core_vcm_init(void)
{
    cy_rslt_t result;
    cy_vcm_config_t config;

#if defined(COMPONENT_PSE84)
    config.ipc_obj = &cybsp_cm55_ipc_instance;   /* MTB IPC object created for CM55 during BSP init. */
    config.channel_num = MTB_IPC_CHAN_1;         /* Same channel as the primary core. */
#else
    config.channel_num = CYHAL_IPC_CHAN_0;
#endif
    config.hal_resource_opt = CY_VCM_USE_HAL_RESOURCE;  /* Second core uses resources. */
    config.event_cb = vcm_event_callback;

    result = cy_vcm_init(&config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("cy_vcm_init failed on the secondary core: 0x%lx\n", (unsigned long)result);
    }

    /* Proceed with the rest of the application logic and virtual API calls. */
}
```

### Deinitialization and recovery

`cy_vcm_deinit()` can be called from either core. When one core calls `cy_vcm_deinit()`, the other core receives the `CY_VCM_EVENT_DEINIT` event and must call `cy_vcm_deinit()` locally to release its resources and recover cleanly:

```c
void handle_remote_deinit(cy_vcm_event_t event)
{
    if (event == CY_VCM_EVENT_DEINIT)
    {
        cy_rslt_t result = cy_vcm_deinit();
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_vcm_deinit failed: 0x%lx\n", (unsigned long)result);
        }
    }
}
```

## Expected Outcome

When the Quick Start example runs successfully, you should observe:

- **Behavior:** VCM initializes on both cores. The primary core creates the IPC resources and boots the secondary core; the secondary core initializes VCM and signals completion, after which virtual connectivity APIs invoked on the secondary core execute on the primary core.
- **Terminal output:** No `cy_vcm_init failed ...` messages are printed on either core. With VCM logs enabled (see the Enable Debug Logs section), initialization progress is printed on the configured UART port(s).

# Enable Debug Logs

## Enable VCM log messages

The VCM library disables all debug log messages by default. To enable them:

1. Add the `ENABLE_VCM_LOGS` macro to `DEFINES` in the project Makefile of the core(s) where logs are needed:
   ```
   DEFINES+=ENABLE_VCM_LOGS
   ```
2. Call the `cy_log_init()` function provided by the `cy-log` module. `cy-log` is part of the `connectivity-utilities` library. See the [connectivity-utilities Reference Manual](https://Infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html).

## Enable logs in a dual-core application

**Note:** The following instructions are specific to PSOC&trade; 62S2 platforms. For other supported platforms, consult the platform-specific documentation.

To print debug logs from both cores, the application must reserve two distinct UART ports. If you are using one of Infineon's BSPs, you can use the on-board KitProg3 USB-UART COM port for one core; a second USB-UART bridge is required for the other core.

- One core can use the default `CYBSP_DEBUG_UART_TX` and `CYBSP_DEBUG_UART_RX` pins to enable the SCB block for KitProg3 using the retarget-io library:
  ```
  cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
  ```
- The other core can use any of the remaining SCB blocks with appropriate TX and RX pins configured through the retarget-io library. For example, on the CY8CEVAL-062S2-MUR-43439M2 board the following pins can be configured for an additional UART port:
  - P13.4 and P13.5 (`CYBSP_MIKROBUS_UART_RX`, `CYBSP_MIKROBUS_UART_TX`)
  - P5.4 and P5.5 (`CYBSP_D4`, `CYBSP_D5`)
  ```
  cy_retarget_io_init(CYBSP_D5, CYBSP_D4, CY_RETARGET_IO_BAUDRATE);
  ```
  For the second core, connect the selected TX and RX pins to an external USB-UART bridge (FTDI) using jumper wires; it is emulated as a UART COM port.

# Troubleshooting

- **VCM initialization hangs or fails:** Ensure the first core to initialize VCM passes `CY_VCM_CREATE_HAL_RESOURCE` and the second passes `CY_VCM_USE_HAL_RESOURCE`. Ensure all PDL initialization is complete before calling `cy_vcm_init()`.
- **Virtual APIs fail or return errors:** Confirm `ENABLE_MULTICORE_CONN_MW` is defined on both cores, `USE_VIRTUAL_API` is defined on the secondary core, and (for MQTT) `VCM_ENABLE_MQTT` is defined on both cores.
- **IPC queue conflicts:** Do not use queue numbers 1-7 on the channel configured for VCM; they are reserved by VCM.
- **Event callbacks are dropped:** Increase `CY_VCM_EVENT_QUEUE_SIZE` on the core that initializes VCM with `CY_VCM_CREATE_HAL_RESOURCE`.
- **PSOC&trade; Edge E84 initialization fails:** Ensure the `cy_vcm_init()` channel number differs from the `mtb_ipc_init()` channel and that a valid IPC object is passed in `config.ipc_obj`.

# Related Code Examples

- Browse the [ModusToolbox&trade; code examples](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) and filter for multi-core connectivity examples (for example, multi-core Wi-Fi/MQTT designs) that use the Virtual Connectivity Manager for cross-core virtual API access.

# Configuration Reference

VCM is configured through compile-time `DEFINES` and the `cy_vcm_config_t` structure passed to `cy_vcm_init()`.

| Define | Applies to | Purpose |
| :--- | :--- | :--- |
| `ENABLE_MULTICORE_CONN_MW` | Both cores | Enables multi-core connectivity middleware support. |
| `USE_VIRTUAL_API` | Secondary core | Selects the virtual-only API subset on the secondary core. |
| `VCM_ENABLE_MQTT` | Both cores | Enables the MQTT virtual APIs. |
| `ENABLE_VCM_LOGS` | Either core | Enables VCM debug log messages. |
| `CY_VCM_EVENT_QUEUE_SIZE` | Create-resource core | Sets the event-callback queue depth (default 20). |

For the full parameter and structure reference, see the [Virtual Connectivity Manager Reference Manual](https://infineon.github.io/virtual-connectivity-manager/api_reference_manual/html/index.html).

# Release Notes and Changelog

- [RELEASE.md](./RELEASE.md) - Detailed release notes and changelog for all versions
- [version.xml](./version.xml) - Current library version

# License

This library is provided under Infineon software license terms. It works alongside the connectivity libraries whose virtual APIs it exposes (for example, Wi-Fi Connection Manager and MQTT) and the `connectivity-utilities` library, which are governed by their respective license terms. You are responsible for reviewing and accepting all applicable third-party license terms.

- [LICENSE](./LICENSE.txt)

---

# Copyright

(c) 2026, Infineon Technologies AG, or an affiliate of Infineon
Technologies AG.  All rights reserved.
