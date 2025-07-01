---
share: true
---
[test](../Firmware.md)

## **1. Architecture & Design Philosophy**

### **ESP32 (Bluedroid)**
- **Monolithic Design**: Single integrated stack with tight coupling
- **Centralized Control**: One main Bluetooth task controller (`btc_init`)
- **Dynamic Configuration**: Runtime configuration through structures
- **High-Level APIs**: Focus on ease of use with abstracted interfaces

### **Telink SDK**
- **Layered Architecture**: Modular design with clear separation of concerns
- **Distributed Control**: Multiple independent layers (tlkapp, stack/ble, tlkapi, drivers)
- **Compile-time Configuration**: Uses `#define` macros for configuration
- **Low-Level Control**: Direct hardware access and fine-grained control

## **2. Stack Organization**

### **ESP32 Stack Layers**
```
Application Layer (Arduino BLE Library)
         ↓
ESP-IDF Bluetooth API (Bluedroid)
         ↓
Controller Layer (ESP32 Hardware)
```

### **Telink Stack Layers**
```
tlkapp (Application Layer)
         ↓
stack/ble (Protocol Stack)
         ↓
tlkapi (API Layer)
         ↓
drivers (Hardware Layer)
```

## **3. Memory Management**

### **ESP32**
- **Dynamic Allocation**: Uses `osi_malloc()`, `osi_calloc()` for runtime memory
- **Future System**: Uses futures for asynchronous operation synchronization
- **Centralized Memory**: Single memory pool managed by the stack

### **Telink**
- **Static Allocation**: Pre-allocated memory pools and buffers
- **Event-Based**: Uses event tables and semaphores for synchronization
- **Distributed Memory**: Each layer manages its own memory

## **4. Task Management**

### **ESP32**
- **Single Bluetooth Task**: `btc_thread` handles all Bluetooth operations
- **Work Queue System**: Uses work queues for message processing
- **Centralized Scheduling**: All Bluetooth operations go through one task

### **Telink**
- **Multiple Tasks**: Separate tasks for different functions (`tlksys_task_create`)
- **Message Queue System**: Individual message queues per task
- **Distributed Scheduling**: Each layer can have its own task

## **5. Configuration Approach**

### **ESP32**
```c
esp_bluedroid_config_t cfg = {
    .mode = ESP_BT_MODE_BLE,
    .controller_task_stack_size = 4096,
    .controller_task_prio = 5,
    .sc_en = true,
    .ssp_en = true
};
esp_bluedroid_init_with_cfg(&cfg);
```

### **Telink**
```c
// Compile-time configuration
#define TLK_STK_BT_ENABLE          1
#define BLE_MAX_CONNECTION         4
#define GATT_MTU_SIZE             23
#define BLE_SECURITY_ENABLE        1
```

## **6. GATT Database Management**

### **ESP32**
- **Dynamic Creation**: Services and characteristics created at runtime
- **Object-Oriented**: Uses C++ classes for GATT elements
- **Automatic Management**: Stack handles database organization

### **Telink**
- **Static Definition**: GATT database defined at compile time
- **Structure-Based**: Uses C structures for GATT elements
- **Manual Management**: Developer must define complete database structure

## **7. Event Handling**

### **ESP32**
```c
esp_ble_gap_register_callback(gap_event_handler);
esp_ble_gatts_register_callback(gatts_event_handler);
```

### **Telink**
```c
blc_gap_register_event_callback(gap_event_handler);
blc_gatt_register_server_callback(gatts_event_handler);
```

## **8. Transport Layer**

### **ESP32**
- **HCI over UART/SPI**: Standard HCI transport
- **Internal Communication**: Direct function calls within SoC

### **Telink**
- **H0TL (Host-0-Transport-Layer)**: Custom transport for SoC mode
- **FIFO-based Communication**: Uses FIFOs for host-controller communication

## **9. Security Features**

### **ESP32**
- **Runtime Security Configuration**: Secure Connections, Simple Secure Pairing
- **Dynamic Key Management**: Runtime encryption key generation

### **Telink**
- **Compile-time Security**: Security features configured at build time
- **Static Key Storage**: Keys stored in flash/OTP areas

## **10. Development Approach**

### **ESP32**
- **High-Level Development**: Focus on rapid prototyping
- **Arduino Compatibility**: Easy migration from Arduino BLE library
- **Automatic Optimization**: Stack handles performance optimization

### **Telink**
- **Low-Level Development**: Focus on performance and control
- **Hardware-Specific**: Optimized for Telink chips
- **Manual Optimization**: Developer controls performance aspects

## **Summary**

The **ESP32 stack prioritizes ease of use and rapid development**, while the **Telink stack prioritizes performance, control, and hardware optimization**. ESP32 is better for quick prototyping and Arduino-style development, while Telink is better for production systems requiring fine-grained control and optimization.

```plaintext
Application Layer (Arduino BLE Library)
         ↓
ESP-IDF Bluetooth API (Bluedroid)
         ↓
Controller Layer (ESP32 Hardware)
```

```plaintext
tlkapp (Application Layer)
         ↓
stack/ble (Protocol Stack)
         ↓
tlkapi (API Layer)
         ↓
drivers (Hardware Layer)
```

```c
esp_bluedroid_config_t cfg = {
    .mode = ESP_BT_MODE_BLE,
    .controller_task_stack_size = 4096,
    .controller_task_prio = 5,
    .sc_en = true,
    .ssp_en = true
};
esp_bluedroid_init_with_cfg(&cfg);
```

```c
// Compile-time configuration
#define TLK_STK_BT_ENABLE          1
#define BLE_MAX_CONNECTION         4
#define GATT_MTU_SIZE             23
#define BLE_SECURITY_ENABLE        1
```

```c
esp_ble_gap_register_callback(gap_event_handler);
esp_ble_gatts_register_callback(gatts_event_handler);
```

```c
blc_gap_register_event_callback(gap_event_handler);
blc_gatt_register_server_callback(gatts_event_handler);
```


###### nvs (non-volatile storage)
```cpp
#ifdef ARDUINO_ARCH_ESP32
    if (!btStart()) { //[[Arduino BLE Libraries#btStart()|btStart()]]
      errRc = ESP_FAIL; //[[ESP-IDF#esp_err_t|esp_err_t]] errRc; #define ESP_FAIL        -1
      return;
    }
#else
    errRc = ::nvs_flash_init(); //[[ESP-IDF#nvs_flash_init|nvs_flash-init]]
    if (errRc != ESP_OK) { //#define ESP_OK 0
      log_e("nvs_flash_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }
```
Espressif has branded their flash storage as "NVS"
Bluetooth needs flash memory for pairing information etc.

creates a partition

###### Memory Allocation
1. Different Architecture: Telink uses static memory allocation vs ESP32's dynamic approach

2. Memory Model: Telink reserves memory permanently for Bluetooth operations

3. Resource Management: No need to release memory since it's always allocated

The Telink SDK doesn't have a memory release function because it uses a different memory management strategy.

###### Controller Init
```c
void controller_init(controller_mode mode, HCI_TR_MODE tr_mode, HCI_TR_UART *hci_tr_uart,  void *parameter);
```

###### Controller Enable
Telink combines initialization and enable into a single step
While Telink doesn't have a direct disable() function, it does have:

- Power management functions (rwip_sleep(), pm_set_dig_module_power_switch())

- Deinitialization patterns in some applications

- Resource cleanup mechanisms
###### Bluetooth stack init
```c
void tlkstk_init(void)

{

#if (TLK_STK_BT_ENABLE)

    tlkstk_mem_init();

    bth_init();

    btp_init();

#endif

}
```
###### Bluetooth stack configs
- Compile-time Configuration: Uses #define macros instead of runtime config
###### Task Controller
```c
int tlksys_task_create(uint08 taskID, const tlksys_task_cfg_t *pCfgs)

{

    if (pCfgs == nullptr || taskID >= TLKSYS_TASKID_MAXNUM) {

        return -TLK_EPARAM;

    }

    tlksys_task_t *pTask = &sTlkSysTaskList[taskID];

    if(pTask->adapt.usage != TLKAPI_ADAPT_USAGE_NOMARL){

        return -TLK_EREPEAT;

    }

    pTask->pCfgs = (tlksys_task_cfg_t *)pCfgs;

    if (pCfgs->Init != nullptr && pCfgs->Init() != TLK_ENONE) {

        return -TLK_EINIT;

    }

    pTask->adapt.usage = TLKAPI_ADAPT_USAGE_FOR_MAIN_THREAD;

    tlkos_event_createTab(16,&sTlkTaskEvtTabHandles[taskID]);

    tlkos_msgq_create(&sTlkTaskMsgQHandles[taskID],128,16);

    tlkos_event_regDealCB(sTlkTaskEvtTabHandles[taskID],0,tlksys_task_handleWakeUp);

    pTask->adapt.taskID = taskID;

    if(pCfgs->isUseNewThread == 0){

        return TLK_ENONE;

    }

    //following code: create new thread

    pTask->adapt.usage = TLKAPI_ADAPT_USAGE_FOR_INDEPENDENCE_THREAD;

    int ret = tlkos_task_create(tlksys_template_task,pCfgs->pTaskName,pCfgs->stackSize,pCfgs->priority,pTask,nullptr,&pTask->taskHandle);

    if(ret != TLK_ENONE){

        pTask->adapt.usage = TLKAPI_ADAPT_USAGE_FOR_MAIN_THREAD;

    }

    return TLK_ENONE;

}

void tlkstk_init(void)
{
#if (TLK_STK_BT_ENABLE)
    tlkstk_mem_init();
    bth_init();
    btp_init();
#endif
}

int tlkmdi_bt_init(void)
{
    tlkmdi_btadapt_init(TLKSYS_TASKID_BTMGR);

    #if (TLK_STK_BT_ENABLE)
    tlkmdi_btacl_init();           // ACL connection management
    tlkmdi_btScan_process_init();  // Scan process initialization
    #endif
    
    #if (TLK_MW_BTINQ_ENABLE)
    tlkmdi_btinq_init();           // Inquiry functionality
    #endif
    
    #if (TLKBTP_CFG_ATTSRV_ENABLE)
    tlkmdi_btatt_init();           // ATT server
    #endif
    
    #if (TLKBTP_CFG_HID_ENABLE)
    tlkmdi_bthid_init();           // HID profile
    #endif
    
    #if (TLKBTP_CFG_HFP_ENABLE)
    tlkmdi_bthfp_init();           // HFP profile
    #endif

    #if (TLKBTP_CFG_A2DP_ENABLE)
    tlkmdi_bta2dp_init();          // A2DP profile
    #endif
    
    #if (TLKBTP_CFG_HFP_ENABLE)
    tlkmdi_btsco_init();           // SCO connection
    #endif
    
    tlkmdi_bt_mgr_ctrlInit();      // Manager control initialization
    
    #if (TLKBTP_CFG_PBAP_ENABLE)
    tlkmdi_pbap_init();            // PBAP profile
    #endif
    
    #if (TLKBTP_CFG_IAP_ENABLE)
    tlkmdi_btiap_init();           // IAP profile
    #endif

    tlkmdi_bt_sppTestInit();       // SPP test initialization
    return TLK_ENONE;
}
```
###### Future pointer
- Purpose: Returns a pointer to a specific future object from an array

- Architecture: Uses futures for asynchronous operation synchronization

- Use Case: Coordinates between different Bluetooth tasks and operations

Telink SDK Approach:

- No Future System: Telink doesn't use futures for task synchronization

- Event-Based: Uses event tables and semaphores for task coordination

- Direct Communication: Tasks communicate directly through message queues and events
###### tlkos_semphr_createBinary
```c
int tlkos_semphr_createBinary(TlkOsSemphrHandle_t *semphrHandle)
{
    if (semphrHandle == nullptr) {
        return -TLK_EPARAM;
    }
    *semphrHandle = xSemaphoreCreateBinary();
    return TLK_ENONE;
}
```
###### tlksys_task_input
```c
int tlksys_task_input(uint16 taskID, uint16 msgID, uint08 *pData, uint16 dataLen)

{

    if (taskID >= TLKSYS_TASKID_MAXNUM || sTlkSysTaskList[taskID].adapt.usage == 0) {

        return -TLK_EPARAM;

    }

    if (sTlkSysTaskList[taskID].pCfgs->Input == nullptr) {

        return -TLK_ENOSUPPORT;

    }

    if(sTlkSysTaskList[taskID].adapt.usage != TLKAPI_ADAPT_USAGE_FOR_INDEPENDENCE_THREAD){

        return sTlkSysTaskList[taskID].pCfgs->Input(msgID, pData, dataLen);

    }

    if(tlkos_task_getRunningTask() == sTlkSysTaskList[taskID].taskHandle){

        return sTlkSysTaskList[taskID].pCfgs->Input(msgID, pData, dataLen);

    }

    //to other thread:

    uint08 buffer[dataLen + 4];

    buffer[0] = (msgID & 0xFF);

    buffer[1] = (msgID & 0xFF00) >> 8;

    buffer[2] = (dataLen & 0xFF);

    buffer[3] = (dataLen & 0xFF00) >> 8;

    tmemcpy(buffer + 4,pData,dataLen);

    int res = tlkos_msgq_send(sTlkTaskMsgQHandles[taskID],buffer, dataLen + 4,TLKOS_WAIT_FOREVER);

    if(res == TLK_ENONE){

        tlksys_task_wakeUp(taskID);

    }

    return res;

}
```
###### tlkapp_enable_bt_stack
```c
void tlkapp_enable_bt_stack(void)

{

    pm_set_dig_module_power_switch(FLD_PD_ZB_EN, PM_POWER_UP);

    user_init();

    delay_us(500);

    // PLL_264M_D25F_DSP_132M_HCLK_66M_PCLK_66M_MSPI_44M_WT_11M;

    app_btmgr_reStart();

}
```
###### blc_svc_setDeviceName
```c
void blc_svc_setDeviceName(const char *name);
```
###### smp security
```c
/**

 * @brief      This function is used to set device's security parameters.

 * @param[in]  mode - The bonding mode value can refer to the structure 'bonding_mode_t'.

 * @param[in]  MITM_en - 0: Disable MITM protection;  1: Enable MITM protection.

 * @param[in]  method - 0: LE_Legacy_Pairing; 1: LE_Secure_Connection.

 * @param[in]  OOB_en - 0: Disable OOB authentication; 1: Enable OOB authentication.

 * @param[in]  keyPress_en - 0: Disable Keypress; 1: Enable Keypress.

 * @param[in]  ioCapability - The IO capability's value can refer to the structure 'io_capability_t'.

 * @return     none.

 */

void blc_smp_setSecurityParameters(bonding_mode_t mode, int MITM_en, pairing_methods_t method, int OOB_en, int keyPress_en, io_capability_t ioCapability);

void blc_smp_setSecurityParameters_central(bonding_mode_t bond_mode, int MITM_en, pairing_methods_t method, int OOB_en, int keyPress_en, io_capability_t ioCapability);

void blc_smp_setSecurityParameters_periphr(bonding_mode_t bond_mode, int MITM_en, pairing_methods_t method, int OOB_en, int keyPress_en, io_capability_t ioCapability);
```
###### Register Server App
```c
void blc_prf_register_service_module(struct ble_prf_process *p_module, const void *param);
```

###### Create Service
```c
extern void blc_gatts_addAttributeServiceGroup_old(atts_group_t *pGroup);
#define blc_gatts_addAttributeServiceGroup(pGroup)  blc_gatts_addAttributeServiceGroup_old((atts_group_t *)pGroup)
```
###### Attribute Values
```c
/**

 * @brief       Register ATT table.

 * @param[in]   p - Pointer point to attribute table.

 * @return[in]  0: success

 *              other: fail

 */

void bls_att_setAttributeTable(u8 *p);

typedef struct __attribute__((packed)) attribute
{
    u16                      attNum;
    u8                       perm;
    u8                       uuidLen;
    u32                      attrLen; //4 bytes aligned
    u8                      *uuid;
    u8                      *pAttrValue;
    att_readwrite_callback_t w;
    att_readwrite_callback_t r;
} attribute_t;
```
There is no direct equivalent to esp_ble_gatts_set_attr_value() in the current Telink SDK. To achieve the same functionality, you would need to:

1. Directly modify the attribute value pointer (pAttrValue) in your attribute table

2. Implement a custom function similar to what was referenced in your translation document

3. Use the write callback mechanism to handle dynamic value changes

The Telink SDK uses a more static, compile-time approach to attribute management compared to ESP32's dynamic runtime approach.

###### GATT Start Service
1. Different Architecture:

- ESP32: Uses a dynamic service creation model where services are created and then "started" to make them available

- Telink: Uses a static attribute table approach where services are automatically available once registered

1. Service Registration Process:

- ESP32: esp_ble_gatts_create_service() → esp_ble_gatts_start_service()

- Telink: Services are immediately available when registered via bls_att_setAttributeTable()

###### Add Characteristic
The Telink SDK uses a static attribute table that defines all services and characteristics at compile time. Here's how it works:

###### GATTS Notify
```c
/**

 *   @brief  Send a notification message to a client.

 *

 *   @param[in]   conn_handle  ACL Connection handle.

 *   @param[in]   attr_handle  Attribute handle.

 *   @param[in]   value        Pointer to the value to be notified.

 *   @param[in]   len          Length of the value to be notified.

 *

 *   @return BLE_HOST_ERR_SUCC if the handle value notification is sent successfully.

 */

int ble_gatts_notify(uint16_t conn_handle, uint16_t attr_handle, const uint8_t *value, uint16_t len);

/**
 *   @brief  Sends an ATT Handle Value Indication(ATT channel) PDU with the given parameters.
 *
 *   @param[in] conn_handle: The connection handle.
 *   @param[in] handle: The handle to indicate attribute value.
 *   @param[in] value: The attribute value to be indicated.
 *   @param[in] length: The size of the attribute value.
 *
 *   @return BLE_HOST_ERR_SUCC if the handle value indication is sent successfully.
 *      - BLE_L2CAP_ERR_ATT_INVALID_HANDLE if the handle is invalid.
 *      - BLE_L2CAP_ERR_INVALID_PARAMS if the value is NULL and length is not 0.
 */
int ble_host_att_send_handle_value_indication(uint16_t conn_handle, uint16_t handle,
    const uint8_t *value, uint16_t length);
```