---
share: true
---

###### UUID type
```c
typedef struct {
#define ESP_UUID_LEN_16     2
#define ESP_UUID_LEN_32     4
#define ESP_UUID_LEN_128    16
    uint16_t len;							/* linked UUID length, 16bit, 32bit or 128bit */
    union {
        uint16_t    uuid16;                 /* linked 16bit UUID */
        uint32_t    uuid32;                 /* linked 32bit UUID */
        uint8_t     uuid128[ESP_UUID_LEN_128]; /* linked 128bit UUID */
    } uuid;									/* linked UUID */
} __attribute__((packed)) esp_bt_uuid_t;
```
###### esp_err_t
```c
typedef int esp_err_t;
```
###### nvs_flash_init
[nvs_sec_cfg_t](ESP-IDF.md/#nvs_sec_cfg_t)  
[nvs_flash_read_security_cfg_v2](ESP-IDF.md/#nvs_flash_read_security_cfg_v2)  
[nvs_sec_scheme_t](ESP-IDF.md/#nvs_sec_scheme_t)  
[nvs_flash_generate_keys_v2](ESP-IDF.md/#nvs_flash_generate_keys_v2)  
[nvs_flash_secure_init](ESP-IDF.md/#nvs_flash_secure_init)  
[nvs_flash_init_partition](ESP-IDF.md/#nvs_flash_init_partition)

This API initialises the default NVS partition. The default NVS partition is the one that is labeled "nvs" in the partition table.

When "NVS_ENCRYPTION" is enabled in the menuconfig, this API enables the NVS encryption for the default NVS partition as follows

1. Read security configurations from the first NVS key partition listed in the partition table. (NVS key partition is any "data" type partition which has the subtype value set to "nvs_keys")
    
2. If the NVS key partition obtained in the previous step is empty, generate and store new keys in that NVS key partition.
    
3. Internally call "nvs_flash_secure_init()" with the security configurations obtained/generated in the previous steps.
```c
extern "C" esp_err_t nvs_flash_init(void)
{
#ifdef CONFIG_NVS_ENCRYPTION
    esp_err_t ret = ESP_FAIL;
    nvs_sec_cfg_t cfg = {}; //[nvs_sec_cfg_t](ESP-IDF.md/#nvs_sec_cfg_t)

    ret = nvs_flash_read_security_cfg_v2(&nvs_sec_default_scheme_cfg, &cfg);//[nvs_flash_read_security_cfg_v2](ESP-IDF.md/#nvs_flash_read_security_cfg_v2) and static [nvs_sec_scheme_t](ESP-IDF.md/#nvs_sec_scheme_t) nvs_sec_default_scheme_cfg;
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read NVS security cfg: [0x%02X] (%s)", ret, esp_err_to_name(ret));
        ESP_LOGI(TAG, "Generating NVS encr-keys...");
        ret = nvs_flash_generate_keys_v2(&nvs_sec_default_scheme_cfg, &cfg); //[nvs_flash_generate_keys_v2](ESP-IDF.md/#nvs_flash_generate_keys_v2)
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to generate NVS encr-keys: [0x%02X] (%s)", ret, esp_err_to_name(ret));
            return ret;
        }
    }

    ret = nvs_flash_secure_init(&cfg); //[nvs_flash_secure_init](ESP-IDF.md/#nvs_flash_secure_init)
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "Failed to initialize NVS partition: [0x%02X] (%s)", ret, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "NVS partition \"%s\" is encrypted.", NVS_DEFAULT_PART_NAME);
    return ret;
#else // CONFIG_NVS_ENCRYPTION
    return nvs_flash_init_partition(NVS_DEFAULT_PART_NAME); //[nvs_flash_init_partition](ESP-IDF.md/#nvs_flash_init_partition)
#endif
}
```
###### nvs_sec_cfg_t
```c
typedef struct {
    uint8_t eky[NVS_KEY_SIZE]; /* linked  XTS encryption and decryption key*/
    uint8_t tky[NVS_KEY_SIZE]; /* linked  XTS tweak key */
} nvs_sec_cfg_t;
```
###### nvs_sec_scheme_t
```c
typedef struct
{
    int scheme_id;                                /* linked  Security Scheme ID (E.g. HMAC) */
    void *scheme_data;                            /* linked  Scheme-specific data (E.g. eFuse block for HMAC-based key generation) */
    nvs_flash_generate_keys_t nvs_flash_key_gen;  /* linked  Callback for the nvs_flash_key_gen implementation */
    nvs_flash_read_cfg_t nvs_flash_read_cfg;      /* linked  Callback for the nvs_flash_read_keys implementation */
} nvs_sec_scheme_t;
```
###### nvs_flash_init_partition
[Lock-init](ESP-IDF.md/#lockinit)  
[NVSPartitionManager-get_instance](ESP-IDF.md/#nvspartitionmanagerget_instance)  
[NVSPartitionManager-init_partition](ESP-IDF.md/#nvspartitionmanagerinit_partition)
```c
extern "C" esp_err_t nvs_flash_init_partition(const char *part_name)
{
    esp_err_t lock_result = Lock::init(); //[Lock-init](ESP-IDF.md/#lock-init)
    if (lock_result != ESP_OK) {
        return lock_result;
    }
    Lock lock;

    assert(nvs::Page::SEC_SIZE == esp_partition_get_main_flash_sector_size());
    return NVSPartitionManager::get_instance()->init_partition(part_name); //[NVSPartitionManager-get_instance](ESP-IDF.md/#nvspartitionmanager-get_instance) and [NVSPartitionManager-init_partition](ESP-IDF.md/#nvspartitionmanager-init_partition)
}
```
###### NVSPartitionManager::init_partition
[esp_partition_get_main_flash_sector_size](ESP-IDF.md/#esp_partition_get_main_flash_sector_size)  
[lookup_storage_from_name](ESP-IDF.md/#lookup_storage_from_name)  
[NVSPartition](ESP-IDF.md/#nvspartition)
```cpp
esp_err_t NVSPartitionManager::init_partition(const char *partition_label)
{
    if (strlen(partition_label) > NVS_PART_NAME_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t size;
    const uint32_t sec_size = esp_partition_get_main_flash_sector_size(); //[esp_partition_get_main_flash_sector_size](ESP-IDF.md/#esp_partition_get_main_flash_sector_size)
    Storage* mStorage;

    mStorage = lookup_storage_from_name(partition_label); //[lookup_storage_from_name](ESP-IDF.md/#lookup_storage_from_name)
    if (mStorage) {
        return ESP_OK;
    }

    NVS_ASSERT_OR_RETURN(sec_size != 0, ESP_FAIL);

    NVSPartition *p = nullptr; //[NVSPartition](ESP-IDF.md/#nvspartition)
    esp_err_t result = partition_lookup::lookup_nvs_partition(partition_label, &p);

    if (result != ESP_OK) {
        goto error;
    }

    size = p->get_size();

    result = init_custom(p, 0, size / sec_size);
    if (result != ESP_OK) {
        goto error;
    }

    nvs_partition_list.push_back(p);

    return ESP_OK;

error:
    delete p;
    return result;
}
```
###### NVSPartition
[esp_partition_t](ESP-IDF.md/#esp_partition_t)
```cpp
NVSPartition::NVSPartition(const esp_partition_t* partition)
	    : mESPPartition(partition) //const [esp_partition_t](ESP-IDF.md/#esp_partition_t)* mESPPartition;
{
    // ensure the class is in a valid state
    if (partition == nullptr) {
        std::abort();
    }
}
```
###### esp_partition_t
```c
typedef struct {
    esp_flash_t* flash_chip;            /* linked SPI flash chip on which the partition resides */
    esp_partition_type_t type;          /* linked partition type (app/data) */
    esp_partition_subtype_t subtype;    /* linked partition subtype */
    uint32_t address;                   /* linked starting address of the partition in flash */
    uint32_t size;                      /* linked size of the partition, in bytes */
    uint32_t erase_size;                /* linked size the erase operation should be aligned to */
    char label[17];                     /* linked partition label, zero-terminated ASCII string */
    bool encrypted;                     /* linked flag is set to true if partition is encrypted */
    bool readonly;                      /* linked flag is set to true if partition is read-only */
} esp_partition_t;
```
###### lookup_storage_from_name
```cpp
static nvs::Storage* lookup_storage_from_name(const char *name)
{
    return NVSPartitionManager::get_instance()->lookup_storage_from_name(name);
}
```
###### esp_partition_get_main_flash_sector_size
```c
uint32_t esp_partition_get_main_flash_sector_size(void)
{
    return SPI_FLASH_SEC_SIZE;
}
```
###### NVSPartitionManager::get_instance
```cpp
NVSPartitionManager* NVSPartitionManager::get_instance()
{
    if (!instance) {
        instance = new (std::nothrow) NVSPartitionManager();
    }

    return instance;
}
```
###### Lock::init
```cpp
esp_err_t Lock::init()
{
    // Let postpone initialization to the Lock::Lock.
    // It is designed to lazy initialize the semaphore in a properly guarded critical section
    return ESP_OK;
}
```
###### nvs_flash_secure_init
```c
extern "C" esp_err_t nvs_flash_secure_init_partition(const char *part_name, nvs_sec_cfg_t* cfg)
{
    esp_err_t lock_result = Lock::init();
    if (lock_result != ESP_OK) {
        return lock_result;
    }
    Lock lock;

    assert(nvs::Page::SEC_SIZE == esp_partition_get_main_flash_sector_size());
    return NVSPartitionManager::get_instance()->secure_init_partition(part_name, cfg);
}

extern "C" esp_err_t nvs_flash_secure_init(nvs_sec_cfg_t* cfg)
{
    return nvs_flash_secure_init_partition(NVS_DEFAULT_PART_NAME, cfg);
}
```
###### nvs_flash_generate_keys_v2
```c
extern "C" esp_err_t nvs_flash_generate_keys_v2(nvs_sec_scheme_t *scheme_cfg, nvs_sec_cfg_t* cfg)
{
    if (scheme_cfg == nullptr || cfg == nullptr || scheme_cfg->nvs_flash_key_gen == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return (scheme_cfg->nvs_flash_key_gen)(scheme_cfg->scheme_data, cfg);
}
```
###### nvs_flash_read_security_cfg_v2
```c
extern "C" esp_err_t nvs_flash_read_security_cfg_v2(nvs_sec_scheme_t *scheme_cfg, nvs_sec_cfg_t* cfg)
{
    if (scheme_cfg == nullptr || cfg == nullptr || scheme_cfg->nvs_flash_read_cfg == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return (scheme_cfg->nvs_flash_read_cfg)(scheme_cfg->scheme_data, cfg);
}
```
###### nvs_flash_init_partition
```c
extern "C" esp_err_t nvs_flash_init_partition(const char *part_name)
{
    esp_err_t lock_result = Lock::init();
    if (lock_result != ESP_OK) {
        return lock_result;
    }
    Lock lock;

    assert(nvs::Page::SEC_SIZE == esp_partition_get_main_flash_sector_size());
    return NVSPartitionManager::get_instance()->init_partition(part_name);
}
```
###### esp_bt_controller_mem_release
[Memory Allocation](Telink.md/#memory-allocation)
```c
esp_err_t esp_bt_controller_mem_release(esp_bt_mode_t mode) //[Memory Allocation](Telink.md/#memory-allocation)
{
    esp_err_t ret = ESP_OK;

    if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (mode == ESP_BT_MODE_BTDM) {
        bt_area_t cont_bss = {
            .start = (intptr_t)&_bt_controller_bss_start,
            .end   = (intptr_t)&_bt_controller_bss_end,
            .name  = "BT Controller BSS",
        };
        bt_area_t cont_data = {
            .start = (intptr_t)&_bt_controller_data_start,
            .end   = (intptr_t)&_bt_controller_data_end,
            .name  = "BT Controller Data"
        };

        ret = esp_bt_mem_release_areas(&cont_data, &cont_bss);
    }

    if (ret == ESP_OK) {
        ret = esp_bt_controller_rom_mem_release(mode);
    }

    return ret;
}
```
###### esp_bt_mode_t
```c
typedef enum {
    ESP_BT_MODE_IDLE       = 0x00,   /* linked Bluetooth is not operating. */
    ESP_BT_MODE_BLE        = 0x01,   /* linked Bluetooth is operating in BLE mode. */
    ESP_BT_MODE_CLASSIC_BT = 0x02,   /* linked Bluetooth is operating in Classic Bluetooth mode. */
    ESP_BT_MODE_BTDM       = 0x03,   /* linked Bluetooth is operating in Dual mode. */
} esp_bt_mode_t;
```
###### esp_bt_controller_config_t
```c
typedef struct {
    uint16_t controller_task_stack_size;    /* linked Bluetooth Controller task stack size in bytes */
    uint8_t controller_task_prio;           /* linked Bluetooth Controller task priority */
    uint8_t hci_uart_no;                    /* linked UART number as HCI I/O interface. Configurable in menuconfig.
                                                - 1 - URAT 1 (default)
                                                - 2 - URAT 2 */
    uint32_t hci_uart_baudrate;             /* linked  UART baudrate. Configurable in menuconfig.
                                                - Range: 115200 - 921600
                                                - Default: 921600 */
    uint8_t scan_duplicate_mode;            /* linked Scan duplicate filtering mode. Configurable in menuconfig.
                                                - 0 - Normal scan duplicate filtering mode (default)
                                                - 1 - Special scan duplicate filtering mode for BLE Mesh */
    uint8_t scan_duplicate_type;            /* linked Scan duplicate filtering type. If `scan_duplicate_mode` is set to 1, this parameter will be ignored. Configurable in menuconfig.
                                                - 0 - Filter scan duplicates by device address only (default)
                                                - 1 - Filter scan duplicates by advertising data only, even if they originate from different devices.
                                                - 2 - Filter scan duplicated by device address and advertising data. */
    uint16_t normal_adv_size;               /* linked Maximum number of devices in scan duplicate filtering list. Configurable in menuconfig
                                                - Range: 10 - 1000
                                                - Default: 100 */
    uint16_t mesh_adv_size;                 /* linked Maximum number of Mesh advertising packets in scan duplicate filtering list. Configurable in menuconfig
                                                - Range: 10 - 1000
                                                - Default: 100 */
    uint16_t send_adv_reserved_size;        /* linked Controller minimum memory value in bytes. Internal use only */
    uint32_t  controller_debug_flag;        /* linked Controller debug log flag. Internal use only */
    uint8_t mode;                           /* linked Controller mode.  Configurable in menuconfig
                                                - 1 - BLE mode
                                                - 2 - Classic Bluetooth mode
                                                - 3 - Dual mode
                                                - 4 - Others: Invalid */
    uint8_t ble_max_conn;                   /* linked Maximum number of BLE connections. Configurable in menuconfig
                                                - Range: 1 - 9
                                                - Default: 3 */
    uint8_t bt_max_acl_conn;                /* linked Maximum number of BR/EDR ACL connections. Configurable in menuconfig
                                                - Range: 1 - 7
                                                - Default: 2 */
    uint8_t bt_sco_datapath;                /* linked SCO data path. Configurable in menuconfig
                                                - 0 - HCI module (default)
                                                - 1 - PCM module */
    bool auto_latency;                      /* linked True if BLE auto latency is enabled, used to enhance Classic Bluetooth performance in the Dual mode; false otherwise (default). Configurable in menuconfig */
    bool bt_legacy_auth_vs_evt;             /* linked True if BR/EDR Legacy Authentication Vendor Specific Event is enabled (default in the classic bluetooth or Dual mode), which is required to protect from BIAS attack; false otherwise. Configurable in menuconfig */
    uint8_t bt_max_sync_conn;               /* linked Maximum number of BR/EDR synchronous connections. Configurable in menuconfig
                                                - Range: 0 - 3
                                                - Default: 0 */
    uint8_t ble_sca;                        /* linked BLE low power crystal accuracy index. Configurable in menuconfig
                                                - 0 - `BTDM_BLE_DEFAULT_SCA_500PPM`
                                                - 1 - `BTDM_BLE_DEFAULT_SCA_250PPM` (default) */
    uint8_t pcm_role;                       /* linked PCM role. Configurable in menuconfig
                                                - 0 - PCM master (default)
                                                - 1 - PCM slave (default) */
    uint8_t pcm_polar;                      /* linked PCM polarity (falling clk edge & rising clk edge). Configurable in menuconfig
                                                - 0 - Falling Edge (default)
                                                - 1 - Rising Edge */
    uint8_t pcm_fsyncshp;                   /* linked Physical shape of the PCM Frame Synchronization signal. Configurable in menuconfig
                                                - 0 - Stereo Mode (default)
                                                - 1 - Mono Mode 1
                                                - 2 - Mono Mode 2 */
    bool hli;                               /* linked True if using high-level (level 4) interrupt (default); false otherwise. Configurable in menuconfig */
    uint8_t enc_key_sz_min;                 /* linked Minimum size of the encryption key
                                                - Range: 7 - 16
                                                - Default: 7 */
    uint16_t dup_list_refresh_period;       /* linked Scan duplicate filtering list refresh period in seconds. Configurable in menuconfig
                                                - Range: 0 - 100 seconds
                                                - Default: 0 second */
    bool ble_scan_backoff;                  /* linked True if BLE scan backoff is enabled; false otherwise (default). Configurable in menuconfig */
    uint8_t ble_llcp_disc_flag;             /* linked Flag indicating whether the Controller disconnects after Instant Passed (0x28) error occurs. Configurable in menuconfig.
                                                - The Controller does not disconnect after Instant Passed (0x28) by default. */
    bool ble_aa_check;                      /* linked True if adds a verification step for the Access Address within the `CONNECT_IND` PDU; false otherwise (default). Configurable in menuconfig */
    uint8_t ble_chan_ass_en;                /* linked True if BLE channel assessment is enabled (default), false otherwise. Configurable in menuconfig */
    uint8_t ble_ping_en;                    /* linked True if BLE ping procedure is enabled (default), false otherwise. Configurable in menuconfig */
    uint32_t magic;                         /* linked Magic number */
} esp_bt_controller_config_t;
```
###### BT_CONTROLLER_INIT_CONFIG_DEFAULT
```c
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {                              \
    .controller_task_stack_size = ESP_TASK_BT_CONTROLLER_STACK,            \
    .controller_task_prio = ESP_TASK_BT_CONTROLLER_PRIO,                   \
    .hci_uart_no = BT_HCI_UART_NO_DEFAULT,                                 \
    .hci_uart_baudrate = BT_HCI_UART_BAUDRATE_DEFAULT,                     \
    .scan_duplicate_mode = SCAN_DUPLICATE_MODE,                            \
    .scan_duplicate_type = SCAN_DUPLICATE_TYPE_VALUE,                      \
    .normal_adv_size = NORMAL_SCAN_DUPLICATE_CACHE_SIZE,                   \
    .mesh_adv_size = MESH_DUPLICATE_SCAN_CACHE_SIZE,                       \
    .send_adv_reserved_size = SCAN_SEND_ADV_RESERVED_SIZE,                 \
    .controller_debug_flag = BTDM_CTRL_CONTROLLER_DEBUG_FLAG,              \
    .mode = BTDM_CONTROLLER_MODE_EFF,                                      \
    .ble_max_conn = CONFIG_BTDM_CTRL_BLE_MAX_CONN_EFF,                     \
    .bt_max_acl_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF,           \
    .bt_sco_datapath = CONFIG_BTDM_CTRL_BR_EDR_SCO_DATA_PATH_EFF,          \
    .auto_latency = BTDM_CTRL_AUTO_LATENCY_EFF,                            \
    .bt_legacy_auth_vs_evt = BTDM_CTRL_LEGACY_AUTH_VENDOR_EVT_EFF,         \
    .bt_max_sync_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF,         \
    .ble_sca = CONFIG_BTDM_BLE_SLEEP_CLOCK_ACCURACY_INDEX_EFF,             \
    .pcm_role = CONFIG_BTDM_CTRL_PCM_ROLE_EFF,                             \
    .pcm_polar = CONFIG_BTDM_CTRL_PCM_POLAR_EFF,                           \
    .pcm_fsyncshp = CONFIG_BTDM_CTRL_PCM_FSYNCSHP_EFF,                     \
    .hli = BTDM_CTRL_HLI,                                                  \
    .enc_key_sz_min = CONFIG_BTDM_CTRL_BR_EDR_MIN_ENC_KEY_SZ_DFT_EFF,      \
    .dup_list_refresh_period = SCAN_DUPL_CACHE_REFRESH_PERIOD,             \
    .ble_scan_backoff = BTDM_CTRL_SCAN_BACKOFF_UPPERLIMITMAX,              \
    .ble_llcp_disc_flag = BTDM_BLE_LLCP_DISC_FLAG,                         \
    .ble_aa_check = BTDM_CTRL_CHECK_CONNECT_IND_ACCESS_ADDRESS_ENABLED,    \
    .ble_chan_ass_en = BTDM_BLE_CHAN_ASS_EN,                               \
    .ble_ping_en = BTDM_BLE_PING_EN,                                       \
    .magic = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,                           \
}
```
###### esp_bt_controller_init
[Controller Init](Telink.md/#controller-init)
```c
esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg) //[Controller Init](Telink.md/#controller-init)
{
    esp_err_t err;
    uint32_t btdm_cfg_mask = 0;

#if CONFIG_BTDM_CTRL_HLI
    hli_queue_setup_pinned_to_core(CONFIG_BTDM_CTRL_PINNED_TO_CORE);
#endif /* CONFIG_BTDM_CTRL_HLI */

    //if all the bt available memory was already released, cannot initialize bluetooth controller
    if (btdm_dram_available_region[0].mode == ESP_BT_MODE_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    osi_funcs_p = (struct osi_funcs_t *)malloc_internal_wrapper(sizeof(struct osi_funcs_t));
    if (osi_funcs_p == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(osi_funcs_p, &osi_funcs_ro, sizeof(struct osi_funcs_t));
    if (btdm_osi_funcs_register(osi_funcs_p) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->controller_task_prio != ESP_TASK_BT_CONTROLLER_PRIO
            || cfg->controller_task_stack_size < ESP_TASK_BT_CONTROLLER_STACK) {
        return ESP_ERR_INVALID_ARG;
    }

    //overwrite some parameters
    cfg->bt_max_sync_conn = CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN_EFF;
    cfg->magic  = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL;

    if (((cfg->mode & ESP_BT_MODE_BLE) && (cfg->ble_max_conn <= 0 || cfg->ble_max_conn > BTDM_CONTROLLER_BLE_MAX_CONN_LIMIT))
            || ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) && (cfg->bt_max_acl_conn <= 0 || cfg->bt_max_acl_conn > BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN_LIMIT))
            || ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) && (cfg->bt_max_sync_conn > BTDM_CONTROLLER_BR_EDR_MAX_SYNC_CONN_LIMIT))) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(BTDM_LOG_TAG, "BT controller compile version [%s]", btdm_controller_get_compile_version());

    s_wakeup_req_sem = semphr_create_wrapper(1, 0);
    if (s_wakeup_req_sem == NULL) {
        err = ESP_ERR_NO_MEM;
        goto error;
    }

    esp_phy_modem_init();

    esp_bt_power_domain_on();

    btdm_controller_mem_init();

    periph_module_enable(PERIPH_BT_MODULE);
    periph_module_reset(PERIPH_BT_MODULE);

#if CONFIG_BTDM_CTRL_HCI_UART_FLOW_CTRL_EN
    sdk_config_set_uart_flow_ctrl_enable(true);
#else
    sdk_config_set_uart_flow_ctrl_enable(false);
#endif

    if ((err = btdm_low_power_mode_init()) != ESP_OK) {
        ESP_LOGE(BTDM_LOG_TAG, "Low power module initialization failed");
        goto error;
    }

#if CONFIG_SW_COEXIST_ENABLE
    coex_init();
#endif

#if CONFIG_BT_BLE_LOG_SPI_OUT_ENABLED
    if (ble_log_spi_out_init() != 0) {
        ESP_LOGE(BTDM_LOG_TAG, "BLE Log SPI output init failed");
        err = ESP_ERR_NO_MEM;
        goto error;
    }
#endif // CONFIG_BT_BLE_LOG_SPI_OUT_ENABLED

    btdm_cfg_mask = btdm_config_mask_load();

    err = btdm_controller_init(btdm_cfg_mask, cfg);

    if (err != 0) {
        ESP_LOGE(BTDM_LOG_TAG, "%s %d\n",__func__,err);
        err = ESP_ERR_NO_MEM;
        goto error;
    }

#ifdef CONFIG_BT_BLUEDROID_ENABLED
    bt_stack_enableSecCtrlVsCmd(true);
#endif // CONFIG_BT_BLUEDROID_ENABLED
#if defined(CONFIG_BT_NIMBLE_ENABLED) || defined(CONFIG_BT_BLUEDROID_ENABLED)
    bt_stack_enableCoexVsCmd(true);
    scan_stack_enableAdvFlowCtrlVsCmd(true);
    adv_stack_enableClearLegacyAdvVsCmd(true);
    advFilter_stack_enableDupExcListVsCmd(true);
#endif // (CONFIG_BT_NIMBLE_ENABLED) || (CONFIG_BT_BLUEDROID_ENABLED)

    btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

    return ESP_OK;

error:

#if CONFIG_BT_BLE_LOG_SPI_OUT_ENABLED
    ble_log_spi_out_deinit();
#endif // CONFIG_BT_BLE_LOG_SPI_OUT_ENABLED

    bt_controller_deinit_internal();

    return err;
}
```
###### esp_bt_controller_enable
```c
esp_err_t esp_bt_controller_enable(esp_bt_mode_t mode)
{
    int ret;

    if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED) {
        return ESP_ERR_INVALID_STATE;
    }

    //As the history reason, mode should be equal to the mode which set in esp_bt_controller_init()
    if (mode != btdm_controller_get_mode()) {
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_PM_ENABLE
    if (!s_btdm_allow_light_sleep) {
        esp_pm_lock_acquire(s_light_sleep_pm_lock);
    }
    esp_pm_lock_acquire(s_pm_lock);
#endif

    esp_phy_enable(PHY_MODEM_BT);

#if CONFIG_SW_COEXIST_ENABLE
    coex_enable();
#endif

    if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG) {
        btdm_controller_enable_sleep(true);
    }

    sdk_config_set_bt_pll_track_enable(true);

    // initialize bluetooth baseband
    btdm_check_and_init_bb();

    ret = btdm_controller_enable(mode);
    if (ret != 0) {
#if CONFIG_SW_COEXIST_ENABLE
        coex_disable();
#endif
        esp_phy_disable(PHY_MODEM_BT);
#ifdef CONFIG_PM_ENABLE
        if (!s_btdm_allow_light_sleep) {
            esp_pm_lock_release(s_light_sleep_pm_lock);
        }
        esp_pm_lock_release(s_pm_lock);
#endif
        return ESP_ERR_INVALID_STATE;
    }

    btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;
    ret = esp_register_shutdown_handler(bt_shutdown);
    if (ret != ESP_OK) {
        ESP_LOGW(BTDM_LOG_TAG, "Register shutdown handler failed, ret = 0x%x", ret);
    }

    return ESP_OK;
}
```
###### esp_bluedroid_status_t
```c
typedef enum {
    ESP_BLUEDROID_STATUS_UNINITIALIZED   = 0,        /* linked Bluetooth not initialized */
    ESP_BLUEDROID_STATUS_INITIALIZED,                /* linked Bluetooth initialized but not enabled */
    ESP_BLUEDROID_STATUS_ENABLED                     /* linked Bluetooth initialized and enabled */
} esp_bluedroid_status_t;
```
###### esp_bluedroid_get_status
```c
esp_bluedroid_status_t esp_bluedroid_get_status(void)
{
    if (bd_already_init) {
        if (bd_already_enable) {
            return ESP_BLUEDROID_STATUS_ENABLED;
        } else {
            return ESP_BLUEDROID_STATUS_INITIALIZED;
        }
    } else {
        return ESP_BLUEDROID_STATUS_UNINITIALIZED;
    }
}
```
###### esp_bluedroid_init
[Bluetooth stack init](Telink.md/#bluetooth-stack-init)  
[esp_bluedroid_config_t](ESP-IDF.md/#esp_bluedroid_config_t)  
[BT_BLUEDROID_INIT_CONFIG_DEFAULT](ESP-IDF.md/#bt_bluedroid_init_config_default)  
[esp_bluedroid_init_with_cfg](ESP-IDF.md/#esp_bluedroid_init_with_cfg)
```c
esp_err_t esp_bluedroid_init(void) //[Bluetooth stack init](Telink.md/#bluetooth-stack-init)
{
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT(); //[esp_bluedroid_config_t](ESP-IDF.md/#esp_bluedroid_config_t) and [BT_BLUEDROID_INIT_CONFIG_DEFAULT](ESP-IDF.md/#bt_bluedroid_init_config_default)
    return esp_bluedroid_init_with_cfg(&cfg); //[esp_bluedroid_init_with_cfg](ESP-IDF.md/#esp_bluedroid_init_with_cfg)
}
```
###### esp_bluedroid_config_t
```c
typedef struct {
    bool ssp_en; /* linked Whether SSP(secure simple pairing) or legacy pairing is used for Classic Bluetooth */
    bool sc_en; /* linked Whether secure connection host support is enabled or disabled for Classic Bluetooth */
} esp_bluedroid_config_t;
```
###### BT_BLUEDROID_INIT_CONFIG_DEFAULT
```c
#define BT_BLUEDROID_INIT_CONFIG_DEFAULT()                                                                             \
    {                                                                                                                  \
        .ssp_en = true,                                                                                                \
        .sc_en = false,                                                                                                \
    }
```
###### esp_bluedroid_init_with_cfg
[Bluetooth stack configs](Telink.md/#bluetooth-stack-configs)  
[btc_msg](ESP-IDF.md/#btc_msg)  
[bluedroid_config_init](ESP-IDF.md/#bluedroid_config_init)  
[btc_init](ESP-IDF.md/#btc_init)  
[btc_main_get_future_p](ESP-IDF.md/#btc_main_get_future_p)  
[future_new](ESP-IDF.md/#future_new)  
[btc_transfer_context](ESP-IDF.md/#btc_transfer_context)  
[future_await](ESP-IDF.md/#future_await)
```c
esp_err_t esp_bluedroid_init_with_cfg(esp_bluedroid_config_t *cfg) //[Bluetooth stack configs](Telink.md/#bluetooth-stack-configs)
{
    btc_msg_t msg; //[btc_msg](ESP-IDF.md/#btc_msg)
    future_t **future_p;
    bt_status_t ret;

    if (!cfg) {
        LOG_ERROR("%s cfg is NULL", __func__);
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->sc_en) {
#if (SC_MODE_INCLUDED == FALSE)
        LOG_ERROR("Secure Connections should not be enabled when target controller is ESP32.\n");
        LOG_ERROR("It may trigger unresolved bugs in the controller.\n");
        return ESP_ERR_INVALID_ARG;
#endif // SC_MODE_INCLUDED

        if (!cfg->ssp_en) {
            LOG_ERROR("secure simple pairing should be enabled when secure connection host support is enabled\n");
            return ESP_ERR_INVALID_ARG;
        }

        LOG_WARN("Please make sure to clear the bond list before enabling the secure connection host support\n");
    }

#if (BT_CONTROLLER_INCLUDED == TRUE)
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        LOG_ERROR("Controller not initialised\n");
        return ESP_ERR_INVALID_STATE;
    }
#endif

    if (bd_already_init) {
        LOG_ERROR("Bluedroid already initialised\n");
        return ESP_ERR_INVALID_STATE;
    }

#if HEAP_MEMORY_DEBUG
    osi_mem_dbg_init();
#endif

    ret = bluedroid_config_init(cfg); //[bluedroid_config_init](ESP-IDF.md/#bluedroid_config_init)
    if (ret != BT_STATUS_SUCCESS) {
        LOG_ERROR("Bluedroid stack initialize fail, ret:%d", ret);
        return ESP_FAIL;
    }

    /*
     * BTC Init
     */
    ret = btc_init(); //[btc_init](ESP-IDF.md/#btc_init)
    if (ret != BT_STATUS_SUCCESS) {
        LOG_ERROR("Bluedroid Initialize Fail");
        return ESP_FAIL;
    }

    future_p = btc_main_get_future_p(BTC_MAIN_INIT_FUTURE); //[btc_main_get_future_p](ESP-IDF.md/#btc_main_get_future_p)
    *future_p = future_new(); //[future_new](ESP-IDF.md/#future_new)
    if (*future_p == NULL) {
        LOG_ERROR("Bluedroid Initialize Fail!");
        return ESP_ERR_NO_MEM;
    }

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_MAIN_INIT;
    msg.act = BTC_MAIN_ACT_INIT;

    if (btc_transfer_context(&msg, NULL, 0, NULL, NULL) != BT_STATUS_SUCCESS) // [btc_transfer_context](ESP-IDF.md/#btc_transfer_context)
    {
        LOG_ERROR("Bluedroid Initialize Fail");
        return ESP_FAIL;
    }

    if (future_await(*future_p) == FUTURE_FAIL) { //[future_await](ESP-IDF.md/#future_await)
        LOG_ERROR("Bluedroid Initialize Fail");
        return ESP_FAIL;
    }

    bd_already_init = true;

#if (BT_HCI_LOG_INCLUDED == TRUE)
    bt_hci_log_init();
#endif // (BT_HCI_LOG_INCLUDED == TRUE)

    return ESP_OK;
}
```
###### bluedroid_config_init
```c
bt_status_t bluedroid_config_init(esp_bluedroid_config_t *cfg)
{
    s_stack_config_env = osi_calloc(sizeof(struct stack_config_env_tag));
    if (!s_stack_config_env) {
        return BT_STATUS_NOMEM;
    }

    memcpy(&s_stack_config_env->cfg, cfg, sizeof(esp_bluedroid_config_t));

    struct bluedroid_config *interface = &s_stack_config_env->interface;
    interface->get_ssp_enabled = get_ssp_enabled;
    interface->get_sc_enabled = get_sc_enabled;

    return BT_STATUS_SUCCESS;
}
```
###### btc_init
[Task Controller](Telink.md/#task-controller)
```c
bt_status_t btc_init(void) //[Task Controller](Telink.md/#task-controller)
{
    const size_t workqueue_len[] = {BTC_TASK_WORKQUEUE0_LEN, BTC_TASK_WORKQUEUE1_LEN};
    btc_thread = osi_thread_create(BTC_TASK_NAME, BTC_TASK_STACK_SIZE, BTC_TASK_PRIO, BTC_TASK_PINNED_TO_CORE,
                                   BTC_TASK_WORKQUEUE_NUM, workqueue_len);
    if (btc_thread == NULL) {
        return BT_STATUS_NOMEM;
    }

#if BTC_DYNAMIC_MEMORY
    if (btc_init_mem() != BT_STATUS_SUCCESS){
        return BT_STATUS_NOMEM;
    }
#endif
#if BTC_GAP_BT_INCLUDED
    btc_gap_bt_init();
#endif

#if (BLE_INCLUDED == TRUE)
    btc_gap_callback_init();
#if (BLE_FEAT_ISO_EN == TRUE)
    btc_iso_callback_init();
#endif // #if (BLE_FEAT_ISO_EN == TRUE)
#if (BLE_FEAT_CTE_EN == TRUE)
    btc_cte_callback_init();
#endif // #if (BLE_FEAT_CTE_EN == TRUE)
    btc_gap_ble_init();
#endif  ///BLE_INCLUDED == TRUE

    /* TODO: initial the profile_tab */
    return BT_STATUS_SUCCESS;
}
```
###### btc_main_get_future_p
[Future pointer](Telink.md/#future-pointer)
```c
future_t **btc_main_get_future_p(btc_main_future_type_t type) //[Future pointer](Telink.md/#future-pointer)
{
    return &main_future[type];
}
```
###### future_new
[tlkos_semphr_createBinary](Telink.md/#tlkos_semphr_createbinary)
```c
future_t *future_new(void) //[tlkos_semphr_createBinary](Telink.md/#tlkos_semphr_createbinary)
{
    future_t *ret = osi_calloc(sizeof(future_t));
    if (!ret) {
        OSI_TRACE_ERROR("%s unable to allocate memory for return value.", __func__);
        goto error;
    }

    if (osi_sem_new(&ret->semaphore, 1, 0) != 0) {
        OSI_TRACE_ERROR("%s unable to allocate memory for the semaphore.", __func__);
        goto error;
    }

    ret->ready_can_be_called = true;
    return ret;
error:;
    future_free(ret);
    return NULL;
}
```
###### btc_msg
```c
typedef struct btc_msg {
    uint8_t sig;    //event signal
    uint8_t aid;    //application id
    uint8_t pid;    //profile id
    uint8_t act;    //profile action, defined in seprerate header files
    UINT8   arg[0]; //param for btc function or function param
} btc_msg_t;

```
###### btc_transfer_context
[tlksys_task_input](Telink.md/#tlksys_task_input)
```c
bt_status_t btc_transfer_context(btc_msg_t *msg, void *arg, int arg_len, btc_arg_deep_copy_t copy_func,
                                    btc_arg_deep_free_t free_func) //[tlksys_task_input](ESP-IDF.md/#tlksys_task_input)
{
    btc_msg_t* lmsg;
    bt_status_t ret;
    //                              arg XOR arg_len
    if ((msg == NULL) || ((arg == NULL) == !(arg_len == 0))) {
        BTC_TRACE_WARNING("%s Invalid parameters\n", __func__);
        return BT_STATUS_PARM_INVALID;
    }

    BTC_TRACE_DEBUG("%s msg %u %u %u %p\n", __func__, msg->sig, msg->pid, msg->act, arg);

    lmsg = (btc_msg_t *)osi_malloc(sizeof(btc_msg_t) + arg_len);
    if (lmsg == NULL) {
        BTC_TRACE_WARNING("%s No memory\n", __func__);
        return BT_STATUS_NOMEM;
    }

    memcpy(lmsg, msg, sizeof(btc_msg_t));
    if (arg) {
        memset(lmsg->arg, 0x00, arg_len);    //important, avoid arg which have no length
        memcpy(lmsg->arg, arg, arg_len);
        if (copy_func) {
            copy_func(lmsg, lmsg->arg, arg);
        }
    }

    ret = btc_task_post(lmsg, OSI_THREAD_MAX_TIMEOUT);
    if (ret != BT_STATUS_SUCCESS) {
        if (copy_func && free_func) {
            free_func(lmsg);
        }
        osi_free(lmsg);
    }

    return ret;
}
```
###### future_await
```c
void *future_await(future_t *future)
{
    assert(future != NULL);

    // If the future is immediate, it will not have a semaphore
    if (future->semaphore) {
        osi_sem_take(&future->semaphore, OSI_SEM_MAX_TIMEOUT);
    }

    void *result = future->result;
    future_free(future);
    return result;
}
```
###### esp_bluedroid_enable
[tlkapp_enable_bt_stack](Telink.md/#tlkapp_enable_bt_stack)
```c
esp_err_t esp_bluedroid_enable(void) // [tlkapp_enable_bt_stack](Telink.md/#tlkapp_enable_bt_stack)
{
    btc_msg_t msg;
    future_t **future_p;

    if (!bd_already_init) {
        LOG_ERROR("Bludroid not initialised\n");
        return ESP_ERR_INVALID_STATE;
    }

    if (bd_already_enable) {
        LOG_ERROR("Bluedroid already enabled\n");
        return ESP_ERR_INVALID_STATE;
    }

    future_p = btc_main_get_future_p(BTC_MAIN_ENABLE_FUTURE);
    *future_p = future_new();
    if (*future_p == NULL) {
        LOG_ERROR("Bluedroid enable failed\n");
        return ESP_ERR_NO_MEM;
    }

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_MAIN_INIT;
    msg.act = BTC_MAIN_ACT_ENABLE;

    if (btc_transfer_context(&msg, NULL, 0, NULL, NULL) != BT_STATUS_SUCCESS) {
        LOG_ERROR("Bluedroid enable failed\n");
        return ESP_FAIL;
    }

    if (future_await(*future_p) == FUTURE_FAIL) {
        LOG_ERROR("Bluedroid enable failed\n");
        return ESP_FAIL;
    }

    bd_already_enable = true;

    return ESP_OK;
}
```
###### esp_ble_gap_register_callback
```c
esp_err_t esp_ble_gap_register_callback(esp_gap_ble_cb_t callback)
{
    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    return (btc_profile_cb_set(BTC_PID_GAP_BLE, callback) == 0 ? ESP_OK : ESP_FAIL);
}
```
###### esp_ble_gattc_register_callback
```c
esp_err_t esp_ble_gattc_register_callback(esp_gattc_cb_t callback)
{
    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    if (callback == NULL) {
        return ESP_FAIL;
    }

    btc_profile_cb_set(BTC_PID_GATTC, callback);
    return ESP_OK;
}
```
###### esp_ble_gap_set_device_name
[blc_svc_setDeviceName](Telink.md/#blc_svc_setdevicename)
```c
esp_err_t esp_ble_gap_set_device_name(const char *name) //[blc_svc_setDeviceName](Telink.md/#blc_svc_setdevicename)
{
    btc_msg_t msg = {0};
    btc_ble_gap_args_t arg;

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!name){
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(name) > BTC_MAX_LOC_BD_NAME_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GAP_BLE;
    msg.act = BTC_GAP_BLE_ACT_SET_DEV_NAME;
    arg.set_dev_name.device_name = (char *)name;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gap_args_t), btc_gap_ble_arg_deep_copy, btc_gap_ble_arg_deep_free) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_ble_gap_set_security_param
[smp security](Telink.md/#smp-security)
```c
esp_err_t esp_ble_gap_set_security_param(esp_ble_sm_param_t param_type,
        void *value, uint8_t len) //[smp security](Telink.md/#smp-security)
{
    if(param_type >= ESP_BLE_SM_MAX_PARAM) {
        return ESP_ERR_INVALID_ARG;
    }
    if((param_type != ESP_BLE_SM_CLEAR_STATIC_PASSKEY) && ( value == NULL || len < sizeof(uint8_t) || len > sizeof(uint32_t))) {
        return ESP_ERR_INVALID_ARG;
    }
    if(param_type == ESP_BLE_SM_SET_STATIC_PASSKEY) {
        uint32_t passkey = 0;
        for(uint8_t i = 0; i < len; i++)
        {
            passkey += (((uint8_t *)value)[i]<<(8*i));
        }
        if(passkey > 999999) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (param_type == ESP_BLE_APP_ENC_KEY_SIZE) {
        LOG_ERROR("ESP_BLE_APP_ENC_KEY_SIZE is deprecated, use ESP_GATT_PERM_ENCRYPT_KEY_SIZE in characteristic definition");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (param_type == ESP_BLE_SM_MAX_KEY_SIZE || param_type == ESP_BLE_SM_MIN_KEY_SIZE) {
        if (((uint8_t *)value)[0] > 16 || ((uint8_t *)value)[0] < 7) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    btc_msg_t msg = {0};
    btc_ble_gap_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GAP_BLE;
    msg.act = BTC_GAP_BLE_SET_SECURITY_PARAM_EVT;
    arg.set_security_param.param_type = param_type;
    arg.set_security_param.len = len;
    arg.set_security_param.value = value;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gap_args_t), btc_gap_ble_arg_deep_copy,
                btc_gap_ble_arg_deep_free) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}

```
###### esp_bt_controller_get_status
[btdm_controller_status](ESP-IDF.md/#btdm_controller_status)
```c
esp_bt_controller_status_t esp_bt_controller_get_status(void)
{
    return btdm_controller_status; //[btdm_controller_status](ESP-IDF.md/#btdm_controller_status)
}
```
###### btdm_controller_status
```c
static DRAM_ATTR esp_bt_controller_status_t btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;
```
###### esp_err_to_name
```c
const char *esp_err_to_name(esp_err_t code)
{
#ifdef CONFIG_ESP_ERR_TO_NAME_LOOKUP
    size_t i;

    for (i = 0; i < sizeof(esp_err_msg_table) / sizeof(esp_err_msg_table[0]); ++i) {
        if (esp_err_msg_table[i].code == code) {
            return esp_err_msg_table[i].msg;
        }
    }
#endif //CONFIG_ESP_ERR_TO_NAME_LOOKUP

    return esp_unknown_msg;
}
```
###### esp_ble_gatts_app_register
[Register Server App](Telink.md/#register-server-app)
```c
esp_err_t esp_ble_gatts_app_register(uint16_t app_id) //[Register Server App](Telink.md/#register-server-app)
{
    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    //if (app_id < ESP_APP_ID_MIN || app_id > ESP_APP_ID_MAX) {
    if (app_id > ESP_APP_ID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_APP_REGISTER;
    arg.app_reg.app_id = app_id;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), NULL, NULL) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_ble_gatts_create_service
[Create Service](Telink.md/#create-service)
```c
esp_err_t esp_ble_gatts_create_service(esp_gatt_if_t gatts_if,
                                       esp_gatt_srvc_id_t *service_id, uint16_t num_handle) //[Create Service](Telink.md/#create-service)
{
    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_CREATE_SERVICE;
    arg.create_srvc.gatts_if = gatts_if;
    arg.create_srvc.num_handle = num_handle;
    memcpy(&arg.create_srvc.service_id, service_id, sizeof(esp_gatt_srvc_id_t));

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), NULL, NULL) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_gatt_char_prop_t
```c
typedef uint8_t esp_gatt_char_prop_t;
```
###### esp_attr_value_t
```c
typedef struct
{
    uint16_t attr_max_len;                                  /* linked  attribute max value length */
    uint16_t attr_len;                                      /* linked  attribute current value length */
    uint8_t  *attr_value;                                   /* linked  the pointer to attribute value */
} esp_attr_value_t;
```
###### esp_ble_gatts_set_attr_value
[Attribute Values](Telink.md/#attribute-values)
```c
esp_err_t esp_ble_gatts_set_attr_value(uint16_t attr_handle, uint16_t length, const uint8_t *value) //[Attribute Values](Telink.md/#attribute-values)
{
    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_SET_ATTR_VALUE;
    arg.set_attr_val.handle = attr_handle;
    arg.set_attr_val.length = length;
    arg.set_attr_val.value  = (uint8_t *)value;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), btc_gatts_arg_deep_copy,
                btc_gatts_arg_deep_free) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_ble_gatts_start_service
[GATT Start Service](Telink.md/#gatt-start-service)
```cpp
esp_err_t esp_ble_gatts_start_service(uint16_t service_handle) //[GATT Start Service](Telink.md/#gatt-start-service)
{
    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_START_SERVICE;
    arg.start_srvc.service_handle = service_handle;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), NULL, NULL) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_attr_control_t
```c
typedef struct
{
    /**
     * @brief Controls who handles the response to Read/Write operations.
     *
     * - If set to @c ESP_GATT_RSP_BY_APP, the application is responsible for
     *   generating the response.
     * - If set to @c ESP_GATT_AUTO_RSP, the GATT stack will automatically generate
     *   the response.
     */
    uint8_t auto_rsp;
} esp_attr_control_t;
```
###### esp_gatt_perm_t
```cpp
typedef uint16_t esp_gatt_perm_t;
```
###### esp_ble_gatts_add_char_descr
[Add Characteristic](Telink.md/#add-characteristic)
```c
esp_err_t esp_ble_gatts_add_char_descr (uint16_t service_handle,
                                        esp_bt_uuid_t   *descr_uuid,
                                        esp_gatt_perm_t perm, esp_attr_value_t *char_descr_val,
                                        esp_attr_control_t *control) //[Add Characteristic](Telink.md/#add-characteristic)
{
    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;
    esp_err_t status;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    /* parameter validation check */
    status = esp_ble_gatts_add_char_desc_param_check(char_descr_val, control);
    if (status != ESP_OK){
        return status;
    }

    memset(&arg, 0, sizeof(btc_ble_gatts_args_t));
    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_ADD_CHAR_DESCR;
    arg.add_descr.service_handle = service_handle;
    arg.add_descr.perm = perm;

    if (char_descr_val != NULL) {
        arg.add_descr.descr_val.attr_max_len = char_descr_val->attr_max_len;
        arg.add_descr.descr_val.attr_len = char_descr_val->attr_len;
        arg.add_descr.descr_val.attr_value = char_descr_val->attr_value;
    }

    if (control != NULL) {
        arg.add_descr.attr_control.auto_rsp = control->auto_rsp;
    }
    memcpy(&arg.add_descr.descr_uuid, descr_uuid, sizeof(esp_bt_uuid_t));

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), btc_gatts_arg_deep_copy,
                btc_gatts_arg_deep_free) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_ble_adv_data_type
```c
typedef enum {
    ESP_BLE_AD_TYPE_FLAG                     = 0x01,    /* relate to BTM_BLE_AD_TYPE_FLAG in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_16SRV_PART               = 0x02,    /* relate to BTM_BLE_AD_TYPE_16SRV_PART in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_16SRV_CMPL               = 0x03,    /* relate to BTM_BLE_AD_TYPE_16SRV_CMPL in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_32SRV_PART               = 0x04,    /* relate to BTM_BLE_AD_TYPE_32SRV_PART in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_32SRV_CMPL               = 0x05,    /* relate to BTM_BLE_AD_TYPE_32SRV_CMPL in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_128SRV_PART              = 0x06,    /* relate to BTM_BLE_AD_TYPE_128SRV_PART in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_128SRV_CMPL              = 0x07,    /* relate to BTM_BLE_AD_TYPE_128SRV_CMPL in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_NAME_SHORT               = 0x08,    /* relate to BTM_BLE_AD_TYPE_NAME_SHORT in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_NAME_CMPL                = 0x09,    /* relate to BTM_BLE_AD_TYPE_NAME_CMPL in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_TX_PWR                   = 0x0A,    /* relate to BTM_BLE_AD_TYPE_TX_PWR in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_DEV_CLASS                = 0x0D,    /* relate to BTM_BLE_AD_TYPE_DEV_CLASS in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SM_TK                    = 0x10,    /* relate to BTM_BLE_AD_TYPE_SM_TK in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SM_OOB_FLAG              = 0x11,    /* relate to BTM_BLE_AD_TYPE_SM_OOB_FLAG in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_INT_RANGE                = 0x12,    /* relate to BTM_BLE_AD_TYPE_INT_RANGE in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SOL_SRV_UUID             = 0x14,    /* relate to BTM_BLE_AD_TYPE_SOL_SRV_UUID in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_128SOL_SRV_UUID          = 0x15,    /* relate to BTM_BLE_AD_TYPE_128SOL_SRV_UUID in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SERVICE_DATA             = 0x16,    /* relate to BTM_BLE_AD_TYPE_SERVICE_DATA in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_PUBLIC_TARGET            = 0x17,    /* relate to BTM_BLE_AD_TYPE_PUBLIC_TARGET in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_RANDOM_TARGET            = 0x18,    /* relate to BTM_BLE_AD_TYPE_RANDOM_TARGET in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_APPEARANCE               = 0x19,    /* relate to BTM_BLE_AD_TYPE_APPEARANCE in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_ADV_INT                  = 0x1A,    /* relate to BTM_BLE_AD_TYPE_ADV_INT in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_LE_DEV_ADDR              = 0x1b,    /* relate to BTM_BLE_AD_TYPE_LE_DEV_ADDR in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_LE_ROLE                  = 0x1c,    /* relate to BTM_BLE_AD_TYPE_LE_ROLE in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SPAIR_C256               = 0x1d,    /* relate to BTM_BLE_AD_TYPE_SPAIR_C256 in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_SPAIR_R256               = 0x1e,    /* relate to BTM_BLE_AD_TYPE_SPAIR_R256 in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_32SOL_SRV_UUID           = 0x1f,    /* relate to BTM_BLE_AD_TYPE_32SOL_SRV_UUID in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_32SERVICE_DATA           = 0x20,    /* relate to BTM_BLE_AD_TYPE_32SERVICE_DATA in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_128SERVICE_DATA          = 0x21,    /* relate to BTM_BLE_AD_TYPE_128SERVICE_DATA in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_LE_SECURE_CONFIRM        = 0x22,    /* relate to BTM_BLE_AD_TYPE_LE_SECURE_CONFIRM in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_LE_SECURE_RANDOM         = 0x23,    /* relate to BTM_BLE_AD_TYPE_LE_SECURE_RANDOM in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_URI                      = 0x24,    /* relate to BTM_BLE_AD_TYPE_URI in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_INDOOR_POSITION          = 0x25,    /* relate to BTM_BLE_AD_TYPE_INDOOR_POSITION in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_TRANS_DISC_DATA          = 0x26,    /* relate to BTM_BLE_AD_TYPE_TRANS_DISC_DATA in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_LE_SUPPORT_FEATURE       = 0x27,    /* relate to BTM_BLE_AD_TYPE_LE_SUPPORT_FEATURE in stack/btm_ble_api.h */
    ESP_BLE_AD_TYPE_CHAN_MAP_UPDATE          = 0x28,    /* relate to BTM_BLE_AD_TYPE_CHAN_MAP_UPDATE in stack/btm_ble_api.h */
    ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE    = 0xFF,    /* relate to BTM_BLE_AD_MANUFACTURER_SPECIFIC_TYPE in stack/btm_ble_api.h */
} esp_ble_adv_data_type;
```
###### esp_ble_adv_params_t
```c
typedef struct {
    uint16_t                adv_int_min;        /* linked Minimum advertising interval for
                                                  undirected and low duty cycle directed advertising.
                                                  Range: 0x0020 to 0x4000 Default: N = 0x0800 (1.28 second)
                                                  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec */
    uint16_t                adv_int_max;        /* linked Maximum advertising interval for
                                                  undirected and low duty cycle directed advertising.
                                                  Range: 0x0020 to 0x4000 Default: N = 0x0800 (1.28 second)
                                                  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec Advertising max interval */
    esp_ble_adv_type_t      adv_type;           /* linked Advertising type */
    esp_ble_addr_type_t     own_addr_type;      /* linked Owner bluetooth device address type */
    esp_bd_addr_t           peer_addr;          /* linked Peer device bluetooth device address */
    esp_ble_addr_type_t     peer_addr_type;     /* linked Peer device bluetooth device address type, only support public address type and random address type */
    esp_ble_adv_channel_t   channel_map;        /* linked Advertising channel map */
    esp_ble_adv_filter_t    adv_filter_policy;  /* linked Advertising filter policy */
} esp_ble_adv_params_t;
```
###### esp_ble_gatts_send_indicate
[GATTS Notify](Telink.md/#gatts-notify)
```c
esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                      uint16_t value_len, uint8_t *value, bool need_confirm) //[GATTS Notify](Telink.md/#gatts-notify)
{
    if (value_len > ESP_GATT_MAX_ATTR_LEN) {
        LOG_ERROR("%s, value_len > ESP_GATT_MAX_ATTR_LEN.", __func__);
        return ESP_ERR_INVALID_SIZE;
    }

    btc_msg_t msg = {0};
    btc_ble_gatts_args_t arg;

    ESP_BLUEDROID_STATUS_CHECK(ESP_BLUEDROID_STATUS_ENABLED);

    tGATT_TCB       *p_tcb = gatt_get_tcb_by_idx(conn_id);
    if (!gatt_check_connection_state_by_tcb(p_tcb)) {
        LOG_WARN("%s, The connection not created.", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if (L2CA_CheckIsCongest(L2CAP_ATT_CID, p_tcb->peer_bda)) {
        LOG_DEBUG("%s, the l2cap channel is congest.", __func__);
        return ESP_FAIL;
    }

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_GATTS;
    msg.act = BTC_GATTS_ACT_SEND_INDICATE;
    arg.send_ind.conn_id = BTC_GATT_CREATE_CONN_ID(gatts_if, conn_id);
    arg.send_ind.attr_handle = attr_handle;
    arg.send_ind.need_confirm = need_confirm;
    arg.send_ind.value_len = value_len;
    arg.send_ind.value = value;
    if(need_confirm == false){
        l2ble_update_att_acl_pkt_num(L2CA_ADD_BTC_NUM, NULL);
    }
    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_gatts_args_t), btc_gatts_arg_deep_copy,
                btc_gatts_arg_deep_free) == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}
```
###### esp_gatt_status_t
```c
typedef enum {
    ESP_GATT_OK                     =   0x0,    /* linked 0x0, Operation successful. Corresponds to BTA_GATT_OK. */
    ESP_GATT_INVALID_HANDLE         =   0x01,   /* linked 0x01, Invalid handle. Corresponds to BTA_GATT_INVALID_HANDLE. */
    ESP_GATT_READ_NOT_PERMIT        =   0x02,   /* linked 0x02, Read operation not permitted. Corresponds to BTA_GATT_READ_NOT_PERMIT. */
    ESP_GATT_WRITE_NOT_PERMIT       =   0x03,   /* linked 0x03, Write operation not permitted. Corresponds to BTA_GATT_WRITE_NOT_PERMIT. */
    ESP_GATT_INVALID_PDU            =   0x04,   /* linked 0x04, Invalid PDU. Corresponds to BTA_GATT_INVALID_PDU. */
    ESP_GATT_INSUF_AUTHENTICATION   =   0x05,   /* linked 0x05, Insufficient authentication. Corresponds to BTA_GATT_INSUF_AUTHENTICATION. */
    ESP_GATT_REQ_NOT_SUPPORTED      =   0x06,   /* linked 0x06, Request not supported. Corresponds to BTA_GATT_REQ_NOT_SUPPORTED. */
    ESP_GATT_INVALID_OFFSET         =   0x07,   /* linked 0x07, Invalid offset. Corresponds to BTA_GATT_INVALID_OFFSET. */
    ESP_GATT_INSUF_AUTHORIZATION    =   0x08,   /* linked 0x08, Insufficient authorization. Corresponds to BTA_GATT_INSUF_AUTHORIZATION. */
    ESP_GATT_PREPARE_Q_FULL         =   0x09,   /* linked 0x09, Prepare queue full. Corresponds to BTA_GATT_PREPARE_Q_FULL. */
    ESP_GATT_NOT_FOUND              =   0x0a,   /* linked 0x0a, Not found. Corresponds to BTA_GATT_NOT_FOUND. */
    ESP_GATT_NOT_LONG               =   0x0b,   /* linked 0x0b, Not long. Corresponds to BTA_GATT_NOT_LONG. */
    ESP_GATT_INSUF_KEY_SIZE         =   0x0c,   /* linked 0x0c, Insufficient key size. Corresponds to BTA_GATT_INSUF_KEY_SIZE. */
    ESP_GATT_INVALID_ATTR_LEN       =   0x0d,   /* linked 0x0d, Invalid attribute length. Corresponds to BTA_GATT_INVALID_ATTR_LEN. */
    ESP_GATT_ERR_UNLIKELY           =   0x0e,   /* linked 0x0e, Unlikely error. Corresponds to BTA_GATT_ERR_UNLIKELY. */
    ESP_GATT_INSUF_ENCRYPTION       =   0x0f,   /* linked 0x0f, Insufficient encryption. Corresponds to BTA_GATT_INSUF_ENCRYPTION. */
    ESP_GATT_UNSUPPORT_GRP_TYPE     =   0x10,   /* linked 0x10, Unsupported group type. Corresponds to BTA_GATT_UNSUPPORT_GRP_TYPE. */
    ESP_GATT_INSUF_RESOURCE         =   0x11,   /* linked 0x11, Insufficient resource. Corresponds to BTA_GATT_INSUF_RESOURCE. */

    /* Additional error codes specific to implementation or future use */
    ESP_GATT_NO_RESOURCES           =   0x80,   /* linked 0x80, No resources. Corresponds to BTA_GATT_NO_RESOURCES. */
    ESP_GATT_INTERNAL_ERROR         =   0x81,   /* linked 0x81, Internal error. Corresponds to BTA_GATT_INTERNAL_ERROR. */
    ESP_GATT_WRONG_STATE            =   0x82,   /* linked 0x82, Wrong state. Corresponds to BTA_GATT_WRONG_STATE. */
    ESP_GATT_DB_FULL                =   0x83,   /* linked 0x83, Database full. Corresponds to BTA_GATT_DB_FULL. */
    ESP_GATT_BUSY                   =   0x84,   /* linked 0x84, Busy. Corresponds to BTA_GATT_BUSY. */
    ESP_GATT_ERROR                  =   0x85,   /* linked 0x85, Generic error. Corresponds to BTA_GATT_ERROR. */
    ESP_GATT_CMD_STARTED            =   0x86,   /* linked 0x86, Command started. Corresponds to BTA_GATT_CMD_STARTED. */
    ESP_GATT_ILLEGAL_PARAMETER      =   0x87,   /* linked 0x87, Illegal parameter. Corresponds to BTA_GATT_ILLEGAL_PARAMETER. */
    ESP_GATT_PENDING                =   0x88,   /* linked 0x88, Operation pending. Corresponds to BTA_GATT_PENDING. */
    ESP_GATT_AUTH_FAIL              =   0x89,   /* linked 0x89, Authentication failed. Corresponds to BTA_GATT_AUTH_FAIL. */
    ESP_GATT_MORE                   =   0x8a,   /* linked 0x8a, More data available. Corresponds to BTA_GATT_MORE. */
    ESP_GATT_INVALID_CFG            =   0x8b,   /* linked 0x8b, Invalid configuration. Corresponds to BTA_GATT_INVALID_CFG. */
    ESP_GATT_SERVICE_STARTED        =   0x8c,   /* linked 0x8c, Service started. Corresponds to BTA_GATT_SERVICE_STARTED. */
    ESP_GATT_ENCRYPTED_MITM         =   ESP_GATT_OK, /* linked 0x0, Encrypted, with MITM protection. Corresponds to BTA_GATT_ENCRYPTED_MITM. */
    ESP_GATT_ENCRYPTED_NO_MITM      =   0x8d,   /* linked 0x8d, Encrypted, without MITM protection. Corresponds to BTA_GATT_ENCRYPTED_NO_MITM. */
    ESP_GATT_NOT_ENCRYPTED          =   0x8e,   /* linked 0x8e, Not encrypted. Corresponds to BTA_GATT_NOT_ENCRYPTED. */
    ESP_GATT_CONGESTED              =   0x8f,   /* linked 0x8f, Congested. Corresponds to BTA_GATT_CONGESTED. */
    ESP_GATT_DUP_REG                =   0x90,   /* linked 0x90, Duplicate registration. Corresponds to BTA_GATT_DUP_REG. */
    ESP_GATT_ALREADY_OPEN           =   0x91,   /* linked 0x91, Already open. Corresponds to BTA_GATT_ALREADY_OPEN. */
    ESP_GATT_CANCEL                 =   0x92,   /* linked 0x92, Operation cancelled. Corresponds to BTA_GATT_CANCEL. */
    /* 0xE0 ~ 0xFC reserved for future use */
    ESP_GATT_STACK_RSP              =   0xe0,   /* linked 0xe0, Stack response. Corresponds to BTA_GATT_STACK_RSP. */
    ESP_GATT_APP_RSP                =   0xe1,   /* linked 0xe1, Application response. Corresponds to BTA_GATT_APP_RSP. */
    /* Error caused by customer application or stack bug */
    ESP_GATT_UNKNOWN_ERROR          =   0xef,   /* linked 0xef, Unknown error. Corresponds to BTA_GATT_UNKNOWN_ERROR. */
    ESP_GATT_CCC_CFG_ERR            =   0xfd,   /* linked 0xfd, Client Characteristic Configuration Descriptor improperly configured. Corresponds to BTA_GATT_CCC_CFG_ERR. */
    ESP_GATT_PRC_IN_PROGRESS        =   0xfe,   /* linked 0xfe, Procedure already in progress. Corresponds to BTA_GATT_PRC_IN_PROGRESS. */
    ESP_GATT_OUT_OF_RANGE           =   0xff    /* linked 0xff, Attribute value out of range. Corresponds to BTA_GATT_OUT_OF_RANGE. */
} esp_gatt_status_t;