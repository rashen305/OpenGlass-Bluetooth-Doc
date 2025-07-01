###### BLEUUID Constructor
[esp_bt_uuid_t](ESP-IDF.md/#uuid-type)
```cpp
BLEUUID::BLEUUID(String value) {
  //Serial.printf("BLEUUID constructor from String=\"%s\"\n", value.c_str());
  m_valueSet = true;
  if (value.length() == 4) {
    m_uuid.len = ESP_UUID_LEN_16; //defined as linked m_uuid;
    m_uuid.uuid.uuid16 = 0;
    for (int i = 0; i < value.length();) {
      uint8_t MSB = value.c_str()[i];
      uint8_t LSB = value.c_str()[i + 1];

      if (MSB > '9') {
        MSB -= 7;
      }
      if (LSB > '9') {
        LSB -= 7;
      }
      m_uuid.uuid.uuid16 += (((MSB & 0x0F) << 4) | (LSB & 0x0F)) << (2 - i) * 4;
      i += 2;
    }
  } else if (value.length() == 8) {
    m_uuid.len = ESP_UUID_LEN_32;
    m_uuid.uuid.uuid32 = 0;
    for (int i = 0; i < value.length();) {
      uint8_t MSB = value.c_str()[i];
      uint8_t LSB = value.c_str()[i + 1];

      if (MSB > '9') {
        MSB -= 7;
      }
      if (LSB > '9') {
        LSB -= 7;
      }
      m_uuid.uuid.uuid32 += (((MSB & 0x0F) << 4) | (LSB & 0x0F)) << (6 - i) * 4;
      i += 2;
    }
  } else if (value.length()
             == 16) {  // How we can have 16 byte length string representing 128 bit uuid??? needs to be investigated (lack of time) - maybe raw data encoded as String (128b==16B)?
    m_uuid.len = ESP_UUID_LEN_128;
    memrcpy(m_uuid.uuid.uuid128, (uint8_t *)value.c_str(), 16);
  } else if (value.length() == 36) {
    //log_d("36 characters:");
    // If the length of the string is 36 bytes then we will assume it is a long hex string in
    // UUID format.
    m_uuid.len = ESP_UUID_LEN_128;
    int n = 0;
    for (int i = 0; i < value.length();) {
      if (value.c_str()[i] == '-') {
        i++;
      }
      uint8_t MSB = value.c_str()[i];
      uint8_t LSB = value.c_str()[i + 1];

      if (MSB > '9') {
        MSB -= 7;
      }
      if (LSB > '9') {
        LSB -= 7;
      }
      m_uuid.uuid.uuid128[15 - n++] = ((MSB & 0x0F) << 4) | (LSB & 0x0F);
      i += 2;
    }
  } else {
    log_e("ERROR: UUID value not 2, 4, 16 or 36 bytes");
    m_valueSet = false;
  }
}  //BLEUUID(String)
```

###### BLEServerCallbacks
```cpp
void BLEServerCallbacks::onConnect(BLEServer *pServer) {
  log_d("BLEServerCallbacks", ">> onConnect(): Default");
  log_d("BLEServerCallbacks", "Device: %s", BLEDevice::toString().c_str());
  log_d("BLEServerCallbacks", "<< onConnect()");
}  // onConnect

void BLEServerCallbacks::onDisconnect(BLEServer *pServer) {
  log_d("BLEServerCallbacks", ">> onDisconnect(): Default");
  log_d("BLEServerCallbacks", "Device: %s", BLEDevice::toString().c_str());
  log_d("BLEServerCallbacks", "<< onDisconnect()");
}  // onDisconnect
```

###### BLECharacteristicCallbacks
```cpp
void BLECharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  log_d(">> onWrite: default");
  log_d("<< onWrite");
}  // onWrite
```

###### characteristic->getLength()
[BLEValue](Arduino%20BLE%20Libraries.md/#blevalue)
```cpp
size_t BLECharacteristic::getLength() {
  return m_value.getLength(); //linked m_value; private variable in the class
}  // getLength
```

###### characteristic->getData()
[BLEValue](Arduino%20BLE%20Libraries.md/#blevalue)
```cpp
uint8_t *BLECharacteristic::getData() {
  return m_value.getData(); //linked m_value; private variable in the class
}  // getData
```

###### BLEValue
```cpp
BLEValue::BLEValue() {
  m_accumulation = ""; //String m_accumulation;
  m_value = ""; //String m_value;
  m_readOffset = 0; //uint16_t m_readOffset;
}  // BLEValue

size_t BLEValue::getLength() {
  return m_value.length();
}  // getLength

uint8_t *BLEValue::getData() {
  return (uint8_t *)m_value.c_str();
}
```

###### BLEDevice::init
[btStart()](Arduino%20BLE%20Libraries.md/#btstart)  
[esp_err_t](ESP-IDF.md/#esp_err_t)  
[nvs_flash_init](ESP-IDF.md/#nvs_flash_init)  
[esp_bt_controller_mem_release](ESP-IDF.md/#esp_bt_controller_mem_release)  
[esp_bt_mode_t](ESP-IDF.md/#esp_bt_mode_t)  
[esp_bt_controller_config_t](ESP-IDF.md/#esp_bt_controller_config_t)  
[BT_CONTROLLER_INIT_CONFIG_DEFAULT](ESP-IDF.md/#bt_controller_init_config_default)  
[esp_bt_controller_init](ESP-IDF.md/#esp_bt_controller_init)  
[esp_bt_controller_enable](ESP-IDF.md/#esp_bt_controller_enable)  
[esp_bluedroid_status_t](ESP-IDF.md/#esp_bluedroid_status_t)  
[esp_bluedroid_get_status](ESP-IDF.md/#esp_bluedroid_get_status)  
[esp_bluedroid_init](ESP-IDF.md/#esp_bluedroid_init)  
[esp_bluedroid_enable](ESP-IDF.md/#esp_bluedroid_enable)  
[esp_ble_gap_register_callback](ESP-IDF.md/#esp_ble_gap_register_callback)  
[esp_ble_gattc_register_callback](ESP-IDF.md/#esp_ble_gattc_register_callback)  
[esp_ble_gap_set_device_name](ESP-IDF.md/#esp_ble_gap_set_device_name)  
[esp_ble_gap_set_security_param](ESP-IDF.md/#esp_ble_gap_set_security_param)
```cpp
/* STATIC */ void BLEDevice::init(String deviceName) {
  if (!initialized) {
    initialized = true;  // Set the initialization flag to ensure we are only initialized once.

    esp_err_t errRc = ESP_OK;
#ifdef ARDUINO_ARCH_ESP32
    if (!btStart()) { //linked
      errRc = ESP_FAIL; //linked errRc; #define ESP_FAIL        -1
      return;
    }
#else
    errRc = ::nvs_flash_init(); //linked
    if (errRc != ESP_OK) { //#define ESP_OK 0
      log_e("nvs_flash_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }

#ifndef CONFIG_BT_CLASSIC_ENABLED
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); // linked and (linked mode);
#endif
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT(); //linked and linked
    errRc = esp_bt_controller_init(&bt_cfg); //linked
    if (errRc != ESP_OK) {
      log_e("esp_bt_controller_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }

#ifndef CONFIG_BT_CLASSIC_ENABLED
    errRc = esp_bt_controller_enable(ESP_BT_MODE_BLE); //linked
    if (errRc != ESP_OK) {
      log_e("esp_bt_controller_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }
#else
    errRc = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (errRc != ESP_OK) {
      log_e("esp_bt_controller_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }
#endif
#endif

    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status(); //linked and linked
    if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
      errRc = esp_bluedroid_init(); //linked
      if (errRc != ESP_OK) {
        log_e("esp_bluedroid_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
        return;
      }
    }

    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
      errRc = esp_bluedroid_enable(); //linked
      if (errRc != ESP_OK) {
        log_e("esp_bluedroid_enable: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
        return;
      }
    }

    errRc = esp_ble_gap_register_callback(BLEDevice::gapEventHandler); //linked
    if (errRc != ESP_OK) {
      log_e("esp_ble_gap_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }

#ifdef CONFIG_GATTC_ENABLE  // Check that BLE client is configured in make menuconfig
    errRc = esp_ble_gattc_register_callback(BLEDevice::gattClientEventHandler); //linked
    if (errRc != ESP_OK) {
      log_e("esp_ble_gattc_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }
#endif  // CONFIG_GATTC_ENABLE

#ifdef CONFIG_GATTS_ENABLE  // Check that BLE server is configured in make menuconfig
    errRc = esp_ble_gatts_register_callback(BLEDevice::gattServerEventHandler);
    if (errRc != ESP_OK) {
      log_e("esp_ble_gatts_register_callback: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    }
#endif  // CONFIG_GATTS_ENABLE

    errRc = ::esp_ble_gap_set_device_name(deviceName.c_str()); //linked
    if (errRc != ESP_OK) {
      log_e("esp_ble_gap_set_device_name: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    };

#ifdef CONFIG_BLE_SMP_ENABLE  // Check that BLE SMP (security) is configured in make menuconfig
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    errRc = ::esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t)); //linked
    if (errRc != ESP_OK) {
      log_e("esp_ble_gap_set_security_param: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
      return;
    };
#endif  // CONFIG_BLE_SMP_ENABLE
  }
  vTaskDelay(200 / portTICK_PERIOD_MS);  // Delay for 200 msecs as a workaround to an apparent Arduino environment issue.
}  // init
```

###### btStart()
[btStartMode()](Arduino%20BLE%20Libraries.md/#btstartmode)
```cpp
bool btStart() {
  return btStartMode(BT_MODE); //linked
}
```

###### btStartMode()
[esp_bt_controller_get_status](ESP-IDF.md/#esp_bt_controller_get_status)  
[esp_err_to_name](ESP-IDF.md/#esp_err_to_name)
```cpp
bool btStartMode(bt_mode mode) {
  esp_bt_mode_t esp_bt_mode;
  esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
  switch (mode) {
    case BT_MODE_BLE:        esp_bt_mode = ESP_BT_MODE_BLE; break;
    case BT_MODE_CLASSIC_BT: esp_bt_mode = ESP_BT_MODE_CLASSIC_BT; break;
    case BT_MODE_BTDM:       esp_bt_mode = ESP_BT_MODE_BTDM; break;
    default:                 esp_bt_mode = BT_MODE; break;
  }
  // esp_bt_controller_enable(MODE) This mode must be equal as the mode in "cfg" of esp_bt_controller_init().
  cfg.mode = esp_bt_mode;
  if (cfg.mode == ESP_BT_MODE_CLASSIC_BT) {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  }
#else
  // other esp variants dont support BT-classic / DM.
  esp_bt_mode = BT_MODE;
#endif

  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) { //linked
    return true;
  }
  esp_err_t ret;
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    if ((ret = esp_bt_controller_init(&cfg)) != ESP_OK) {
      log_e("initialize controller failed: %s", esp_err_to_name(ret));
      return false;
    }
    while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {}
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    if ((ret = esp_bt_controller_enable(esp_bt_mode)) != ESP_OK) {
      log_e("BT Enable mode=%d failed %s", BT_MODE, esp_err_to_name(ret)); //linked
      return false;
    }
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    return true;
  }
  log_e("BT Start failed");
  return false;
}
```

###### BLEDevice::createServer()
[BLEServer::BLEServer()](Arduino%20BLE%20Libraries.md/#bleserverbleserver)  
[BLEServer::createApp](Arduino%20BLE%20Libraries.md/#bleservercreateapp)
```cpp
/**
 * @brief Create a new instance of a server.
 * @return A new instance of the server.
 */
/* STATIC */ BLEServer* BLEDevice::createServer() {
	ESP_LOGD(LOG_TAG, ">> createServer");
#ifndef CONFIG_GATTS_ENABLE  // Check that BLE GATTS is enabled in make menuconfig
	ESP_LOGE(LOG_TAG, "BLE GATTS is not enabled - CONFIG_GATTS_ENABLE not defined");
	abort();
#endif // CONFIG_GATTS_ENABLE
	m_pServer = new BLEServer(); //linked
	m_pServer->createApp(m_appId++); //linked
	ESP_LOGD(LOG_TAG, "<< createServer");
	return m_pServer;
} // createServer
```

###### BLEServer::BLEServer()
```cpp
BLEServer::BLEServer() {
  m_appId = ESP_GATT_IF_NONE;
  m_gatts_if = ESP_GATT_IF_NONE;
  m_connectedCount = 0;
  m_connId = ESP_GATT_IF_NONE;
  m_pServerCallbacks = nullptr;
}  // BLEServer
```

###### BLEServer::createApp
[BLEServer::registerApp](Arduino%20BLE%20Libraries.md/#bleserverregisterapp)
```cpp
void BLEServer::createApp(uint16_t appId) {
  m_appId = appId;
  registerApp(appId); //linked
}  // createApp
```

###### BLEServer::registerApp
[m_semaphoreRegisterAppEvt](Arduino%20BLE%20Libraries.md/#m_semaphoreregisterappevt)  
[esp_ble_gatts_app_register](ESP-IDF.md/#esp_ble_gatts_app_register)
```cpp
void BLEServer::registerApp(uint16_t m_appId) {
  log_v(">> registerApp - %d", m_appId);
  m_semaphoreRegisterAppEvt.take("registerApp");  // Take the mutex, will be released by ESP_GATTS_REG_EVT event.
  //linked
  ::esp_ble_gatts_app_register(m_appId); //linked
  m_semaphoreRegisterAppEvt.wait("registerApp");
  log_v("<< registerApp");
}  // registerApp
```

###### m_semaphoreRegisterAppEvt
```cpp
FreeRTOS::Semaphore m_semaphoreRegisterAppEvt = FreeRTOS::Semaphore("RegisterAppEvt");

```

###### setCallbacks
```cpp
void BLEServer::setCallbacks(BLEServerCallbacks *pCallbacks) {
  m_pServerCallbacks = pCallbacks;
}  // setCallbacks
```

###### server->createService
[BLEServer::createService](Arduino%20BLE%20Libraries.md/#bleservercreateservice)
```cpp
BLEService *BLEServer::createService(const char *uuid) {
  return createService(BLEUUID(uuid)); //linked
}
```

###### BLEServer::createService
[BLEServiceMap](Arduino%20BLE%20Libraries.md/#bleservicemap)  
[BLEServiceMap::getByUUID](Arduino%20BLE%20Libraries.md/#bleservicemapgetbyuuid)  
[BLEService::BLEService](Arduino%20BLE%20Libraries.md/#bleservicebleservice)  
[BLEServiceMap::setByUUID](Arduino%20BLE%20Libraries.md/#bleservicemapsetbyuuid)  
[executeCreate](Arduino%20BLE%20Libraries.md/#bleserviceexecutecreate)
```cpp
BLEService *BLEServer::createService(BLEUUID uuid, uint32_t numHandles, uint8_t inst_id) {
  log_v(">> createService - %s", uuid.toString().c_str());
  m_semaphoreCreateEvt.take("createService");

  // Check that a service with the supplied UUID does not already exist.
  if (m_serviceMap.getByUUID(uuid) != nullptr) { //linked and linked
    log_w("<< Attempt to create a new service with uuid %s but a service with that UUID already exists.", uuid.toString().c_str());
  }

  BLEService *pService = new BLEService(uuid, numHandles); //linked
  pService->m_instId = inst_id;
  m_serviceMap.setByUUID(uuid, pService);  // Save a reference to this service being on this server. linked
  pService->executeCreate(this);           // Perform the API calls to actually create the service. linked

  m_semaphoreCreateEvt.wait("createService");

  log_v("<< createService");
  return pService;
}  // createService
```

###### BLEService::BLEService
```cpp
BLEService::BLEService(const char *uuid, uint16_t numHandles) : BLEService(BLEUUID(uuid), numHandles) {}

/**
 * @brief Construct an instance of the BLEService
 * @param [in] uuid The UUID of the service.
 * @param [in] numHandles The maximum number of handles associated with the service.
 */
BLEService::BLEService(BLEUUID uuid, uint16_t numHandles) {
  m_uuid = uuid;
  m_handle = NULL_HANDLE;
  m_pServer = nullptr;
  //m_serializeMutex.setName("BLEService");
  m_lastCreatedCharacteristic = nullptr;
  m_numHandles = numHandles;
}  // BLEService
```

###### BLEServiceMap
```cpp
class BLEServiceMap {
public:
  BLEService *getByHandle(uint16_t handle);
  BLEService *getByUUID(const char *uuid);
  BLEService *getByUUID(BLEUUID uuid, uint8_t inst_id = 0);
  void handleGATTServerEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
  void setByHandle(uint16_t handle, BLEService *service);
  void setByUUID(const char *uuid, BLEService *service);
  void setByUUID(BLEUUID uuid, BLEService *service);
  String toString();
  BLEService *getFirst();
  BLEService *getNext();
  void removeService(BLEService *service);
  int getRegisteredServiceCount();

private:
  std::map<uint16_t, BLEService *> m_handleMap;
  std::map<BLEService *, String> m_uuidMap;
  std::map<BLEService *, String>::iterator m_iterator;
};
```

###### BLEServiceMap::getByUUID
```cpp
BLEService *BLEServiceMap::getByUUID(const char *uuid) {
  return getByUUID(BLEUUID(uuid));
}

/**
 * @brief Return the service by UUID.
 * @param [in] UUID The UUID to look up the service.
 * @return The characteristic.
 */
BLEService *BLEServiceMap::getByUUID(BLEUUID uuid, uint8_t inst_id) {
  for (auto &myPair : m_uuidMap) {
    if (myPair.first->getUUID().equals(uuid)) {
      return myPair.first;
    }
  }
  //return m_uuidMap.at(uuid.toString());
  return nullptr;
}  // getByUUID
```

###### BLEServiceMap::setByUUID
[std::map](Arduino%20BLE%20Libraries.md/#stdmap)
```cpp
void BLEServiceMap::setByUUID(BLEUUID uuid, BLEService *service) {
  m_uuidMap.insert(std::pair<BLEService *, String>(service, uuid.toString())); // linked<BLEService *, String> m_uuidMap
}  // setByUUID
```

###### std::map
```cpp
std::map<BLEService *, String> m_uuidMap;
```

###### BLEService::executeCreate
[esp_ble_gatts_create_service](ESP-IDF.md/#esp_ble_gatts_create_service)
```cpp
void BLEService::executeCreate(BLEServer *pServer) {
  log_v(">> executeCreate() - Creating service (esp_ble_gatts_create_service) service uuid: %s", getUUID().toString().c_str());
  m_pServer = pServer;
  m_semaphoreCreateEvt.take("executeCreate");  // Take the mutex and release at event ESP_GATTS_CREATE_EVT

  esp_gatt_srvc_id_t srvc_id;
  srvc_id.is_primary = true;
  srvc_id.id.inst_id = m_instId;
  srvc_id.id.uuid = *m_uuid.getNative();
  esp_err_t errRc =
    ::esp_ble_gatts_create_service(getServer()->getGattsIf(), &srvc_id, m_numHandles);  // The maximum number of handles associated with the service.
//linked
  if (errRc != ESP_OK) {
    log_e("esp_ble_gatts_create_service: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
    return;
  }

  m_semaphoreCreateEvt.wait("executeCreate");
  log_v("<< executeCreate");
}  // executeCreate
```

###### server->createCharacteristic
[BLECharacteristic::BLECharacteristic](Arduino%20BLE%20Libraries.md/#blecharacteristicblecharacteristic)  
[addCharacteristic](Arduino%20BLE%20Libraries.md/#addcharacteristic)
```cpp
BLECharacteristic *BLEService::createCharacteristic(const char *uuid, uint32_t properties) {
  return createCharacteristic(BLEUUID(uuid), properties);
}

/**
 * @brief Create a new BLE Characteristic associated with this service.
 * @param [in] uuid - The UUID of the characteristic.
 * @param [in] properties - The properties of the characteristic.
 * @return The new BLE characteristic.
 */
BLECharacteristic *BLEService::createCharacteristic(BLEUUID uuid, uint32_t properties) {
  BLECharacteristic *pCharacteristic = new BLECharacteristic(uuid, properties); //linked
  addCharacteristic(pCharacteristic); //linked
  return pCharacteristic;
}  // createCharacteristic
```

###### BLECharacteristic::BLECharacteristic
[BLECharacteristic::setBroadcastProperty](Arduino%20BLE%20Libraries.md/#blecharacteristicsetbroadcastproperty)
```cpp
BLECharacteristic::BLECharacteristic(BLEUUID uuid, uint32_t properties) {
  m_bleUUID = uuid;
  m_handle = NULL_HANDLE;
  m_properties = (esp_gatt_char_prop_t)0;
  m_pCallbacks = &defaultCallback;

  setBroadcastProperty((properties & PROPERTY_BROADCAST) != 0); //linked
  setReadProperty((properties & PROPERTY_READ) != 0);
  setWriteProperty((properties & PROPERTY_WRITE) != 0);
  setNotifyProperty((properties & PROPERTY_NOTIFY) != 0);
  setIndicateProperty((properties & PROPERTY_INDICATE) != 0);
  setWriteNoResponseProperty((properties & PROPERTY_WRITE_NR) != 0);
}  // BLECharacteristic
```

###### addCharacteristic
```cpp
void BLEService::addCharacteristic(BLECharacteristic *pCharacteristic) {
  // We maintain a mapping of characteristics owned by this service.  These are managed by the
  // BLECharacteristicMap class instance found in m_characteristicMap.  We add the characteristic
  // to the map and then ask the service to add the characteristic at the BLE level (ESP-IDF).

  log_v(">> addCharacteristic()");
  log_d("Adding characteristic: uuid=%s to service: %s", pCharacteristic->getUUID().toString().c_str(), toString().c_str());

  // Check that we don't add the same characteristic twice.
  if (m_characteristicMap.getByUUID(pCharacteristic->getUUID()) != nullptr) {
    log_w("<< Adding a new characteristic with the same UUID as a previous one");
    //return;
  }

  // Remember this characteristic in our map of characteristics.  At this point, we can lookup by UUID
  // but not by handle.  The handle is allocated to us on the ESP_GATTS_ADD_CHAR_EVT.
  m_characteristicMap.setByUUID(pCharacteristic, pCharacteristic->getUUID());

  log_v("<< addCharacteristic()");
}  // addCharacteristic
```

###### BLECharacteristic::setBroadcastProperty
[esp_gatt_char_prop_t](ESP-IDF.md/#esp_gatt_char_prop_t)
```cpp
void BLECharacteristic::setBroadcastProperty(bool value) {
  //log_d("setBroadcastProperty(%d)", value);
  if (value) {
    m_properties = (esp_gatt_char_prop_t)(m_properties | ESP_GATT_CHAR_PROP_BIT_BROADCAST); //linked m_properties;
  } else {
    m_properties = (esp_gatt_char_prop_t)(m_properties & ~ESP_GATT_CHAR_PROP_BIT_BROADCAST);
  }
}  // setBroadcastProperty
```

###### BLE2902()
[BLEDescriptor::setValue](Arduino%20BLE%20Libraries.md/#bledescriptorsetvalue)
```cpp
BLE2902::BLE2902() : BLEDescriptor(BLEUUID((uint16_t)0x2902)) {
  uint8_t data[2] = {0, 0};
  setValue(data, 2); //linked
}  // BLE2902
```

###### BLEDescriptor::setValue
[esp_attr_value_t](ESP-IDF.md/#esp_attr_value_t)  
[esp_ble_gatts_set_attr_value](ESP-IDF.md/#esp_ble_gatts_set_attr_value)
```cpp
void BLEDescriptor::setValue(uint8_t *data, size_t length) {
  if (length > ESP_GATT_MAX_ATTR_LEN) {
    log_e("Size %d too large, must be no bigger than %d", length, ESP_GATT_MAX_ATTR_LEN);
    return;
  }
  m_value.attr_len = length; //linked m_value;
  memcpy(m_value.attr_value, data, length);
  if (m_handle != NULL_HANDLE) {
    esp_ble_gatts_set_attr_value(m_handle, length, (const uint8_t *)data); //linked
    log_d("Set the value in the GATTS database using handle 0x%x", m_handle);
  }
}  // setValue
```

###### setNotifications()
[BLEDescriptor::getValue()](Arduino%20BLE%20Libraries.md/#bledescriptorgetvalue)
```cpp
void BLE2902::setNotifications(bool flag) {
  uint8_t *pValue = getValue();
  if (flag) {
    pValue[0] |= 1 << 0;
  } else {
    pValue[0] &= ~(1 << 0);
  }
  setValue(pValue, 2); //linked
}  // setNotifications
```

###### BLEDescriptor::getValue()
[esp_attr_value_t](ESP-IDF.md/#esp_attr_value_t)
```cpp
uint8_t *BLEDescriptor::getValue() {
  return m_value.attr_value; //linked m_value;
}  // getValue
```

###### addDescriptor()
[std::map](Arduino%20BLE%20Libraries.md/#stdmap)
```cpp
void BLECharacteristic::addDescriptor(BLEDescriptor *pDescriptor) {//BLEDescriptor is the parent class of BLE2902
  log_v(">> addDescriptor(): Adding %s to %s", pDescriptor->toString().c_str(), toString().c_str());
  m_descriptorMap.setByUUID(pDescriptor->getUUID(), pDescriptor); 
  //linked<std::string, BLERemoteDescriptor *> m_descriptorMap;
  log_v("<< addDescriptor()");
}  // addDescriptor
```

###### std::map
```cpp
std::map<std::string, BLERemoteDescriptor *> m_descriptorMap;
```

###### BLECharacteristic->setValue()
[BLECharacteristic::setValue](Arduino%20BLE%20Libraries.md/#blecharacteristicsetvalue)
```cpp
void BLECharacteristic::setValue(String value) {
  setValue((uint8_t *)(value.c_str()), value.length()); //linked
}  // setValue
```

###### BLECharacteristic::setValue
[BLEValue](Arduino%20BLE%20Libraries.md/#blevalue)  
[BLEValue::setValue](Arduino%20BLE%20Libraries.md/#blevaluesetvalue)
```cpp
void BLECharacteristic::setValue(uint8_t *data, size_t length) {
// The call to BLEUtils::buildHexData() doesn't output anything if the log level is not
// "VERBOSE". As it is quite CPU intensive, it is much better to not call it if not needed.
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE
  char *pHex = BLEUtils::buildHexData(nullptr, data, length);
  log_v(">> setValue: length=%d, data=%s, characteristic UUID=%s", length, pHex, getUUID().toString().c_str());
  free(pHex);
#endif
  if (length > ESP_GATT_MAX_ATTR_LEN) {
    log_e("Size %d too large, must be no bigger than %d", length, ESP_GATT_MAX_ATTR_LEN);
    return;
  }
  m_semaphoreSetValue.take();
  m_value.setValue(data, length); //linked m_value; and linked
  m_semaphoreSetValue.give();
  log_v("<< setValue");
}  // setValue
```

###### BLEValue::setValue
```cpp
void BLEValue::setValue(String value) {
  m_value = value;
}  // setValue
```

###### BLEservice->start()
[BLECharacteristicMap](Arduino%20BLE%20Libraries.md/#blecharacteristicmap)  
[BLECharacteristicMap::getFirst()](Arduino%20BLE%20Libraries.md/#blecharacteristicmapgetfirst)  
[BLECharacteristic::executeCreate](Arduino%20BLE%20Libraries.md/#blecharacteristicexecutecreate)  
[BLECharacteristicMap::getNext](Arduino%20BLE%20Libraries.md/#blecharacteristicmapgetnext)  
[esp_ble_gatts_start_service](ESP-IDF.md/#esp_ble_gatts_start_service)
```cpp
void BLEService::start() {
  // We ask the BLE runtime to start the service and then create each of the characteristics.
  // We start the service through its local handle which was returned in the ESP_GATTS_CREATE_EVT event
  // obtained as a result of calling esp_ble_gatts_create_service().
  //
  log_v(">> start(): Starting service (esp_ble_gatts_start_service): %s", toString().c_str());
  if (m_handle == NULL_HANDLE) {
    log_e("<< !!! We attempted to start a service but don't know its handle!");
    return;
  }

  BLECharacteristic *pCharacteristic = m_characteristicMap.getFirst(); //linked m_characteristicMap; and linked

  while (pCharacteristic != nullptr) {
    m_lastCreatedCharacteristic = pCharacteristic;
    pCharacteristic->executeCreate(this); //linked

    pCharacteristic = m_characteristicMap.getNext(); //linked
  }
  // Start each of the characteristics ... these are found in the m_characteristicMap.

  m_semaphoreStartEvt.take("start");
  esp_err_t errRc = ::esp_ble_gatts_start_service(m_handle); //linked

  if (errRc != ESP_OK) {
    log_e("<< esp_ble_gatts_start_service: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
    return;
  }
  m_semaphoreStartEvt.wait("start");

  log_v("<< start()");
}  // start

```

###### BLECharacteristic::executeCreate
[esp_attr_control_t](ESP-IDF.md/#esp_attr_control_t)  
[esp_gatt_perm_t](ESP-IDF.md/#esp_gatt_perm_t)  
[BLEDescriptor::BLEDescriptor](Arduino%20BLE%20Libraries.md/#bledescriptorbledescriptor)  
[BLEDescriptorMap](Arduino%20BLE%20Libraries.md/#bledescriptormap)  
[BLEDescriptor::executeCreate](Arduino%20BLE%20Libraries.md/#bledescriptorexecutecreate)
```cpp
void BLECharacteristic::executeCreate(BLEService *pService) {
  log_v(">> executeCreate()");

  if (m_handle != NULL_HANDLE) {
    log_e("Characteristic already has a handle.");
    return;
  }

  m_pService = pService;  // Save the service to which this characteristic belongs.

  log_d("Registering characteristic (esp_ble_gatts_add_char): uuid: %s, service: %s", getUUID().toString().c_str(), m_pService->toString().c_str());

  esp_attr_control_t control; //linked
  control.auto_rsp = ESP_GATT_RSP_BY_APP;

  m_semaphoreCreateEvt.take("executeCreate");
  esp_err_t errRc = ::esp_ble_gatts_add_char(
    m_pService->getHandle(), getUUID().getNative(), static_cast<esp_gatt_perm_t>(m_permissions), getProperties(), nullptr,
    &control
  );  // Whether to auto respond or not.
  //linked

  if (errRc != ESP_OK) {
    log_e("<< esp_ble_gatts_add_char: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
    return;
  }
  m_semaphoreCreateEvt.wait("executeCreate");

  BLEDescriptor *pDescriptor = m_descriptorMap.getFirst(); //linked and linked m_descriptorMap;
  while (pDescriptor != nullptr) {
    pDescriptor->executeCreate(this); //linked
    pDescriptor = m_descriptorMap.getNext();
  }  // End while

  log_v("<< executeCreate");
}  // executeCreate
```

###### BLEDescriptor::executeCreate
[esp_ble_gatts_add_char_descr](ESP-IDF.md/#esp_ble_gatts_add_char_descr)
```cpp
void BLEDescriptor::executeCreate(BLECharacteristic *pCharacteristic) {
  log_v(">> executeCreate(): %s", toString().c_str());

  if (m_handle != NULL_HANDLE) {
    log_e("Descriptor already has a handle.");
    return;
  }

  m_pCharacteristic = pCharacteristic;  // Save the characteristic associated with this service.

  esp_attr_control_t control;
  control.auto_rsp = ESP_GATT_AUTO_RSP;
  m_semaphoreCreateEvt.take("executeCreate");
  esp_err_t errRc =
    ::esp_ble_gatts_add_char_descr(pCharacteristic->getService()->getHandle(), getUUID().getNative(), (esp_gatt_perm_t)m_permissions, &m_value, &control); //linked
  if (errRc != ESP_OK) {
    log_e("<< esp_ble_gatts_add_char_descr: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
    return;
  }

  m_semaphoreCreateEvt.wait("executeCreate");
  log_v("<< executeCreate");
}  // executeCreate

```

###### BLEDescriptorMap
```cpp
class BLEDescriptorMap {
public:
  void setByUUID(const char *uuid, BLEDescriptor *pDescriptor);
  void setByUUID(BLEUUID uuid, BLEDescriptor *pDescriptor);
  void setByHandle(uint16_t handle, BLEDescriptor *pDescriptor);
  BLEDescriptor *getByUUID(const char *uuid);
  BLEDescriptor *getByUUID(BLEUUID uuid);
  BLEDescriptor *getByHandle(uint16_t handle);
  String toString();
  void handleGATTServerEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
  BLEDescriptor *getFirst();
  BLEDescriptor *getNext();

private:
  std::map<BLEDescriptor *, String> m_uuidMap;
  std::map<uint16_t, BLEDescriptor *> m_handleMap;
  std::map<BLEDescriptor *, String>::iterator m_iterator;
};
```

###### BLEDescriptor::BLEDescriptor
```cpp
BLEDescriptor::BLEDescriptor(BLEUUID uuid, uint16_t max_len) {
  m_bleUUID = uuid;
  m_value.attr_len = 0;            // Initial length is 0.
  m_value.attr_max_len = max_len;  // Maximum length of the data.
  m_handle = NULL_HANDLE;          // Handle is initially unknown.
  m_pCharacteristic = nullptr;     // No initial characteristic.
  m_pCallback = nullptr;           // No initial callback.

  m_value.attr_value = (uint8_t *)malloc(max_len);  // Allocate storage for the value.
}  // BLEDescriptor
```

###### BLECharacteristicMap
```cpp
class BLECharacteristicMap {
public:
  void setByUUID(BLECharacteristic *pCharacteristic, const char *uuid);
  void setByUUID(BLECharacteristic *pCharacteristic, BLEUUID uuid);
  void setByHandle(uint16_t handle, BLECharacteristic *pCharacteristic);
  BLECharacteristic *getByUUID(const char *uuid);
  BLECharacteristic *getByUUID(BLEUUID uuid);
  BLECharacteristic *getByHandle(uint16_t handle);
  BLECharacteristic *getFirst();
  BLECharacteristic *getNext();
  String toString();
  void handleGATTServerEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

private:
  std::map<BLECharacteristic *, String> m_uuidMap;
  std::map<uint16_t, BLECharacteristic *> m_handleMap;
  std::map<BLECharacteristic *, String>::iterator m_iterator;
};
```

###### BLECharacteristicMap::getFirst()
```cpp
BLECharacteristic *BLECharacteristicMap::getFirst() {
  m_iterator = m_uuidMap.begin();
  if (m_iterator == m_uuidMap.end()) {
    return nullptr;
  }
  BLECharacteristic *pRet = m_iterator->first;
  m_iterator++;
  return pRet;
}  // getFirst
```

###### BLECharacteristicMap::getNext
```cpp
BLECharacteristic *BLECharacteristicMap::getNext() {
  if (m_iterator == m_uuidMap.end()) { //std::map<BLECharacteristic *, String> m_uuidMap;
    return nullptr;
  }
  BLECharacteristic *pRet = m_iterator->first;
  m_iterator++;
  return pRet;
}  // getNext
```

###### BLEDevice::getAdvertising
[BLEAdvertising::BLEAdvertising()](Arduino%20BLE%20Libraries.md/#bleadvertisingbleadvertising)
```cpp
BLEAdvertising *BLEDevice::getAdvertising() {
  if (m_bleAdvertising == nullptr) {
    m_bleAdvertising = new BLEAdvertising(); //linked
    log_i("create advertising");
  }
  log_d("get advertising");
  return m_bleAdvertising;
}
```

###### BLEAdvertising::BLEAdvertising()
[esp_ble_adv_data_t](ESP-IDF.md/#esp_ble_adv_data_type)  
[esp_ble_adv_params_t](ESP-IDF.md/#esp_ble_adv_params_t)
```cpp
BLEAdvertising::BLEAdvertising() : m_scanRespData{} {
  m_advData.set_scan_rsp = false; //linked m_advData;
  m_advData.include_name = true;
  m_advData.include_txpower = true;
  m_advData.min_interval = 0x20;
  m_advData.max_interval = 0x40;
  m_advData.appearance = 0x00;
  m_advData.manufacturer_len = 0;
  m_advData.p_manufacturer_data = nullptr;
  m_advData.service_data_len = 0;
  m_advData.p_service_data = nullptr;
  m_advData.service_uuid_len = 0;
  m_advData.p_service_uuid = nullptr;
  m_advData.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

  m_advParams.adv_int_min = 0x20; //linked m_advParams;
  m_advParams.adv_int_max = 0x40;
  m_advParams.adv_type = ADV_TYPE_IND;
  m_advParams.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  m_advParams.channel_map = ADV_CHNL_ALL;
  m_advParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
  m_advParams.peer_addr_type = BLE_ADDR_TYPE_PUBLIC;

  m_customAdvData = false;           // No custom advertising data
  m_customScanResponseData = false;  // No custom scan response data
}  // BLEAdvertising
```

###### BLEAdvertising->addServiceUUID()
```cpp
void BLEAdvertising::addServiceUUID(BLEUUID serviceUUID) {
  m_serviceUUIDs.push_back(serviceUUID); //std::vector<BLEUUID> m_serviceUUIDs;
}  // addServiceUUID
```

###### BLEService->getUUID()
```cpp
BLEUUID BLEService::getUUID() {
  return m_uuid;
}  // getUUID
```

###### BLEAdvertising->setScanResponse()
```cpp
void BLEAdvertising::setScanResponse(bool set) {
  m_scanResp = set;
}
```

###### BLEAdvertising->setMin/MaxPreferred()
```cpp
void BLEAdvertising::setMinPreferred(uint16_t mininterval) {
  m_advData.min_interval = mininterval;
}  //

void BLEAdvertising::setMaxPreferred(uint16_t maxinterval) {
  m_advData.max_interval = maxinterval;
}  //
```

###### BLEDevice::startAdvertising()
[BLEDevice::getAdvertising](Arduino%20BLE%20Libraries.md/#bledevicegetadvertising)
```cpp
void BLEDevice::startAdvertising() {
  log_v(">> startAdvertising");
  getAdvertising()->start(); //linked
  log_v("<< startAdvertising");
}  // startAdvertising
```

###### BLEDevice::getAdvertising
[BLEAdvertising::BLEAdvertising()](Arduino%20BLE%20Libraries.md/#bleadvertisingbleadvertising)
```cpp
BLEAdvertising *BLEDevice::getAdvertising() {
  if (m_bleAdvertising == nullptr) {
    m_bleAdvertising = new BLEAdvertising(); //linked
    log_i("create advertising");
  }
  log_d("get advertising");
  return m_bleAdvertising;
}
```

###### BLECharacteristic->notify()
[BLECharacteristic::getDescriptorByUUID](Arduino%20BLE%20Libraries.md/#blecharacteristicgetdescriptorbyuuid)  
[BLE2902::getNotifications](Arduino%20BLE%20Libraries.md/#ble2902getnotifications)  
[BLE2902::getIndications](Arduino%20BLE%20Libraries.md/#ble2902getindications)  
[BLEServer::getPeerDevices](Arduino%20BLE%20Libraries.md/#bleservergetpeerdevices)  
[esp_ble_gatts_send_indicate](ESP-IDF.md/#esp_ble_gatts_send_indicate)  
[esp_gatt_status_t](ESP-IDF.md/#esp_gatt_status_t)
```cpp
void BLECharacteristic::notify(bool is_notification) {
  log_v(">> notify: length: %d", m_value.getValue().length());

  assert(getService() != nullptr);
  assert(getService()->getServer() != nullptr);

  m_pCallbacks->onNotify(this);  // Invoke the notify callback.

  // GeneralUtils::hexDump() doesn't output anything if the log level is not
  // "VERBOSE". Additionally, it is very CPU intensive, even when it doesn't
  // output anything! So it is much better to *not* call it at all if not needed.
  // In a simple program which calls BLECharacteristic::notify() every 50 ms,
  // the performance gain of this little optimization is 37% in release mode
  // (-O3) and 57% in debug mode.
  // Of course, the "#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE" guard
  // could also be put inside the GeneralUtils::hexDump() function itself. But
  // it's better to put it here also, as it is clearer (indicating a verbose log
  // thing) and it allows to remove the "m_value.getValue().c_str()" call, which
  // is, in itself, quite CPU intensive.
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE
  GeneralUtils::hexDump((uint8_t *)m_value.getValue().c_str(), m_value.getValue().length());
#endif

  if (getService()->getServer()->getConnectedCount() == 0) {
    log_v("<< notify: No connected clients.");
    m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_NO_CLIENT, 0);
    return;
  }

  // Test to see if we have a 0x2902 descriptor.  If we do, then check to see if notification is enabled
  // and, if not, prevent the notification.

  BLE2902 *p2902 = (BLE2902 *)getDescriptorByUUID((uint16_t)0x2902); 
  //linked
  if (is_notification) {
    if (p2902 != nullptr && !p2902->getNotifications()) { 
    //linked
      log_v("<< notifications disabled; ignoring");
      m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_NOTIFY_DISABLED, 0);  // Invoke the notify callback.
      return;
    }
  } else {
    if (p2902 != nullptr && !p2902->getIndications()) { //linked
      log_v("<< indications disabled; ignoring");
      m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_INDICATE_DISABLED, 0);  // Invoke the notify callback.
      return;
    }
  }
  for (auto &myPair : getService()->getServer()->getPeerDevices(false)) { 
  // linked
    uint16_t _mtu = (myPair.second.mtu);
    if (m_value.getValue().length() > _mtu - 3) {
      log_w("- Truncating to %d bytes (maximum notify size)", _mtu - 3);
    }

    size_t length = m_value.getValue().length();
    if (!is_notification) {  // is indication
      m_semaphoreConfEvt.take("indicate");
    }
    esp_err_t errRc = ::esp_ble_gatts_send_indicate( 
    //linked
      getService()->getServer()->getGattsIf(), myPair.first, getHandle(), length, (uint8_t *)m_value.getValue().c_str(), !is_notification
    );  // The need_confirm = false makes this a notify.
    if (errRc != ESP_OK) {
      log_e("<< esp_ble_gatts_send_ %s: rc=%d %s", is_notification ? "notify" : "indicate", errRc, GeneralUtils::errorToString(errRc));
      m_semaphoreConfEvt.give();
      m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_GATT, errRc);  // Invoke the notify callback.
      return;
    }
    if (!is_notification) {  // is indication
      if (!m_semaphoreConfEvt.timedWait("indicate", indicationTimeout)) {
        m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_INDICATE_TIMEOUT, 0);  // Invoke the notify callback.
      } else {
        auto code = (esp_gatt_status_t)m_semaphoreConfEvt.value(); //linked
        if (code == ESP_GATT_OK) {
          m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::SUCCESS_INDICATE, code);  // Invoke the notify callback.
        } else {
          m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::ERROR_INDICATE_FAILURE, code);
        }
      }
    } else {
      m_pCallbacks->onStatus(this, BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY, 0);  // Invoke the notify callback.
    }
  }
  log_v("<< notify");
}  // Notify
```

###### BLECharacteristic::getDescriptorByUUID
```cpp
BLEDescriptor *BLECharacteristic::getDescriptorByUUID(BLEUUID descriptorUUID) {
  return m_descriptorMap.getByUUID(descriptorUUID);
}  // getDescriptorByUUID
```

###### BLE2902::getNotifications
```cpp
bool BLE2902::getNotifications() {
  return (getValue()[0] & (1 << 0)) != 0;
}  // getNotifications
```

###### BLE2902::getIndications
```cpp
bool BLE2902::getIndications() {
  return (getValue()[0] & (1 << 1)) != 0;
}  // getIndications
```

###### BLEServer::getPeerDevices
```cpp
std::map<uint16_t, conn_status_t> BLEServer::getPeerDevices(bool _client) {
  return m_connectedServersMap; //std::map<uint16_t, conn_status_t> m_connectedServersMap;
}
```