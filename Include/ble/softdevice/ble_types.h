/* Copyright (c) 2011 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */
/**
  @addtogroup BLE_COMMON
  @{
  @defgroup ble_types Common types and macro definitions.
  @{

  @brief Common types and macro definitions for the S110 Softdevice.
 */

#ifndef BLE_TYPES_H__
#define BLE_TYPES_H__

#include <stdint.h>

/** @defgroup BLE_CONN_HANDLES BLE Connection Handles.
 * @{ */
#define BLE_CONN_HANDLE_INVALID 0xFFFF  /**< Invalid Connection Handle. */
#define BLE_CONN_HANDLE_ALL     0xFFFE  /**< Applies to all Connection Handles. */
/** @} */


/** @defgroup BLE_UUID_VALUES Assigned Values for BLE UUIDs.
 * @{ */
/* Generic UUIDs, applicable to all services */
#define BLE_UUID_UNKNOWN                              0x0000 /**< Reserved UUID. */
#define BLE_UUID_SERVICE_PRIMARY                      0x2800 /**< Primary Service. */
#define BLE_UUID_SERVICE_SECONDARY                    0x2801 /**< Secondary Service. */
#define BLE_UUID_SERVICE_INCLUDE                      0x2802 /**< Include. */
#define BLE_UUID_CHARACTERISTIC                       0x2803 /**< Characteristic. */
#define BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP             0x2900 /**< Characteristic Extended Properties Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_USER_DESC            0x2901 /**< Characteristic User Description Descriptor. */
#define BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG        0x2902 /**< Client Characteristic Configuration Descriptor. */
#define BLE_UUID_DESCRIPTOR_SERVER_CHAR_CONFIG        0x2903 /**< Server Characteristic Configuration Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT  0x2904 /**< Characteristic Presentation Format Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_AGGREGATE_FORMAT     0x2905 /**< Characteristic Aggregate Format Descriptor. */
/* GATT specific UUIDs */
#define BLE_UUID_GATT                                 0x1801 /**< Generic Attribute Profile. */
#define BLE_UUID_GATT_CHARACTERISTIC_SERVICE_CHANGED  0x2A05 /**< Service Changed Characteristic. */
/* GAP specific UUIDs */
#define BLE_UUID_GAP                                  0x1800 /**< Generic Access Profile. */
#define BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME       0x2A00 /**< Device Name Characteristic. */
#define BLE_UUID_GAP_CHARACTERISTIC_APPEARANCE        0x2A01 /**< Appearance Characteristic. */
#define BLE_UUID_GAP_CHARACTERISTIC_PPF               0x2A02 /**< Peripheral Privacy Flag Characteristic. */
#define BLE_UUID_GAP_CHARACTERISTIC_RECONN_ADDR       0x2A03 /**< Reconnection Address Characteristic. */
#define BLE_UUID_GAP_CHARACTERISTIC_PPCP              0x2A04 /**< Peripheral Preferred Connection Parameters Characteristic. */
/** @} */


/** @defgroup BLE_UUID_TYPES Types of UUID.
 * @{ */
#define BLE_UUID_TYPE_UNKNOWN       0x00 /**< Invalid UUID base. */
#define BLE_UUID_TYPE_BLE           0x01 /**< Bluetooth SIG UUID base. */
#define BLE_UUID_TYPE_VENDOR_BEGIN  0x02 /**< Vendor UUID bases start at this index. */
/** @} */


/** @defgroup BLE_GATT_CPF_FORMATS Characteristic Presentation Formats.
 *  @note Found at http://developer.bluetooth.org/gatt/descriptors/Pages/DescriptorViewer.aspx?u=org.bluetooth.descriptor.gatt.characteristic_presentation_format.xml
 * @{ */
#define BLE_GATT_CPF_FORMAT_RFU                 0x00 /**< Reserved For Future Use. */
#define BLE_GATT_CPF_FORMAT_BOOLEAN             0x01 /**< Boolean. */
#define BLE_GATT_CPF_FORMAT_2BIT                0x02 /**< Unsigned 2-bit integer. */
#define BLE_GATT_CPF_FORMAT_NIBBLE              0x03 /**< Unsigned 4-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT8               0x04 /**< Unsigned 8-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT12              0x05 /**< Unsigned 12-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT16              0x06 /**< Unsigned 16-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT24              0x07 /**< Unsigned 24-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT32              0x08 /**< Unsigned 32-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT48              0x09 /**< Unsigned 48-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT64              0x0A /**< Unsigned 64-bit integer. */
#define BLE_GATT_CPF_FORMAT_UINT128             0x0B /**< Unsigned 128-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT8               0x0C /**< Signed 2-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT12              0x0D /**< Signed 12-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT16              0x0E /**< Signed 16-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT24              0x0F /**< Signed 24-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT32              0x10 /**< Signed 32-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT48              0x11 /**< Signed 48-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT64              0x12 /**< Signed 64-bit integer. */
#define BLE_GATT_CPF_FORMAT_SINT128             0x13 /**< Signed 128-bit integer. */
#define BLE_GATT_CPF_FORMAT_FLOAT32             0x14 /**< IEEE-754 32-bit floating point. */
#define BLE_GATT_CPF_FORMAT_FLOAT64             0x15 /**< IEEE-754 64-bit floating point. */
#define BLE_GATT_CPF_FORMAT_SFLOAT              0x16 /**< IEEE-11073 16-bit SFLOAT. */
#define BLE_GATT_CPF_FORMAT_FLOAT               0x17 /**< IEEE-11073 32-bit FLOAT. */
#define BLE_GATT_CPF_FORMAT_DUINT16             0x18 /**< IEEE-20601 format. */
#define BLE_GATT_CPF_FORMAT_UTF8S               0x19 /**< UTF-8 string. */
#define BLE_GATT_CPF_FORMAT_UTF16S              0x1A /**< UTF-16 string. */
#define BLE_GATT_CPF_FORMAT_STRUCT              0x1B /**< Opaque Structure. */
/** @} */

/** @defgroup BLE_GATT_CPF_NAMESPACES GATT Bluetooth Namespaces.
 * @{
 * @ingroup BLE_GATTS
 */
#define BLE_GATT_CPF_NAMESPACE_BTSIG            0x01

#define BLE_GATT_CPF_NAMESPACE_DESCRIPTION_UNKNOWN 0x0000
/** @} */


/** @defgroup BLE_APPEARANCES Bluetooth Appearance values.
 *  @note Retrieved from http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
 * @{ */
#define BLE_APPEARANCE_UNKNOWN                              0   /**< Unknown. */
#define BLE_APPEARANCE_GENERIC_PHONE                        64  /**< Generic Phone. */
#define BLE_APPEARANCE_GENERIC_COMPUTER                     128 /**< Generic Computer. */
#define BLE_APPEARANCE_GENERIC_WATCH                        192 /**< Generic Watch. */
#define BLE_APPEARANCE_WATCH_SPORTS_WATCH                   193 /**< Watch: Sports Watch. */
#define BLE_APPEARANCE_GENERIC_CLOCK                        256 /**< Generic Clock. */
#define BLE_APPEARANCE_GENERIC_DISPLAY                      320 /**< Generic Display. */
#define BLE_APPEARANCE_GENERIC_REMOTE_CONTROL               384 /**< Generic Remote Control. */
#define BLE_APPEARANCE_GENERIC_EYE_GLASSES                  448 /**< Generic Eye-glasses. */
#define BLE_APPEARANCE_GENERIC_TAG                          512 /**< Generic Tag. */
#define BLE_APPEARANCE_GENERIC_KEYRING                      576 /**< Generic Keyring. */
#define BLE_APPEARANCE_GENERIC_MEDIA_PLAYER                 640 /**< Generic Media Player. */
#define BLE_APPEARANCE_GENERIC_BARCODE_SCANNER              704 /**< Generic Barcode Scanner. */
#define BLE_APPEARANCE_GENERIC_THERMOMETER                  768 /**< Generic Thermometer. */
#define BLE_APPEARANCE_THERMOMETER_EAR                      769 /**< Thermometer: Ear. */
#define BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR            832 /**< Generic Heart rate Sensor. */
#define BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT    833 /**< Heart Rate Sensor: Heart Rate Belt. */
#define BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE               896 /**< Generic Blood Pressure. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_ARM                   897 /**< Blood Pressure: Arm. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_WRIST                 898 /**< Blood Pressure: Wrist. */
#define BLE_APPEARANCE_GENERIC_HID                          960 /**< Human Interface Device (HID). */
#define BLE_APPEARANCE_HID_KEYBOARD                         961 /**< Keyboard (HID Subtype). */
#define BLE_APPEARANCE_HID_MOUSE                            962 /**< Mouse (HID Subtype). */
#define BLE_APPEARANCE_HID_JOYSTICK                         963 /**< Joystiq (HID Subtype). */
#define BLE_APPEARANCE_HID_GAMEPAD                          964 /**< Gamepad (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITIZERSUBTYPE                 965 /**< Digitizer Tablet (HID Subtype). */
#define BLE_APPEARANCE_HID_CARD_READER                      966 /**< Card Reader (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITAL_PEN                      967 /**< Digital Pen (HID Subtype). */
#define BLE_APPEARANCE_HID_BARCODE                          968 /**< Barcode Scanner (HID Subtype). */
#define BLE_APPEARANCE_GENERIC_GLUCOSE_METER               1024 /**< Generic Glucose Meter. */
#define BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR      1088 /**< Generic Running Walking Sensor. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE      1089 /**< Running Walking Sensor: In-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE      1090 /**< Running Walking Sensor: On-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP       1091 /**< Running Walking Sensor: On-Hip. */
#define BLE_APPEARANCE_GENERIC_CYCLING                     1152 /**< Generic Cycling. */
#define BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER            1153 /**< Cycling: Cycling Computer. */
#define BLE_APPEARANCE_CYCLING_SPEED_SENSOR                1154 /**< Cycling: Speed Sensor. */
#define BLE_APPEARANCE_CYCLING_CADENCE_SENSOR              1155 /**< Cycling: Cadence Sensor. */
#define BLE_APPEARANCE_CYCLING_POWER_SENSOR                1156 /**< Cycling: Power Sensor. */
#define BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR        1157 /**< Cycling: Speed and Cadence Sensor. */
/** @} */

/** @brief 128 bit UUID values. */
typedef struct
{ 
    unsigned char uuid128[16];
} ble_uuid128_t;

/** @brief  Bluetooth Low Energy UUID type, encapsulates both 16-bit and 128-bit UUIDs. */
typedef struct
{
    uint16_t    uuid; /**< 16-bit UUID value or octets 12-13 of 128-bit UUID. */
    uint8_t     type; /**< 1 (16-bit UUID) or base index, 2.. (128-bit UUID). See @ref BLE_UUID_TYPES. */
} ble_uuid_t;


/** @brief Set .type and .uuid fields of ble_uuid_struct to specified uuid value. */
#define BLE_UUID_BLE_ASSIGN(instance, value) do {\
            instance.type = BLE_UUID_TYPE_BLE; \
            instance.uuid = value;} while(0)

/** @brief Copy type and uuid members from src to dst ble_uuid_t pointer. Both pointers must be valid/non-null. */
#define BLE_UUID_COPY_PTR(dst, src) do {\
            (dst)->type = (src)->type; \
            (dst)->uuid = (src)->uuid;} while(0)

/** @brief Copy type and uuid members from src to dst ble_uuid_t struct. */
#define BLE_UUID_COPY_INST(dst, src) do {\
            (dst).type = (src).type; \
            (dst).uuid = (src).uuid;} while(0)

/** @brief Compare for equality both type and uuid members of two (valid, non-null) ble_uuid_t pointers. */
#define BLE_UUID_EQ(p_uuid1, p_uuid2) \
            (((p_uuid1)->type == (p_uuid2)->type) && ((p_uuid1)->uuid == (p_uuid2)->uuid))

/** @brief Compare for difference both type and uuid members of two (valid, non-null) ble_uuid_t pointers. */
#define BLE_UUID_NEQ(p_uuid1, p_uuid2) \
            (((p_uuid1)->type != (p_uuid2)->type) || ((p_uuid1)->uuid != (p_uuid2)->uuid))



#endif /* BLE_TYPES_H__ */

/**
  @}
  @}
*/
