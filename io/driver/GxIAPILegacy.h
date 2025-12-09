/**
@File      GxLegacy.h
@Brief     This header file contains legacy data structures and definitions
           that are maintained for compatibility with older versions of the software.
@Author    Software Department
@Date      2024-11-13
@Version   1.0.2411.9131
 */

#ifndef GX_LEGACY_H
#define GX_LEGACY_H

#include "GXDef.h"

//***************************************************
//Chinese： 不建议使用的功能和定义
//English： Features and definitions not recommended
//***************************************************

typedef enum GX_ACCESS_STATUS
{
	GX_ACCESS_STATUS_UNKNOWN	= 0,						///< \Chinese 设备当前状态未知																\English The device's current status is unknown
	GX_ACCESS_STATUS_READWRITE  = 1,						///< \Chinese 设备当前可读可写																\English The device currently supports reading and writing
	GX_ACCESS_STATUS_READONLY   = 2,						///< \Chinese 设备当前只支持读																\English The device currently only supports reading
	GX_ACCESS_STATUS_NOACCESS   = 3,						///< \Chinese 设备当前既不支持读，又不支持写												\English The device currently does neither support reading nor support writing
}GX_ACCESS_STATUS;
typedef int32_t GX_ACCESS_STATUS_CMD;

typedef struct GX_INT_RANGE
{
	int64_t nMin;										///< \Chinese 整型值最小值										\English Minimum value
	int64_t nMax;										///< \Chinese 整型值最大值										\English Maximum value
	int64_t nInc;										///< \Chinese 整型值步长										\English Step size
	int32_t reserved[8];								///< \Chinese 保留												\English 32 bytes,reserved
}GX_INT_RANGE;

typedef struct GX_FLOAT_RANGE
{
	double  dMin;										///< \Chinese 浮点型最小值										\English Minimum value
	double  dMax;										///< \Chinese 浮点型最大值										\English Maximum value
	double  dInc;										///< \Chinese 浮点型步长										\English Step size
	char    szUnit[GX_INFO_LENGTH_8_BYTE];				///< \Chinese 浮点型单位										\English Unit. 8 bytes
	bool    bIncIsValid;								///< \Chinese 步长是否有效										\English Indicates whether the step size is supported, 1 byte
	int8_t  reserved[31];								///< \Chinese 保留												\English 31 bytes,reserved
}GX_FLOAT_RANGE;

typedef struct GX_ENUM_DESCRIPTION
{
	int64_t nValue;										///< \Chinese 枚举值											\English The value of the enumeration item
	char    szSymbolic[GX_INFO_LENGTH_64_BYTE];			///< \Chinese 字符描述											\English The character description information of the enumeration item, 64 bytes
	int32_t reserved[8];								///< \Chinese 保留												\English 32 bytes,reserved
}GX_ENUM_DESCRIPTION;

typedef struct GX_DEVICE_IP_INFO
{
	char szDeviceID[GX_INFO_LENGTH_64_BYTE + 4];         ///< \Chinese 设备唯一标识,如果实际长度超过64字节有效字符串，则只保留64个有效字符			\English the unique identifier of the device, 64+4 bytes
	char szMAC[GX_INFO_LENGTH_32_BYTE];                  ///< \Chinese MAC地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符				\English MAC address, 32 bytes,
	char szIP[GX_INFO_LENGTH_32_BYTE];                   ///< \Chinese IP地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符				\English IP address, 32 bytes,
	char szSubNetMask[GX_INFO_LENGTH_32_BYTE];           ///< \Chinese 子网掩码,如果实际长度超过32字节有效字符串，则只保留31个有效字符				\English subnet mask, 32 bytes
	char szGateWay[GX_INFO_LENGTH_32_BYTE];              ///< \Chinese 网关,如果实际长度超过32字节有效字符串，则只保留31个有效字符					\English gateway, 32 bytes
	char szNICMAC[GX_INFO_LENGTH_32_BYTE];               ///< \Chinese 对应网卡的MAC地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符		\English the MAC address of the corresponding NIC(Network Interface Card), 32 bytes
	char szNICIP[GX_INFO_LENGTH_32_BYTE];                ///< \Chinese 对应网卡的IP地址,如果实际长度超过32字节有效字符串，则只保留31个有效字符		\English the IP address of the corresponding NIC, 32 bytes
	char szNICSubNetMask[GX_INFO_LENGTH_32_BYTE];        ///< \Chinese 对应网卡的子网掩码,如果实际长度超过32字节有效字符串，则只保留31个有效字符	\English the subnet mask of the corresponding NIC, 32 bytes
	char szNICGateWay[GX_INFO_LENGTH_32_BYTE];           ///< \Chinese 对应网卡的网关,如果实际长度超过32字节有效字符串，则只保留31个有效字符		\English the gateway of the corresponding NIC, 32 bytes
	char szNICDescription[GX_INFO_LENGTH_128_BYTE + 4];  ///< \Chinese 对应网卡描述,如果实际长度超过128字节有效字符串，则只保留128个有效字符		\English the description of the corresponding NIC, 128+4 bytes
	char reserved[512];                                  ///< \Chinese 保留																			\English reserved, 512 bytes
}GX_DEVICE_IP_INFO;

typedef struct GX_DEVICE_BASE_INFO
{
	char szVendorName[GX_INFO_LENGTH_32_BYTE];           ///< \Chinese 厂商名称,如果实际长度超过32字节有效字符串，则只保留31个有效字符				\English vendor name, 32 bytes
	char szModelName[GX_INFO_LENGTH_32_BYTE];            ///< \Chinese 设备类型名称,如果实际长度超过32字节有效字符串，则只保留31个有效字符			\English model name, 32 bytes
	char szSN[GX_INFO_LENGTH_32_BYTE];                   ///< \Chinese 设备序列号,如果实际长度超过32字节有效字符串，则只保留31个有效字符			\English device serial number, 32 bytes
	char szDisplayName[GX_INFO_LENGTH_128_BYTE + 4];     ///< \Chinese 设备展示名称,如果实际长度超过128字节有效字符串，则只保留128个有效字符		\English device display name, 128+4 bytes
	char szDeviceID[GX_INFO_LENGTH_64_BYTE + 4];         ///< \Chinese 设备唯一标识,如果实际长度超过64字节有效字符串，则只保留64个有效字符			\English the unique identifier of the device, 64+4 bytes
	char szUserID[GX_INFO_LENGTH_64_BYTE + 4];           ///< \Chinese 用户自定义名称,如果实际长度超过64字节有效字符串，则只保留64个有效字符		\English user-defined name, 64+4 bytes
	GX_ACCESS_STATUS_CMD  accessStatus;                  ///< \Chinese 设备当前支持的访问状态														\English access status that is currently supported by the device. Refer to GX_ACCESS_STATUS, 4 bytes
	GX_DEVICE_CLASS   deviceClass;                       ///< \Chinese 设备种类，比如USB2.0、GEV													\English device type, such as USB2.0, GEV, 4 bytes
	char reserved[300];                                  ///< \Chinese 保留																			\English reserved, 300 bytes
}GX_DEVICE_BASE_INFO;

//------------------------------------------------------------------------------
//Chinese：功能码名称定义
//English：feature name definition
//------------------------------------------------------------------------------
typedef struct GX_FEATURE_NAME
{
	char chFeatureName[GX_INFO_LENGTH_128_BYTE];	///> \Chinese 属性名称		\English Feature name
}GX_FEATURE_NAME;

//------------------------------------------------------------------------------
//Chinese：功能码类型定义
//English：Code type definition
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_TYPE
{
	GX_FEATURE_INT     = 0x10000000,   ///< \Chinese 整型数		\English Integer type
	GX_FEATURE_FLOAT   = 0X20000000,   ///< \Chinese 浮点数		\English Floating point type
	GX_FEATURE_ENUM    = 0x30000000,   ///< \Chinese 枚举		\English Enum type
	GX_FEATURE_BOOL    = 0x40000000,   ///< \Chinese 布尔		\English Boolean type
	GX_FEATURE_STRING  = 0x50000000,   ///< \Chinese 字符串		\English String type
	GX_FEATURE_BUFFER  = 0x60000000,   ///< \Chinese buffer		\English Block data type
	GX_FEATURE_COMMAND = 0x70000000,   ///< \Chinese 命令		\English Command type
}GX_FEATURE_TYPE;

//------------------------------------------------------------------------------
//Chinese：功能码所属层级定义
//English：Hierarchy of functional codes
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_LEVEL
{
	GX_FEATURE_LEVEL_REMOTE_DEV = 0x00000000,   ///< \Chinese RemoteDevice层									\English Remote device layer
	GX_FEATURE_LEVEL_TL         = 0x01000000,   ///< \Chinese TL层												\English TL layer
	GX_FEATURE_LEVEL_IF         = 0x02000000,   ///< \Chinese Interface层										\English Interface layer
	GX_FEATURE_LEVEL_DEV        = 0x03000000,   ///< \Chinese Device层											\English Device layer
	GX_FEATURE_LEVEL_DS         = 0x04000000,   ///< \Chinese DataStream层										\English DataStream layer
}GX_FEATURE_LEVEL;

//------------------------------------------------------------------------------
//Chinese：功能码定义
//English：functional codes definition
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_ID
{
	//////////////////////////////////////////////////////////////////////////
	///Chinese	远端设备层
	///English	Remote Device Feature
	//////////////////////////////////////////////////////////////////////////

	//---------------DeviceInfomation Section--------------------------
	GX_STRING_DEVICE_VENDOR_NAME              = 0   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 厂商名称															\English Name of the manufacturer of the device.
	GX_STRING_DEVICE_MODEL_NAME               = 1   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备型号															\English Model of the device.
	GX_STRING_DEVICE_FIRMWARE_VERSION         = 2   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备固件版本														\English Version of the firmware in the device.
	GX_STRING_DEVICE_VERSION                  = 3   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备版本															\English Version of the device.
	GX_STRING_DEVICE_SERIAL_NUMBER            = 4   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备序列号														\English Device serial number.
	GX_STRING_FACTORY_SETTING_VERSION         = 6   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 出厂参数版本														\English Factory parameter version
	GX_STRING_DEVICE_USERID                   = 7   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 用户自定义名称													\English User-programmable device identifier.
	GX_INT_DEVICE_LINK_SELECTOR               = 8   | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备链路选择														\English Selects which Link of the device to control.
	GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE = 9   | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备带宽限制模式，参考GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY	\English Controls if the DeviceLinkThroughputLimit is active.
	GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT       = 10  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备链路带宽限制													\English Limits the maximum bandwidth of the data that will be streamed out by the device on the selected Link.
	GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT     = 11  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 当前设备采集带宽													\English The bandwidth of current device acquisition
	GX_COMMAND_DEVICE_RESET                   = 12  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备复位															\English reset device
	GX_INT_TIMESTAMP_TICK_FREQUENCY           = 13  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 时间戳频率														\English Time stamp clock frequency
	GX_COMMAND_TIMESTAMP_LATCH                = 14  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 时间戳锁存 														\English Timestamp latch
	GX_COMMAND_TIMESTAMP_RESET                = 15  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 重置时间戳														\English reset Timestamp
	GX_COMMAND_TIMESTAMP_LATCH_RESET          = 16  | GX_FEATURE_COMMAND| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 重置时间戳锁存													\English reset Timestamp latch
	GX_INT_TIMESTAMP_LATCH_VALUE              = 17  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 时间戳锁存值														\English Timestamp Latch value
	GX_STRING_DEVICE_PHY_VERSION              = 18  | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备网络芯片版本													\English Device network chip version
	GX_ENUM_DEVICE_TEMPERATURE_SELECTOR       = 19  | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备温度选择														\English Device temperature selection
	GX_FLOAT_DEVICE_TEMPERATURE               = 20  | GX_FEATURE_FLOAT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备温度															\English Device temperature
	GX_STRING_DEVICE_ISP_FIRMWARE_VERSION     = 21  | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备ISP固件版本													\English Device ISP firmware version
	GX_ENUM_LOWPOWER_MODE                     = 22  | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 低功耗模式,参考GX_LOWPOWER_MODE_ENTRY							\English Low power consumption mode,refer to GX_LOWPOWER_MODE_ENTRY
	GX_ENUM_CLOSE_CCD                         = 23  | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 关闭CCD,参考GX_CLOSE_CCD_ENTRY									\English Close CCD, refer to GX_CLOSE_CCD_ENTRY
	GX_STRING_PRODUCTION_CODE                 = 24  | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 生产代码															\English Production code
	GX_STRING_DEVICE_ORIGINAL_NAME            = 25  | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 原始名称															\English Original name
	GX_INT_REVISION                           = 26  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese CXP协议版本														\English CXP protocol version
	GX_INT_VERSIONS_SUPPORTED                 = 27  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 支持的CXP协议版本												\English Supported CXP protocol versions
	GX_INT_VERSION_USED                       = 28  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 使用版本															\English Use version
	GX_BOOL_TEC_ENABLE                        = 29  | GX_FEATURE_BOOL   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese TEC开关															\English TEC switch
	GX_FLOAT_TEC_TARGET_TEMPERATURE           = 30  | GX_FEATURE_FLOAT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese TEC目标温度														\English TEC target temperature
	GX_BOOL_FAN_ENABLE                        = 31  | GX_FEATURE_BOOL   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 风扇开关															\English Fan switch
	GX_INT_TEMPERATURE_DETECTION_STATUS       = 32  | GX_FEATURE_INT	| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 温度状态检测														\English Temperature state detection
	GX_INT_FAN_SPEED                          = 33  | GX_FEATURE_INT	| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 风扇转速															\English Fan speed
	GX_FLOAT_DEVICE_HUMIDITY                  = 34  | GX_FEATURE_FLOAT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备湿度 														\English Equipment humidity
	GX_FLOAT_DEVICE_PRESSURE                  = 35  | GX_FEATURE_FLOAT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备气压															\English Equipment air pressure
	GX_INT_AIR_CHANGE_DETECTION_STATUS        = 36  | GX_FEATURE_INT	| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 换气状态检测														\English Ventilation status detection
	GX_INT_AIR_TIGHTNESS_DETECTION_STATUS     = 37  | GX_FEATURE_INT	| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 气密性状态检测													\English Airtightness state detection
	GX_ENUM_DEVICE_SCAN_TYPE                  = 38  | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备扫描方式														\English Device scanning mode
	GX_INT_TECSELFTEST_FACTORY_STATUS         = 39  | GX_FEATURE_INT	| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 相机自检出厂状态                                                  \English Camera self inspection factory status

	//---------------ImageFormat Section--------------------------------
	GX_INT_SENSOR_WIDTH                     = 1000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 传感器宽度														\English Effective width of the sensor in pixels.
	GX_INT_SENSOR_HEIGHT                    = 1001 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 传感器高度														\English Effective height of the sensor in pixels.
	GX_INT_WIDTH_MAX                        = 1002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 最大宽度															\English Maximum width of the image(in pixels).
	GX_INT_HEIGHT_MAX                       = 1003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 最大高度															\English Maximum height of the image(in pixels).
	GX_INT_OFFSET_X                         = 1004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 水平偏移															\English Horizontal offset from the origin to the region of interest(in pixels).
	GX_INT_OFFSET_Y                         = 1005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 垂直偏移															\English Vertical offset from the origin to the region of interest(in pixels).
	GX_INT_WIDTH                            = 1006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 图像宽度															\English Width of the image provided by the device(in pixels).
	GX_INT_HEIGHT                           = 1007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 图像高度															\English Height of the image provided by the device(in pixels).
	GX_INT_BINNING_HORIZONTAL               = 1008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 水平像素Binning													\English Number of horizontal photo-sensitive cells to combine together.
	GX_INT_BINNING_VERTICAL                 = 1009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 垂直像素Binning													\English Number of vertical photo-sensitive cells to combine together.
	GX_INT_DECIMATION_HORIZONTAL            = 1010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 水平像素抽样														\English Horizontal sub-sampling of the image.
	GX_INT_DECIMATION_VERTICAL              = 1011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 垂直像素抽样														\English Vertical sub-sampling of the image.
	GX_ENUM_PIXEL_SIZE                      = 1012 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 像素位深,参考GX_PIXEL_SIZE_ENTRY									\English Total size in bits of a pixel of the image.
	GX_ENUM_PIXEL_COLOR_FILTER              = 1013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Bayer格式,参考GX_PIXEL_COLOR_FILTER_ENTRY							\English Type of color filter that is applied to the image.
	GX_ENUM_PIXEL_FORMAT                    = 1014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 像素格式,参考GX_PIXEL_FORMAT_ENTRY								\English Format of the pixels provided by the device.
	GX_BOOL_REVERSE_X                       = 1015 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 水平翻转															\English Flip horizontally the image sent by the device.
	GX_BOOL_REVERSE_Y                       = 1016 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 垂直翻转															\English Flip vertically the image sent by the device.
	GX_ENUM_TEST_PATTERN                    = 1017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 测试图,参考GX_TEST_PATTERN_ENTRY									\English Selects the type of test pattern that is generated by the device as image source.
	GX_ENUM_TEST_PATTERN_GENERATOR_SELECTOR = 1018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 测试图源选择，参考GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY		\English Selects which test pattern generator is controlled by the TestPattern feature.
	GX_ENUM_REGION_SEND_MODE                = 1019 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese ROI输出模式, 参考GX_REGION_SEND_MODE								\English ROI output mode, see also GX_REGION_SEND_MODE_ENTRY
	GX_ENUM_REGION_MODE                     = 1020 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 区域开关, 参考GX_REGION_MODE										\English zone switch, see also GX_REGION_MODE_ENTRY
	GX_ENUM_RREGION_SELECTOR                = 1021 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 区域选择，参考GX_RREGION_SELECTOR									\English Selects the Region of interest to control.
	GX_INT_CENTER_WIDTH                     = 1022 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 窗口宽度															\English width of window
	GX_INT_CENTER_HEIGHT                    = 1023 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 窗口高度															\English height of window
	GX_ENUM_BINNING_HORIZONTAL_MODE         = 1024 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 水平像素Binning模式,参考GX_BINNING_HORIZONTAL_MODE_ENTRY			\English Binning Horizontal mode, see also GX_BINNING_HORIZONTAL_MODE_ENTRY
	GX_ENUM_BINNING_VERTICAL_MODE           = 1025 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 垂直像素Binning模式,参考GX_BINNING_VERTICAL_MODE_ENTRY			\English Binning vertical mode, see also GX_BINNING_VERTICAL_MODE_ENTRY
	GX_ENUM_SENSOR_SHUTTER_MODE             = 1026 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Sensor曝光时间模式，参考GX_SENSOR_SHUTTER_MODE_ENTRY				\English Sensor shutter mode, refer to GX_SENSOR_SHUTTER_MODE_ENTRY
	GX_INT_DECIMATION_LINENUMBER            = 1027 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 抽样行数															\English decimation line number
	GX_INT_SENSOR_DECIMATION_HORIZONTAL     = 1028 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Sensor水平像素抽样												\English Sensor horizontal decimation
	GX_INT_SENSOR_DECIMATION_VERTICAL       = 1029 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Sensor垂直像素抽样												\English Sensor vertical decimation
	GX_ENUM_SENSOR_SELECTOR                 = 1030 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 选择当前传感器类型，参考GX_SENSOR_SELECTOR_ENTRY					\English selector current sonsor, refer to GX_SENSOR_SELECTOR_ENTRY
	GX_INT_CURRENT_SENSOR_WIDTH             = 1031 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器图像宽度												\English current sonsor width
	GX_INT_CURRENT_SENSOR_HEIGHT            = 1032 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器图像高度												\English current sonsor height
	GX_INT_CURRENT_SENSOR_OFFSETX           = 1033 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器水平偏移												\English current sonsor offset X
	GX_INT_CURRENT_SENSOR_OFFSETY           = 1034 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器垂直偏移												\English current sonsor offset Y
	GX_INT_CURRENT_SENSOR_WIDTHMAX          = 1035 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器宽度最大值												\English current sonsor width max
	GX_INT_CURRENT_SENSOR_HEIGHTMAX         = 1036 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 当前传感器高度最大值												\English current sonsor height max
	GX_ENUM_SENSOR_BIT_DEPTH                = 1037 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Sensor位深,参考GX_SENSOR_BIT_DEPTH_ENTRY							\English Sensor Bit Depth, refer to GX_SENSOR_BIT_DEPTH_ENTRY
	GX_BOOL_WATERMARK_ENABLE                = 1038 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 水印																\English Watermark
	GX_ENUM_BINNING_SELECTOR                = 1039 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese Binning选择，参考GX_BINNING_SELECTOR_ENTRY                        \English Binning select, refer to GX_BINNING_SELECTOR_ENTRY

	//---------------TransportLayer Section-------------------------------
	GX_INT_PAYLOAD_SIZE                              = 2000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 数据大小															\English Provides the number of bytes transferred for each image or chunk on the stream channel.
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_LLA          = 2001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< \Chinese LLA方式配置IP													\English Controls whether the Link Local Address IP configuration scheme is activated on the given logical link.
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_DHCP         = 2002 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< \Chinese DHCP方式配置IP													\English Controls whether the DHCP IP configuration scheme is activated on the given logical link.
	GX_BOOL_GEV_CURRENT_IPCONFIGURATION_PERSISTENTIP = 2003 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< \Chinese 永久IP方式配置IP													\English Controls whether the PersistentIP configuration scheme is activated on the given logical link.
	GX_INT_ESTIMATED_BANDWIDTH                       = 2004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 预估带宽，单位Bps(Bytes per second)								\English EstimatedBandwidth, Unit: Bps(Bytes per second)
	GX_INT_GEV_HEARTBEAT_TIMEOUT                     = 2005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 心跳超时时间														\English Controls the current heartbeat timeout in milliseconds.
	GX_INT_GEV_PACKETSIZE                            = 2006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 流通道包长														\English Specifies the stream packet size, in bytes, to send on the selected channel for a GVSP transmitter or specifies the maximum packet size supported by a GVSP receiver.
	GX_INT_GEV_PACKETDELAY                           = 2007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 流通道包间隔														\English Controls the delay(in timestamp counter unit) to insert between each packet for this stream channel.
	GX_INT_GEV_LINK_SPEED                            = 2008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< \Chinese 连接速度															\English It indicates the connection speed in Mbps for the selected network interface.
	GX_ENUM_DEVICE_TAP_GEOMETRY						 = 2009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< \Chinese 设备几何结构														\English Equipment geometry

	//---------------AcquisitionTrigger Section---------------------------
	GX_ENUM_ACQUISITION_MODE                 = 3000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 采集模式,参考GX_ACQUISITION_MODE_ENTRY							\English Sets the acquisition mode of the device.
	GX_COMMAND_ACQUISITION_START             = 3001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 开始采集															\English Starts the Acquisition of the device.
	GX_COMMAND_ACQUISITION_STOP              = 3002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 停止采集															\English Stops the Acquisition of the device at the end of the current Frame.
	GX_INT_ACQUISITION_SPEED_LEVEL           = 3003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 采集速度级别														\English Setting the speed level of acquiring image.
	GX_INT_ACQUISITION_FRAME_COUNT           = 3004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 多帧采集帧数														\English Number of frames to acquire in MultiFrame Acquisition mode.
	GX_ENUM_TRIGGER_MODE                     = 3005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 触发模式,参考GX_TRIGGER_MODE_ENTRY								\English Controls if the selected trigger is active.
	GX_COMMAND_TRIGGER_SOFTWARE              = 3006 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 软触发															\English Generates an internal trigger.
	GX_ENUM_TRIGGER_ACTIVATION               = 3007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 触发极性,参考GX_TRIGGER_ACTIVATION_ENTRY							\English Specifies the activation mode of the trigger.
	GX_ENUM_TRIGGER_SWITCH                   = 3008 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 外触发开关,参考GX_TRIGGER_SWITCH_ENTRY							\English Control external trigger signal is valid
	GX_FLOAT_EXPOSURE_TIME                   = 3009 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 曝光时间															\English Sets the Exposure time when ExposureMode is Timed and ExposureAuto is Off.
	GX_ENUM_EXPOSURE_AUTO                    = 3010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动曝光,参考GX_EXPOSURE_AUTO_ENTRY								\English Sets the automatic exposure mode when ExposureMode is Timed.
	GX_FLOAT_TRIGGER_FILTER_RAISING          = 3011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 上升沿触发滤波													\English Raising edge signal pulse width is smaller than this value is invalid.
	GX_FLOAT_TRIGGER_FILTER_FALLING          = 3012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 下降沿触发滤波													\English Falling edge signal pulse width is smaller than this value is invalid.
	GX_ENUM_TRIGGER_SOURCE                   = 3013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 触发源,参考GX_TRIGGER_SOURCE_ENTRY								\English Specifies the internal signal or physical input Line to use as the trigger source.
	GX_ENUM_EXPOSURE_MODE                    = 3014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 曝光模式，参考GX_EXPOSURE_MODE_ENTRY								\English Sets the operation mode of the Exposure(or shutter).
	GX_ENUM_TRIGGER_SELECTOR                 = 3015 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 触发类型选择，参考GX_TRIGGER_SELECTOR_ENTRY						\English Selects the type of trigger to configure.
	GX_FLOAT_TRIGGER_DELAY                   = 3016 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 触发延迟															\English Specifies the delay in microseconds(us) to apply after the trigger reception before activating it.
	GX_ENUM_TRANSFER_CONTROL_MODE            = 3017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 传输控制模式，参考GX_TRANSFER_CONTROL_MODE_ENTRY					\English Selects the control method for the transfers.
	GX_ENUM_TRANSFER_OPERATION_MODE          = 3018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 传输操作模式，参考GX_TRANSFER_OPERATION_MODE_ENTRY				\English Selects the operation mode of the transfer.
	GX_COMMAND_TRANSFER_START                = 3019 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 开始传输															\English Starts the streaming of data blocks out of the device.
	GX_INT_TRANSFER_BLOCK_COUNT              = 3020 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 传输帧数。														\English frame number of transmission. when set GX_ENUM_TRANSFER_OPERATION_MODE as GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK, this function is actived
	GX_BOOL_FRAMESTORE_COVER_ACTIVE          = 3021 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 帧存覆盖使能														\English FrameBufferOverwriteActive
	GX_ENUM_ACQUISITION_FRAME_RATE_MODE      = 3022 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 采集帧率调节模式，参考GX_ACQUISITION_FRAME_RATE_MODE_ENTRY		\English Controls if the acquisitionFrameRate is active
	GX_FLOAT_ACQUISITION_FRAME_RATE          = 3023 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 采集帧率															\English Controls the acquisition rate(in Hertz) at which the frames are captured.
	GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE  = 3024 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 当前采集帧率														\English Indicates the maximum allowed frame acquisition rate.
	GX_ENUM_FIXED_PATTERN_NOISE_CORRECT_MODE = 3025  | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 模板噪声校正，参考GX_FIXED_PATTERN_NOISE_CORRECT_MODE				\English Controls if the FixedPatternNoise is active
	GX_INT_ACQUISITION_BURST_FRAME_COUNT     = 3030 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 高速连拍帧数														\English frame number of transmission
	GX_ENUM_ACQUISITION_STATUS_SELECTOR      = 3031 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 采集状态选择,参考GX_ACQUISITION_STATUS_SELECTOR_ENTRY				\English Acquisition status selection
	GX_BOOL_ACQUISITION_STATUS               = 3032 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 采集状态															\English Acquisition status
	GX_FLOAT_EXPOSURE_DELAY                  = 30300| GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 曝光延迟															\English Delay of exposure
	GX_FLOAT_EXPOSURE_OVERLAP_TIME_MAX       = 30301 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 交叠曝光时间最大值												\English Maximum overlapping exposure time
	GX_ENUM_EXPOSURE_TIME_MODE               = 30302 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 曝光时间模式,参考GX_EXPOSURE_TIME_MODE_ENTRY						\English Exposure time mode
	GX_ENUM_ACQUISITION_BURST_MODE           = 30303 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 突发采集模式，参考GX_EXPOSURE_TIME_MODE_ENTRY						\English Burst acquisition mode
	GX_ENUM_OVERLAP_MODE                     = 30304 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 交叠模式,参考GX_OVERLAP_MODE_ENTRY								\English overlap mode,
	GX_ENUM_MULTISOURCE_SELECTOR             = 30305 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 多源触发选择,参考GX_MULTISOURCE_SELECTOR_ENTRY					\English MultiSourceSelector to GX_MULTISOURCE_SELECTOR_ENTRY
	GX_BOOL_MULTISOURCE_ENABLE               = 30306 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 多源触发使能														\English MultiSource Trigger Enable
	GX_BOOL_TRIGGER_CACHE_ENABLE             = 30307 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 缓存触发使能														\English Trigger Cache Enable

	//----------------DigitalIO Section----------------------------------
	GX_ENUM_USER_OUTPUT_SELECTOR    = 4000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 用户自定义输出选择,参考GX_USER_OUTPUT_SELECTOR_ENTRY				\English Selects which bit of the User Output register will be set by UserOutputValue.
	GX_BOOL_USER_OUTPUT_VALUE       = 4001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 用户自定义输出值													\English Sets the value of the bit selected by UserOutputSelector.
	GX_ENUM_USER_OUTPUT_MODE        = 4002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 用户IO输出模式,参考GX_USER_OUTPUT_MODE_ENTRY						\English Output signal can be used for different purposes, flash or a user-defined constant level
	GX_ENUM_STROBE_SWITCH           = 4003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 闪光灯开关,参考GX_STROBE_SWITCH_ENTRY								\English Set the flash light switch
	GX_ENUM_LINE_SELECTOR           = 4004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚选择,参考GX_LINE_SELECTOR_ENTRY								\English Selects the physical line(or pin) of the external device connector to configure.
	GX_ENUM_LINE_MODE               = 4005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚方向,参考GX_LINE_MODE_ENTRY									\English Controls if the physical Line is used to Input or Output a signal.
	GX_BOOL_LINE_INVERTER           = 4006 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚电平反转														\English Controls the inversion of the signal of the selected input or output Line.
	GX_ENUM_LINE_SOURCE             = 4007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚输出源,参考GX_LINE_SOURCE_ENTRY								\English Selects which internal acquisition or I/O source signal to output on the selected Line.
	GX_BOOL_LINE_STATUS             = 4008 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚状态															\English Returns the current status of the selected input or output Line.
	GX_INT_LINE_STATUS_ALL          = 4009 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 所有引脚的状态													\English Returns the current status of all available Line signals at time of polling in a single bit field.
	GX_FLOAT_PULSE_WIDTH            = 4010 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 用户自定义脉冲宽度												\English User-defined pulse width
	GX_INT_LINE_RANGE               = 4011 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 闪光灯脉宽														\English flash line ragne
	GX_INT_LINE_DELAY               = 4012 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 闪光灯延迟														\English flash line delay
	GX_INT_LINE_FILTER_RAISING_EDGE = 4013 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚上升沿滤波													\English Pin rising edge filtering
	GX_INT_LINE_FILTER_FALLING_EDGE = 4014 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 引脚下降沿滤波													\English Pin falling edge filtering

	//----------------AnalogControls Section----------------------------
	GX_ENUM_GAIN_AUTO                        = 5000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 自动增益,参考GX_GAIN_AUTO_ENTRY									\English Sets the automatic gain control(AGC) mode.
	GX_ENUM_GAIN_SELECTOR                    = 5001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 增益通道选择,参考GX_GAIN_SELECTOR_ENTRY							\English Selects which Gain is controlled by the various Gain features.
	GX_ENUM_BLACKLEVEL_AUTO                  = 5003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 自动黑电平,参考GX_BLACKLEVEL_AUTO_ENTRY							\English Controls the mode for automatic black level adjustment.
	GX_ENUM_BLACKLEVEL_SELECTOR              = 5004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 黑电平通道选择,参考GX_BLACKLEVEL_SELECTOR_ENTRY					\English Selects which Black Level is controlled by the various Black Level features.
	GX_ENUM_BALANCE_WHITE_AUTO               = 5006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 自动白平衡,参考GX_BALANCE_WHITE_AUTO_ENTRY						\English Controls the mode for automatic white balancing between the color channels.
	GX_ENUM_BALANCE_RATIO_SELECTOR           = 5007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 白平衡通道选择,参考GX_BALANCE_RATIO_SELECTOR_ENTRY				\English Selects which Balance ratio to control.
	GX_FLOAT_BALANCE_RATIO                   = 5008 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 白平衡系数														\English Controls ratio of the selected color component to a reference color component.
	GX_ENUM_COLOR_CORRECT                    = 5009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 颜色校正,参考GX_COLOR_CORRECT_ENTRY								\English Color correction
	GX_ENUM_DEAD_PIXEL_CORRECT               = 5010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 坏点校正,参考GX_DEAD_PIXEL_CORRECT_ENTRY							\English The dead pixel correct function can eliminate dead pixels in the image
	GX_FLOAT_GAIN                            = 5011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 增益																\English The value is an float value that sets the selected gain control in units specific to the camera.
	GX_FLOAT_BLACKLEVEL                      = 5012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 黑电平															\English Controls the analog black level as an absolute physical value.
	GX_BOOL_GAMMA_ENABLE                     = 5013 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese Gamma使能															\English Enable bit of Gamma
	GX_ENUM_GAMMA_MODE                       = 5014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese Gamma模式,参考GX_GAMMA_MODE_ENTRY									\English Gamma select
	GX_FLOAT_GAMMA                           = 5015 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese Gamma																\English Gamma
	GX_INT_DIGITAL_SHIFT                     = 5016 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese bit select														\English bit select
	GX_ENUM_LIGHT_SOURCE_PRESET              = 5017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 环境光源预设,参考GX_LIGHT_SOURCE_PRESET_ENTRY						\English Ambient Light Presets
	GX_BOOL_BLACKLEVEL_CALIB_STATUS          = 5018 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 黑电平标定状态													\English BlackLevelCalibStatus
	GX_INT_BLACKLEVEL_CALIB_VALUE            = 5019 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 黑电平标定值														\English BlackLevelCalibValue
	GX_FLOAT_PGA_GAIN                        = 5020 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese PGAGain增益														\English PGAGain
	GX_INT_FIXED_PATTERN_NOISE_CORRECT_RATIO = 5021 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 模板噪声校正系数                                                  \English Template noise correction coefficient

	//---------------CustomFeature Section-------------------------
	GX_INT_ADC_LEVEL                                     = 6000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese AD转换级别														\English When the pixel size is not 8bits, this function can be used to choose 8bits form 10bits or 12bit for show image.
	GX_INT_H_BLANKING                                    = 6001 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 水平消隐															\English Horizontal blanking
	GX_INT_V_BLANKING                                    = 6002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 垂直消隐															\English Vertical blanking
	GX_STRING_USER_PASSWORD                              = 6003 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 用户加密区密码													\English user password
	GX_STRING_VERIFY_PASSWORD                            = 6004 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 用户加密区校验密码												\English verify password
	GX_BUFFER_USER_DATA                                  = 6005 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 用户加密区内容													\English user data
	GX_INT_GRAY_VALUE                                    = 6006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 期望灰度值														\English ExpectedGrayValue_InqIsImplemented
	GX_ENUM_AA_LIGHT_ENVIRONMENT                         = 6007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 自动曝光、自动增益，光照环境类型,参考GX_AA_LIGHT_ENVIRMENT_ENTRY	\English Automatic function according to the external light conditions better for accommodation
	GX_INT_AAROI_OFFSETX                                 = 6008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动调节感兴趣区域X坐标											\English This value sets the X offset(left offset) for the rect of interest in pixels for 2A, i.e., the distance in pixels between the left side of the image area and the left side of the AAROI.
	GX_INT_AAROI_OFFSETY                                 = 6009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动调节感兴趣区域Y坐标											\English This value sets the Y offset(top offset) for the rect of interest for 2A, i.e., the distance in pixels between the top of the image area and the top of the AAROI.
	GX_INT_AAROI_WIDTH                                   = 6010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动调节感兴趣区域宽度											\English This value sets the width of the rect of interest in pixels for 2A.
	GX_INT_AAROI_HEIGHT                                  = 6011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动调节感兴趣区域高度											\English This value sets the height of the rect of interest in pixels for 2A.
	GX_FLOAT_AUTO_GAIN_MIN                               = 6012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 自动增益最小值													\English Setting up automatic gain range of minimum. When the gain is set to auto mode, this function works.
	GX_FLOAT_AUTO_GAIN_MAX                               = 6013 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 自动增益最大值													\English Setting up automatic gain range of maximum. When the gain is set to auto mode, this function works.
	GX_FLOAT_AUTO_EXPOSURE_TIME_MIN                      = 6014 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 自动曝光最小值													\English Setting up automatic shutter range of minimum. When the shutter is set to auto mode, this function works.
	GX_FLOAT_AUTO_EXPOSURE_TIME_MAX                      = 6015 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 自动曝光最大值													\English Setting up automatic shutter range of maximum. When the shutter is set to auto mode, this function works.
	GX_BUFFER_FRAME_INFORMATION                          = 6016 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 图像帧信息														\English FrameInformation
	GX_INT_CONTRAST_PARAM                                = 6017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 对比度参数														\English Contrast parameter
	GX_FLOAT_GAMMA_PARAM                                 = 6018 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 伽马参数															\English Gamma parameter
	GX_INT_COLOR_CORRECTION_PARAM                        = 6019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 颜色校正系数														\English Color correction coefficient
	GX_ENUM_IMAGE_GRAY_RAISE_SWITCH                      = 6020 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 图像亮度拉伸开关,参考GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY				\English Control ImageGrayRaise is valid
	GX_ENUM_AWB_LAMP_HOUSE                               = 6021 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 自动白平衡光源,参考GX_AWB_LAMP_HOUSE_ENTRY						\English Refers to the AWB working environment
	GX_INT_AWBROI_OFFSETX                                = 6022 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动白平衡感兴趣区域X坐标											\English This value sets the X offset(left offset) for the rect of interest in pixels for Auto WhiteBalance
	GX_INT_AWBROI_OFFSETY                                = 6023 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动白平衡感兴趣区域Y坐标											\English This value sets the Y offset(top offset) for the rect of interest for Auto WhiteBalance
	GX_INT_AWBROI_WIDTH                                  = 6024 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动白平衡感兴趣区域宽度											\English This value sets the width of the rect of interest in pixels for Auto WhiteBalance
	GX_INT_AWBROI_HEIGHT                                 = 6025 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 自动白平衡感兴趣区域高度											\English This value sets the height of the rect of interest in pixels for Auto WhiteBalance
	GX_ENUM_SHARPNESS_MODE                               = 6026 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 锐化模式,参考GX_SHARPNESS_MODE_ENTRY								\English Sharpening mode
	GX_FLOAT_SHARPNESS                                   = 6027 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 锐度																\English Sharpness
	GX_ENUM_USER_DATA_FILED_SELECTOR                     = 6028 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 用户选择Flash数据区域，参考GX_USER_DATA_FILED_SELECTOR_ENTRY		\English User selects Flash data area
	GX_BUFFER_USER_DATA_FILED_VALUE                      = 6029 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 用户区内容														\English User Area Content
	GX_ENUM_FLAT_FIELD_CORRECTION                        = 6030 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 平场校正开关,参考GX_FLAT_FIELD_CORRECTION_ENTRY					\English Flat field correction switch
	GX_ENUM_NOISE_REDUCTION_MODE                         = 6031 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 降噪开关,参考GX_NOISE_REDUCTION_MODE_ENTRY						\English Noise reduction switch
	GX_FLOAT_NOISE_REDUCTION                             = 6032 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 降噪																\English Noise reduction
	GX_BUFFER_FFCLOAD                                    = 6033 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 获取平场校正参数													\English Obtain flat field correction parameters
	GX_BUFFER_FFCSAVE                                    = 6034 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 保存平场校正参数													\English Set flat field correction parameters
	GX_ENUM_STATIC_DEFECT_CORRECTION                     = 6035 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 静态坏点校正，参考GX_ENUM_STATIC_DEFECT_CORRECTION_ENTRY			\English Static bad point correction
	GX_ENUM_2D_NOISE_REDUCTION_MODE                      = 6036 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 2D降噪模式,参考GX_2D_NOISE_REDUCTION_MODE_ENTRY					\English 2d noise reduction mode
	GX_ENUM_3D_NOISE_REDUCTION_MODE                      = 6037 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 3D降噪模式,参考GX_3D_NOISE_REDUCTION_MODE_ENTRY					\English 3d noise reduction mode
	GX_COMMAND_CLOSE_ISP                                 = 6038 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 关闭ISP															\English Close ISP
	GX_BUFFER_STATIC_DEFECT_CORRECTION_VALUE_ALL         = 6039 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 静态坏点校正值，参考GX_BUFFER_FFCSAVE					\English static defect conrrection value Refer to GX_BUFFER_FFCSAVE
	GX_BUFFER_STATIC_DEFECT_CORRECTION_FLASH_VALUE       = 6040 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 静态坏点校正flash值，参考GX_BUFFER_FFCSAVE			\English static defect conrrection flash value Refer to GX_BUFFER_FFCSAVE
	GX_INT_STATIC_DEFECT_CORRECTION_FINISH               = 6041 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,         ///< \Chinese 静态坏点校正结束，参考GX_INT_AWBROI_HEIGHT			\English static defect conrrection finish Refer to GX_INT_AWBROI_HEIGHT
	GX_BUFFER_STATIC_DEFECT_CORRECTION_INFO              = 6042 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 静态坏点校正信息，参考GX_BUFFER_FFCSAVE				\English static defect conrrection Info Refer to GX_BUFFER_FFCSAVE
	GX_COMMAND_STRIP_CALIBRATION_START                   = 6043 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 开始条纹标定											\English Starts the strip calibration
	GX_COMMAND_STRIP_CALIBRATION_STOP                    = 6044 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 结束条纹标定											\English Ready to stop the strip calibration
	GX_BUFFER_USER_DATA_FILED_VALUE_ALL                  = 6045 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 连续用户区内容										\English Continuous user area content
	GX_ENUM_SHADING_CORRECTION_MODE                      = 6046 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 明暗校正模式											\English Shade correction mode
	GX_COMMAND_FFC_GENERATE                              = 6047 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 生成平场校正系数										\English Generate flat field correction factor
	GX_ENUM_FFC_GENERATE_STATUS                          = 6048 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 平场校正状态											\English Level-field correction status
	GX_ENUM_FFC_EXPECTED_GRAY_VALUE_ENABLE               = 6049 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 平场校正期望灰度值使能								\English Level-field correction expected gray value enable
	GX_INT_FFC_EXPECTED_GRAY                             = 6050 | GX_FEATURE_INT     | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 平场校正期望灰度值									\English Flat-field correction expected gray value
	GX_INT_FFC_COEFFICIENTS_SIZE                         = 6051 | GX_FEATURE_INT     | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 平场校正系数大小										\English Level-field correction factor size
	GX_BUFFER_FFC_VALUE_ALL                              = 6052 | GX_FEATURE_BUFFER  | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 平场校正值											\English Level-field correction value
	GX_ENUM_DSNU_SELECTOR                                = 6053 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 暗场校正系数选择										\English Selection of dark field correction coefficient
	GX_COMMAND_DSNU_GENERATE                             = 6054 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 生成暗场校正系数										\English Generate dark field correction factor
	GX_ENUM_DSNU_GENERATE_STATUS                         = 6055 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 暗场校正状态											\English Dark field correction status
	GX_COMMAND_DSNU_SAVE                                 = 6056 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 保存暗场校正系数										\English Save dark-field correction factor
	GX_COMMAND_DSNU_LOAD                                 = 6057 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 加载暗场校正系数										\English Load dark-field correction factor
	GX_ENUM_PRNU_SELECTOR                                = 6058 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 明场校正系数选择										\English Selection of bright field correction coefficient
	GX_COMMAND_PRNU_GENERATE                             = 6059 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 生成明场校正系数										\English Generate bright field correction factor
	GX_ENUM_PRNU_GENERATE_STATUS                         = 6060 | GX_FEATURE_ENUM    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 明场校正状态											\English Bright-field correction status
	GX_COMMAND_PRNU_SAVE                                 = 6061 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 保存明场校正系数										\English Save the bright field correction factor
	GX_COMMAND_PRNU_LOAD                                 = 6062 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 加载明场校正系数										\English Loaded open field correction factor
	GX_FLOAT_SHARPNESS_NOISE_SUPPRESSION_THRESHOLD       = 6063 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 锐化噪声抑制阈值										\English Sharpening noise suppression threshold
	GX_INT_STATIC_DEFECT_CORRECTION_CALIB_STATUS         = 6064 | GX_FEATURE_INT		| GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 静态坏点标定状态										\English Static bad point calibration status
	GX_INT_FFC_FACTORY_STATUS                            = 6065 | GX_FEATURE_INT		| GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 平场校正状态检测										\English Level-field correction status detection
	GX_INT_DSNU_FACTORY_STATUS                           = 6066 | GX_FEATURE_INT		| GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 暗场校正状态检测										\English Detection of dark-field correction state
	GX_INT_PRNU_FACTORY_STATUS                           = 6067 | GX_FEATURE_INT		| GX_FEATURE_LEVEL_REMOTE_DEV,        ///< \Chinese 明场校正状态检测										\English Open field correction state detection
	GX_BUFFER_DETECT                                     = 6068 | GX_FEATURE_BUFFER	| GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese BUffer探测（CXP）										\English Buffer detection（CXP）
	GX_ENUM_FFC_COEFFICIENT                              = 6069 | GX_FEATURE_ENUM     | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 平场校正系数选择										\English Selection of flat field correction coefficient
	GX_BUFFER_FFCFLASH_LOAD                              = 6070 | GX_FEATURE_BUFFER   | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 加载平场校正系数										\English Load the flat field correction coefficient
	GX_BUFFER_FFCFALSH_SAVE                              = 6071 | GX_FEATURE_BUFFER   | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 保存平场校正系数										\English Save the flat field correction coefficient
	GX_INT_FFCROI_WIDTH                                  = 6072 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 平场校正感兴趣区域宽度								\English Flat field correction ROI width
	GX_INT_FFCROI_HEIGHT                                 = 6073 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 平场校正感兴趣区域高度								\English Flat field correction of the height of the region of interest
	GX_INT_FFCROI_OFFSETX                                = 6074 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 平场校正感兴趣区域水平偏移							\English Horizontal offset of region of interest for flat field correction
	GX_INT_FFCROI_OFFSETY                                = 6075 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 平场校正感兴趣区域垂直偏移							\English Flat field correction for vertical offset of region of interest
	GX_INT_STATIC_DEFECT_CORRECTION_VALUE_ALL_AONCE_MODE = 6076 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 万兆网相机标志										\English 10 Gigabit Ethernet camera logo
	GX_COMMAND_FFCFACTORY_CALIB                          = 6077 | GX_FEATURE_COMMAND   | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 平场校正出厂标定										\English Factory calibration for flat field calibration
	GX_INT_FFCFACTORY_CALIB_STATUS                       = 6078 | GX_FEATURE_INT   | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 平场校正出厂标定状态									\English Factory calibration status for flat field calibration

	//---------------UserSetControl Section-------------------------
	GX_ENUM_USER_SET_SELECTOR               = 7000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 参数组选择,参考GX_USER_SET_SELECTOR_ENTRY						\English Selects the feature User Set to load, save or configure.
	GX_COMMAND_USER_SET_LOAD                = 7001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 加载参数组													\English Loads the User Set specified by UserSetSelector to the device and makes it active.
	GX_COMMAND_USER_SET_SAVE                = 7002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 保存参数组													\English Save the User Set specified by UserSetSelector to the non-volatile memory of the devi
	GX_ENUM_USER_SET_DEFAULT                = 7003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 启动参数组,参考GX_USER_SET_DEFAULT_ENTRY						\English Selects the feature User Set to load and make active by default when the device is reset.
	GX_INT_DATA_FIELD_VALUE_ALL_USED_STATUS = 7004 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 用户数据区出厂状态											\English Factory status of user data area

	//---------------Event Section-------------------------
	GX_ENUM_EVENT_SELECTOR                             = 8000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 事件源选择,参考GX_EVENT_SELECTOR_ENTRY						\English Selects which Event to signal to the host application.
	GX_ENUM_EVENT_NOTIFICATION                         = 8001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 事件使能,参考GX_EVENT_NOTIFICATION_ENTRY						\English Activate or deactivate the notification to the host application of the occurrence of the selected Event.
	GX_INT_EVENT_EXPOSUREEND                           = 8002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 曝光结束事件ID												\English Returns the unique identifier of the ExposureEnd type of Event.
	GX_INT_EVENT_EXPOSUREEND_TIMESTAMP                 = 8003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 曝光结束事件时间戳											\English Returns the Timestamp of the ExposureEnd Event.
	GX_INT_EVENT_EXPOSUREEND_FRAMEID                   = 8004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 曝光结束事件帧ID												\English Returns the unique Identifier of the Frame(or image) that generated the ExposureEnd Event.
	GX_INT_EVENT_BLOCK_DISCARD                         = 8005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 数据块丢失事件ID												\English This enumeration value indicates the BlockDiscard event ID.
	GX_INT_EVENT_BLOCK_DISCARD_TIMESTAMP               = 8006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 数据块丢失事件时间戳											\English Indicates the time stamp for the BlockDiscard event
	GX_INT_EVENT_OVERRUN                               = 8007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 事件队列溢出事件ID											\English This enumeration value indicates the EventOverrun event ID.
	GX_INT_EVENT_OVERRUN_TIMESTAMP                     = 8008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 事件队列溢出事件时间戳										\English Indicates the time stamp of the EventOverrun event
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER                = 8009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 触发信号被屏蔽事件ID											\English This enumeration value indicates the FrameStartOverTrigger event ID.
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER_TIMESTAMP      = 8010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 触发信号被屏蔽事件时间戳										\English Indicates the time stamp of the FrameStartOverTrigger event
	GX_INT_EVENT_BLOCK_NOT_EMPTY                       = 8011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧存不为空事件ID												\English This enumeration value indicates the BlockNotEmpty event.
	GX_INT_EVENT_BLOCK_NOT_EMPTY_TIMESTAMP             = 8012 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧存不为空事件时间戳											\English Indicates the time stamp of the BlockNotEmpty event
	GX_INT_EVENT_INTERNAL_ERROR                        = 8013 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 内部错误事件ID												\English This enumeration value indicates the InternalError event.
	GX_INT_EVENT_INTERNAL_ERROR_TIMESTAMP              = 8014 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 内部错误事件时间戳											\English Indicates the time stamp of the InternalError event
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER           = 8015 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧触发屏蔽事件ID											\English Multi frame trigger mask event ID
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER_FRAMEID   = 8016 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧触发屏蔽事件帧ID											\English Multi frame trigger mask event frame ID
	GX_INT_EVENT_FRAMEBURSTSTART_OVERTRIGGER_TIMESTAMP = 8017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧触发屏蔽事件时间戳										\English Multi frame trigger mask event timestamp
	GX_INT_EVENT_FRAMESTART_WAIT                       = 8018 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧等待事件ID													\English Frame Wait Event ID
	GX_INT_EVENT_FRAMESTART_WAIT_TIMESTAMP             = 8019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧等待事件时间戳												\English Frame Wait Event Timestamp
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT                  = 8020 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧等待事件ID												\English Multi frame waiting event ID
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT_TIMESTAMP        = 8021 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧等待事件时间戳											\English Multi frame waiting event timestamp
	GX_INT_EVENT_BLOCK_DISCARD_FRAMEID                 = 8022 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 数据块丢失事件帧ID											\English Block Loss Event Frame ID
	GX_INT_EVENT_FRAMESTART_OVERTRIGGER_FRAMEID        = 8023 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 触发信号被屏蔽事件帧ID										\English Trigger signal masked event frame ID
	GX_INT_EVENT_BLOCK_NOT_EMPTY_FRAMEID               = 8024 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧存不为空事件帧ID											\English The frame memory is not empty Event frame ID
	GX_INT_EVENT_FRAMESTART_WAIT_FRAMEID               = 8025 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 帧等待事件帧ID												\English Frame Wait Event Frame ID
	GX_INT_EVENT_FRAMEBURSTSTART_WAIT_FRAMEID          = 8026 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 多帧等待事件帧ID 												\English Multi frame waiting event frame ID
	GX_ENUM_EVENT_SIMPLE_MODE                          = 8027 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 事件帧ID使能,参考GX_EVENT_SIMPLE_MODE_ENTRY					\English event block ID enable,refer to GX_EVENT_SIMPLE_MODE_ENTRY

	//---------------LUT Section-------------------------
	GX_ENUM_LUT_SELECTOR       = 9000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 查找表选择,参考GX_LUT_SELECTOR_ENTRY							\English Selects which LUT to control.
	GX_BUFFER_LUT_VALUEALL     = 9001 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 查找表内容													\English Accesses all the LUT coefficients in a single access without using individual LUTIndex.
	GX_BOOL_LUT_ENABLE         = 9002 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 查找表使能													\English Activates the selected LUT.
	GX_INT_LUT_INDEX           = 9003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 查找表索引													\English Control the index(offset) of the coefficient to access in the selected LUT.
	GX_INT_LUT_VALUE           = 9004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 查找表值														\English Returns the Value at entry LUTIndex of the LUT selected by LUTSelector.
	GX_INT_LUT_FACTORY_STATUS  = 9005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 查找表出厂状态												\English Lookup table factory status
	GX_INT_LUT_BLACK_THRESHOLD = 9006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 暗阈值														\English Dark threshold
	GX_INT_LUT_WHITE_THRESHOLD = 9007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 亮阈值														\English Bright threshold
	GX_BOOL_LUT_REVERSE        = 9008 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 灰度翻转开关													\English Gray flip switch

	//---------------ChunkData Section-------------------------
	GX_BOOL_CHUNKMODE_ACTIVE = 10001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 帧信息使能													\English Activates the inclusion of Chunk data in the payload of the image.
	GX_ENUM_CHUNK_SELECTOR   = 10002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 帧信息项选择，参考GX_CHUNK_SELECTOR_ENTRY						\English Selects which Chunk to enable or control.
	GX_BOOL_CHUNK_ENABLE     = 10003 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 单项帧信息使能												\English Enables the inclusion of the selected Chunk data in the payload of the image.

    //---------------Color Transformation Control-------------------------
	GX_ENUM_COLOR_TRANSFORMATION_MODE           = 11000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 颜色转换模式，参考GX_COLOR_TRANSFORMATION_MODE_ENTRY						\English Color conversion selection, see also GX_COLOR_TRANSFORMATION_MODE_ENTRY
	GX_BOOL_COLOR_TRANSFORMATION_ENABLE         = 11001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 颜色转换使能																\English Activates the selected Color Transformation module.
	GX_ENUM_COLOR_TRANSFORMATION_VALUE_SELECTOR = 11002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 颜色转换矩阵元素选择，参考GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_ENTRY	\English Selects the Gain factor or Offset of the Transformation matrix to access in the selected Color Transformation module.
	GX_FLOAT_COLOR_TRANSFORMATION_VALUE         = 11003 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 颜色转换矩阵元素															\English Represents the value of the selected Gain factor or Offset inside the Transformation matrix.
	GX_ENUM_SATURATION_MODE                     = 11004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 饱和度模式，参考GX_ENUM_SATURATION_MODE_ENTRY								\English Saturation Mode refer to GX_ENUM_SATURATION_MODE_ENTRY
	GX_INT_SATURATION                           = 11005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 饱和度																	\English Saturation

	//---------------CounterAndTimerControl Section-------------------------
	GX_ENUM_TIMER_SELECTOR           = 12000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计时器选择，参考GX_TIMER_SELECTOR_ENTRY						\English Selects which Counter to configure, Refer to GX_TIMER_SELECTOR_ENTRY
	GX_FLOAT_TIMER_DURATION          = 12001 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计时器持续时间												\English Sets the duration(in microseconds) of the Timer pulse.
	GX_FLOAT_TIMER_DELAY             = 12002 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计时器延迟时间												\English Sets the duration(in microseconds) of the delay to apply at the reception of a trigger before starting the Timer.
	GX_ENUM_TIMER_TRIGGER_SOURCE     = 12003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计时器触发源，参考GX_TIMER_TRIGGER_SOURCE_ENTRY				\English Selects the source of the trigger to start the Timer, Refer to GX_TIMER_TRIGGER_SOURCE_ENTRY
	GX_ENUM_COUNTER_SELECTOR         = 12004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器选择，参考GX_COUNTER_SELECTOR_ENTRY						\English Selects which Counter to configure, Refer to GX_COUNTER_SELECTOR_ENTRY
	GX_ENUM_COUNTER_EVENT_SOURCE     = 12005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器事件触发源，参考GX_COUNTER_EVENT_SOURCE_ENTRY			\English Select the events that will be the source to increment the Counter, Refer to GX_COUNTER_EVENT_SOURCE_ENTRY
	GX_ENUM_COUNTER_RESET_SOURCE     = 12006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器复位源，参考GX_COUNTER_RESET_SOURCE_ENTRY				\English Selects the signals that will be the source to reset the Counter, Refer to GX_COUNTER_RESET_SOURCE_ENTRY
	GX_ENUM_COUNTER_RESET_ACTIVATION = 12007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器复位信号极性，参考GX_COUNTER_RESET_ACTIVATION_ENTRY		\English Selects the Activation mode of the Counter Reset Source signal, Refer to GX_COUNTER_RESET_ACTIVATION_ENTRY
	GX_COMMAND_COUNTER_RESET         = 12008 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 计数器复位													\English Does a software reset of the selected Counter and starts it.
	GX_ENUM_COUNTER_TRIGGER_SOURCE   = 12009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器触发源，参考GX_COUNTER_TRIGGER_SOURCE_ENTRY				\English Counter trigger source refer to GX_COUNTER_TRIGGER_SOURCE_ENTRY
	GX_INT_COUNTER_DURATION          = 12010 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器持续时间												\English Counter Duration
	GX_ENUM_TIMER_TRIGGER_ACTIVATION = 12011 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 定时器触发极性,参考GX_TIMER_TRIGGER_ACTIVATION_ENTRY			\English Timer Trigger Activation see also GX_TIMER_TRIGGER_ACTIVATION_ENTRY
	GX_INT_COUNTER_VALUE             = 12012 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 计数器值														\English counter value

	//---------------RemoveParameterLimitControl Section-------------------------
	GX_ENUM_REMOVE_PARAMETER_LIMIT     = 13000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 取消参数范围限制, 参考GX_REMOVE_PARAMETER_LIMIT_ENTRY			\English Cancel parameter range restriction, refer to GX_REMOVE_PARAMETER_LIMIT_ENTRY

    //---------------HDRControl Section-------------------------
    GX_ENUM_HDR_MODE                   = 14000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese HDR模式,参考GX_HDR_MODE_ENTRY									\English HDR mode,Refer to GX_HDR_MODE_ENTRY
    GX_INT_HDR_TARGET_LONG_VALUE       = 14001 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 亮场期望值													\English Bright field expectations
    GX_INT_HDR_TARGET_SHORT_VALUE      = 14002 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 暗场期望值													\English dark field expectations
    GX_INT_HDR_TARGET_MAIN_VALUE       = 14003 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 融合期望值													\English Convergence expectations

    //---------------MultiGrayControl Section-------------------------
    GX_ENUM_MGC_MODE                   = 15001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 多帧灰度控制模式,参考GX_MGC_MODE_ENTRY						\English Multi-frame grey scale control mode ,Refer to GX_MGC_MODE_ENTRY
    GX_INT_MGC_SELECTOR                = 15002 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 多帧灰度选择													\English Multiframe grey color selection
    GX_FLOAT_MGC_EXPOSURE_TIME         = 15003 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 多帧灰度曝光时间												\English Multi-frame grey time exposure time
    GX_FLOAT_MGC_GAIN                  = 15004 | GX_FEATURE_FLOAT| GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 多帧灰度增益													\English Multiframe grey gain

    //---------------ImageQualityControl Section-------------------------
	GX_BUFFER_STRIPED_CALIBRATION_INFO = 16001 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 条纹标定信息，参考GX_BUFFER_STATIC_DEFECT_CORRECTION_INFO	\English Fringe calibration information Refer to GX_BUFFER_STATIC_DEFECT_CORRECTION_INFO
	GX_FLOAT_CONTRAST                  = 16002 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 对比度，参考GX_FLOAT_MGC_GAIN								\English Contrast
	GX_ENUM_HOTPIXEL_CORRECTION        = 16003 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 热像素校正													\English Hotpixel correction

    //---------------GyroControl Section-------------------------
    GX_BUFFER_IMU_DATA                                     = 17001 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,	///< \Chinese 陀螺仪数据																			\English IMU data
    GX_ENUM_IMU_CONFIG_ACC_RANGE                           = 17002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 加速计测量范围，参考GX_IMU_CONFIG_ACC_RANGE_ENTRY										\English IMU config acc range, refer to GX_IMU_CONFIG_ACC_RANGE_ENTRY
    GX_ENUM_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_SWITCH      = 17003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 加速计低通滤波开关， 参考GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_SWITCH_ENTRY			\English IMU config acc odr low pass filter switch, refer to GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_SWITCH_ENTRY
    GX_ENUM_IMU_CONFIG_ACC_ODR                             = 17004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 加速计输出数据率，参考GX_IMU_CONFIG_ACC_ODR_ENTRY										\English IMU config acc odr, refer to GX_IMU_CONFIG_ACC_ODR_ENTRY
    GX_ENUM_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY   = 17005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 加速计加速计低通截止频率，参考GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY	\English imu config acc odr low pass filter frequency, refer to GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_XRANGE                         = 17006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 陀螺仪X方向测量范围，参考GX_IMU_CONFIG_GYRO_RANGE_ENTRY								\English imu config gyro Xrange, refer to GX_IMU_CONFIG_GYRO_RANGE_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_YRANGE                         = 17007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 陀螺仪Y方向测量范围，参考GX_IMU_CONFIG_GYRO_RANGE_ENTRY								\English imu config gyro Yrange, refer to GX_IMU_CONFIG_GYRO_RANGE_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_ZRANGE                         = 17008 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 陀螺仪Z方向测量范围，参考GX_IMU_CONFIG_GYRO_RANGE_ENTRY								\English imu config gyro Zrange, refer to GX_IMU_CONFIG_GYRO_RANGE_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_SWITCH     = 17009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 陀螺仪低通滤波开关，参考GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_SWITCH_ENTRY			\English imu config gyro odr low pass filter switch, refer to GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_SWITCH_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_ODR                            = 17010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 陀螺仪输出数据率，参考GX_IMU_CONFIG_GYRO_ODR_ENTRY									\English imu config gyro odr, refer to GX_IMU_CONFIG_GYRO_ODR_ENTRY
    GX_ENUM_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY  = 17011 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 加速计低通截止频率，参考GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY		\English imu config gyro odr low pass filter frequency, refer to GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY
    GX_FLOAT_IMU_ROOM_TEMPERATURE                          = 17012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,	///< \Chinese 陀螺仪温度室温设置																	\English imu room temperature
    GX_ENUM_IMU_TEMPERATURE_ODR                            = 17013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,		///< \Chinese 温度计输出数据率设置，参考GX_IMU_TEMPERATURE_ODR_ENTRY								\English imu temperature odr, refer to GX_IMU_TEMPERATURE_ODR_ENTRY

	//---------------FrameBufferControl Section-------------------------
	GX_INT_FRAME_BUFFER_COUNT                              = 18001 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 帧存深度																				\English Frame memory depth
	GX_COMMAND_FRAME_BUFFER_FLUSH                          = 18002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 清空帧存																				\English Empty the frame save


	//----------------SerialPortControl Section----------------------------------
	GX_ENUM_SERIALPORT_SELECTOR                   = 19001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 串口选择																				\English Serial port selection
	GX_ENUM_SERIALPORT_SOURCE                     = 19002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 串口输入源																			\English Serial port input source
	GX_ENUM_SERIALPORT_BAUDRATE                   = 19003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 串口波特率																			\English Serial baud rate
	GX_INT_SERIALPORT_DATA_BITS                   = 19004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 串口数据位																			\English Serial port data bit
	GX_ENUM_SERIALPORT_STOP_BITS                  = 19005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 串口停止位																			\English Serial port stop bit
	GX_ENUM_SERIALPORT_PARITY                     = 19006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 串口奇偶校验																			\English Serial port parity
	GX_INT_TRANSMIT_QUEUE_MAX_CHARACTER_COUNT     = 19007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 传输队列最大值字符数																	\English Maximum number of characters in transmission queue
	GX_INT_TRANSMIT_QUEUE_CURRENT_CHARACTER_COUNT = 19008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 传输队列当前字符数																	\English Current number of characters in the transmission queue
	GX_INT_RECEIVE_QUEUE_MAX_CHARACTER_COUNT      = 19009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 接收队列最大值字符数																	\English Maximum number of characters in receive queue
	GX_INT_RECEIVE_QUEUE_CURRENT_CHARACTER_COUNT  = 19010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 接收队列当前字符数																	\English Current number of characters in the receive queue
	GX_INT_RECEIVE_FRAMING_ERROR_COUNT            = 19011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 接收帧错误计数																		\English Received frame error count
	GX_INT_RECEIVE_PARITY_ERROR_COUNT             = 19012 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 接收奇偶校验错误计数																	\English Receive parity error count
	GX_COMMAND_RECEIVE_QUEUE_CLEAR                = 19013 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 队列清除																				\English Queue Clear
	GX_BUFFER_SERIALPORT_DATA                     = 19014 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 串口数据																				\English serial data
	GX_INT_SERIALPORT_DATA_LENGTH                 = 19015 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 串口数据长度 																			\English Serial port data length
	GX_INT_SERIAL_PORT_DETECTION_STATUS           = 19016 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 串口状态检测																			\English Serial port status detection

	//---------------CoaXPress Section-------------------------
	GX_ENUM_CXP_LINK_CONFIGURATION                = 20001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 连接配置																				\English Connection configuration
	GX_ENUM_CXP_LINK_CONFIGURATION_PREFERRED      = 20002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 预设连接配置																			\English Preset connection configuration
	GX_ENUM_CXP_LINK_CONFIGURATION_STATUS         = 20003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese CXP连接配置状态																		\English CXP connection configuration status
	GX_INT_IMAGE1_STREAM_ID                       = 20004 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 第一张图流ID																			\English First image flow ID
	GX_ENUM_CXP_CONNECTION_SELECTOR               = 20005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 连接选择																				\English Connection selection
	GX_ENUM_CXP_CONNECTION_TEST_MODE              = 20006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 连接测试模式																			\English Connection test mode
	GX_INT_CXP_CONNECTION_TEST_ERROR_COUNT        = 20007 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 连接测试错误计数																		\English Connection test error count
	GX_INT_CXP_CONNECTION_TEST_PACKET_RX_COUNT    = 20008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 连接测试数据包接收个数																\English Number of connection test packets received
	GX_INT_CXP_CONNECTION_TEST_PACKET_TX_COUNT    = 20009 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 连接测试数据包发送个数																\English Number of connection test packets sent

	//--------------SequencerControl Section-------------------------
	GX_ENUM_SEQUENCER_MODE                        = 21001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器模式																			\English Sequencer mode
	GX_ENUM_SEQUENCER_CONFIGURATION_MODE          = 21002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器配置模式																		\English Sequencer configuration mode
	GX_ENUM_SEQUENCER_FEATURE_SELECTOR            = 21003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器功能选择器																		\English Sequencer function selector
	GX_BOOL_SEQUENCER_FEATURE_ENABLE              = 21004 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器功能启用																		\English Sequencer function enabled
	GX_INT_SEQUENCER_SET_SELECTOR                 = 21005 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器设置选择器																		\English Sequencer setting selector
	GX_INT_SEQUENCER_SET_COUNT                    = 21006 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器计数																			\English Sequencer count
	GX_INT_SEQUENCER_SET_ACTIVE                   = 21007 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列器设置激活																		\English Sequencer settings active
	GX_COMMAND_SEQUENCER_SET_RESET                = 21008 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 序列器设置重置																		\English Sequencer setting reset
	GX_INT_SEQUENCER_PATH_SELECTOR                = 21009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 序列器路径选择																		\English Sequencer payh selection
	GX_INT_SEQUENCER_SET_NEXT                     = 21010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,       ///< \Chinese 序列器设置下一个																		\English Sequencer Next
	GX_ENUM_SEQUENCER_TRIGGER_SOURCE              = 21011 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 序列触发源																			\English Sequencer Trigger
	GX_COMMAND_SEQUENCER_SET_SAVE                 = 21012 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 保存序列																				\English Sequencer Save
	GX_COMMAND_SEQUENCER_SET_LOAD                 = 21013 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 加载序列																				\English Sequencer Load

	//--------------EnoderControl Section-------------------------
	GX_ENUM_ENCODER_SELECTOR                      = 22001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 编码器选择器																			\English Encoder selector
	GX_ENUM_ENCODER_DIRECTION                     = 22002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 编码器方向																			\English Encoder direction
	GX_INT_ENCODER_VALUE                          = 22003 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 解码器值																				\English Decoder value
	GX_ENUM_ENCODER_SOURCEA                       = 22004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 编码器A相输入																			\English Encoder phase A input
	GX_ENUM_ENCODER_SOURCEB                       = 22005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 编码器B相输入																			\English Encoder phase B input
	GX_ENUM_ENCODER_MODE                          = 22006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 编码器模式																			\English Encoder Mode


	//////////////////////////////////////////////////////////////////////////
	///Chinese	本地设备层
	///English	Device Feature
	//////////////////////////////////////////////////////////////////////////
	GX_DEV_INT_COMMAND_TIMEOUT                = 0 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DEV,   ///< \Chinese 命令超时																				\English Indicates the current command timeout of the specific Link.
	GX_DEV_INT_COMMAND_RETRY_COUNT            = 1 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DEV,   ///< \Chinese 命令重试次数																			\English Command retry times

	//////////////////////////////////////////////////////////////////////////
	///Chinese	流层
	///English	DataStream Feature
	//////////////////////////////////////////////////////////////////////////
	GX_DS_INT_ANNOUNCED_BUFFER_COUNT          = 0 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 声明的Buffer个数																		\English Number of Buffers declared
	GX_DS_INT_DELIVERED_FRAME_COUNT           = 1 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 接收帧个数(包括残帧)																	\English Number of received frames(including residual frames)
	GX_DS_INT_LOST_FRAME_COUNT                = 2 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese buffer不足导致的丢帧个数																\English Number of frames lost due to insufficient buffers
	GX_DS_INT_INCOMPLETE_FRAME_COUNT          = 3 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 接收的残帧个数																		\English Number of residual frames received
	GX_DS_INT_DELIVERED_PACKET_COUNT          = 4 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 接收到的包数																			\English Number of packets received
	GX_DS_INT_RESEND_PACKET_COUNT             = 5 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 重传包个数																			\English Number of retransmission packets
	GX_DS_INT_RESCUED_PACKED_COUNT            = 6 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 重传成功包个数																		\English Number of successful retransmitted packets
	GX_DS_INT_RESEND_COMMAND_COUNT            = 7 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 重传命令次数																			\English Repeat command times
	GX_DS_INT_UNEXPECTED_PACKED_COUNT         = 8 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 异常包个数																			\English Exception packet number
	GX_DS_INT_MAX_PACKET_COUNT_IN_ONE_BLOCK   = 9 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,     ///< \Chinese 数据块最大重传包数																	\English Maximum number of retransmissions of data blocks
	GX_DS_INT_MAX_PACKET_COUNT_IN_ONE_COMMAND = 10 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 一次重传命令最大包含的包数															\English Maximum number of packets contained in a retransmit command
	GX_DS_INT_RESEND_TIMEOUT                  = 11 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 重传超时时间																			\English Retransmission timeout time
	GX_DS_INT_MAX_WAIT_PACKET_COUNT           = 12 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 最大等待包数																			\English Maximum waiting packet number
	GX_DS_ENUM_RESEND_MODE                    = 13 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_DS,   ///< \Chinese 重传模式,参考GX_DS_RESEND_MODE_ENTRY													\English Retransmission, see also GX_DS_RESEND_MODE_ENTRY
	GX_DS_INT_MISSING_BLOCKID_COUNT           = 14 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese BlockID丢失个数																		\English Missing number of BlockID
	GX_DS_INT_BLOCK_TIMEOUT                   = 15 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 数据块超时时间																		\English Data block timeout
	GX_DS_INT_STREAM_TRANSFER_SIZE            = 16 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 传输数据块大小																		\English size of transfer block
	GX_DS_INT_STREAM_TRANSFER_NUMBER_URB      = 17 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 传输数据块数量																		\English Number of data blocks transmitted
	GX_DS_INT_PACKET_TIMEOUT                  = 19 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 包超时时间																			\English time of package timeout
	GX_DS_INT_SOCKET_BUFFER_SIZE              = 20 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 套接字缓冲区大小																		\English Socket buffer size in kilobytes
	GX_DS_ENUM_STOP_ACQUISITION_MODE          = 21 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_DS,   ///< \Chinese 停采模式，参考GX_STOP_ACQUISITION_MODE_ENTRY											\English stop acquisition mode
	GX_DS_ENUM_STREAM_BUFFER_HANDLING_MODE    = 22 | GX_FEATURE_ENUM| GX_FEATURE_LEVEL_DS,    ///< \Chinese Buffer处理模式,参考GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY							\English Buffer processing mode, refer to GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY
	GX_DS_INT_ACQUISITION_BUFFER_CACHE_PREC   = 23 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,    ///< \Chinese 采集buffer缓冲数量																	\English Number of buffer caches collected
	GX_DS_ENUM_MULTI_RESEND_MODE              = 24 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_DS,   ///< \Chinese 重传模式,参考GX_DS_MULTI_RESEND_MODE_ENTRY										    \English Retransmission, see also GX_DS_MULTI_RESEND_MODE_ENTRY

	//////////////////////////////////////////////////////////////////////////
	///Chinese	废弃区间段
	///English	Deprecated Section
	//////////////////////////////////////////////////////////////////////////
	GX_STRING_DEVICE_ID                       = 4    | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备序列号[弃用，改用GX_STRING_DEVICE_SERIAL_NUMBER]											\English switch to GX_STRING_DEVICE_SERIAL_NUMBER
	GX_STRING_DEVICE_HARDWARE_VERSION         = 5    | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< \Chinese 设备硬件版本[弃用]																			\English Device hardware version
	GX_INT_GAIN                               = 5002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 增益[弃用，改用GX_FLOAT_GAIN]																	\English switch to GX_FLOAT_GAIN
	GX_INT_BLACKLEVEL                         = 5005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 黑电平[弃用，改用GX_FLOAT_BLACKLEVEL]															\English switch to GX_FLOAT_BLACKLEVEL
	GX_FLOAT_BALANCE_RATIO_SELECTOR           = 5007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 白平衡通道选择[弃用，改用GX_ENUM_BALANCE_RATIO_SELECTOR]										\English switch to GX_ENUM_BALANCE_RATIO_SELECTOR
	GX_ENUM_AA_LIGHT_ENVIRMENT                = 6007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< \Chinese 自动曝光、自动增益，光照环境类型[弃用，改用GX_ENUM_AA_LIGHT_ENVIRONMENT]						\English switch to GX_ENUM_AA_LIGHT_ENVIRONMENT
	GX_INT_ROI_X                              = 6008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动调节感兴趣区域X坐标[弃用，改用GX_INT_AAROI_OFFSETX]										\English switch to GX_INT_AAROI_OFFSETX
	GX_INT_ROI_Y                              = 6009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动调节感兴趣区域Y坐标[弃用，改用GX_INT_AAROI_OFFSETY]										\English switch to GX_INT_AAROI_OFFSETY
	GX_INT_ROI_WIDTH                          = 6010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动调节感兴趣区域宽度[弃用，改用GX_INT_AAROI_WIDTH]											\English switch to GX_INT_AAROI_WIDTH
	GX_INT_ROI_HEIGHT                         = 6011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动调节感兴趣区域高度[弃用，改用GX_INT_AAROI_HEIGHT]											\English switch to GX_INT_AAROI_HEIGHT
	GX_INT_AUTO_GAIN_VALUEMIN                 = 6012 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动增益最小值[弃用，改用GX_FLOAT_AUTO_GAIN_MIN]												\English switch to GX_FLOAT_AUTO_GAIN_MIN
	GX_INT_AUTO_GAIN_VALUEMAX                 = 6013 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动增益最大值[弃用，改用GX_FLOAT_AUTO_GAIN_MAX]												\English switch to GX_FLOAT_AUTO_GAIN_MAX
	GX_INT_AUTO_SHUTTER_VALUEMIN              = 6014 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动曝光最小值[弃用，改用GX_FLOAT_AUTO_EXPOSURE_TIME_MIN]										\English switch to GX_FLOAT_AUTO_EXPOSURE_TIME_MIN
	GX_INT_AUTO_SHUTTER_VALUEMAX              = 6015 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 自动曝光最大值[弃用，改用GX_FLOAT_AUTO_EXPOSURE_TIME_MAX]										\English switch to GX_FLOAT_AUTO_EXPOSURE_TIME_MAX
	GX_INT_CONTRASTPARAM                      = 6017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 对比度参数[弃用，改用GX_INT_CONTRAST_PARAM]													\English switch to GX_INT_CONTRAST_PARAM
	GX_FLOAT_GAMMAPARAM                       = 6018 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< \Chinese 伽马参数[弃用，改用GX_FLOAT_GAMMA_PARAM]														\English switch to GX_FLOAT_GAMMA_PARAM
	GX_INT_COLORCORRECTIONPARAM               = 6019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,      ///< \Chinese 颜色校正系数[弃用，改用GX_INT_COLOR_CORRECTION_PARAM]											\English switch to GX_INT_COLOR_CORRECTION_PARAM
	GX_DS_INT_MAX_NUM_QUEUE_BUFFER            = 18   | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,              ///< \Chinese 采集队列最大Buffer个数[弃用，可选择使用GXSetAcqusitionBufferNumber接口设置采集buffer个数]		\English the max number queue buffer
}GX_FEATURE_ID;
typedef int32_t GX_FEATURE_ID_CMD;


typedef enum GX_SENSOR_BIT_DEPTH_ENTRY
{
	GX_SENSOR_BIT_DEPTH_BPP8  = 8,    ///< \Chinese 位深8												\English BitDepth8
	GX_SENSOR_BIT_DEPTH_BPP10 = 10,   ///< \Chinese 位深10											\English BitDepth10
	GX_SENSOR_BIT_DEPTH_BPP12 = 12,   ///< \Chinese 位深12											\English BitDepth12
}GX_SENSOR_BIT_DEPTH_ENTRY;

typedef enum GX_ACQUISITION_MODE_ENTRY
{
	GX_ACQ_MODE_SINGLE_FRAME = 0,   ///< \Chinese 单帧模式											\English Single frame mode
	GX_ACQ_MODE_MULITI_FRAME = 1,   ///< \Chinese 多帧模式											\English Multi frame mode
	GX_ACQ_MODE_CONTINUOUS   = 2,   ///< \Chinese 连续模式											\English Continuous mode
}GX_ACQUISITION_MODE_ENTRY;

typedef enum GX_TRIGGER_MODE_ENTRY
{
	GX_TRIGGER_MODE_OFF = 0,   ///< \Chinese 关闭触发模式										\English Switch off the trigger mode
	GX_TRIGGER_MODE_ON  = 1,   ///< \Chinese 打开触发模式										\English Switch on the trigger mode
}GX_TRIGGER_MODE_ENTRY;

typedef enum GX_OVERLAP_MODE_ENTRY
{
	GX_OVERLAP_MODE_OFF = 0,   ///< \Chinese 关闭交叠模式										\English Switch off the overlap mode
	GX_OVERLAP_MODE_ON  = 1,   ///< \Chinese 打开交叠模式										\English Switch on the overlap mode
}GX_OVERLAP_MODE_ENTRY;

typedef enum GX_TRIGGER_SOURCE_ENTRY
{
	GX_TRIGGER_SOURCE_SOFTWARE    = 0,   ///< \Chinese 软触发											\English Software trigger
	GX_TRIGGER_SOURCE_LINE0       = 1,   ///< \Chinese 触发源0											\English Trigger source 0
	GX_TRIGGER_SOURCE_LINE1       = 2,   ///< \Chinese 触发源1											\English Trigger source 1
	GX_TRIGGER_SOURCE_LINE2       = 3,   ///< \Chinese 触发源2											\English Trigger source 2
	GX_TRIGGER_SOURCE_LINE3       = 4,   ///< \Chinese 触发源3											\English Trigger source 3
	GX_TRIGGER_SOURCE_COUNTER2END = 5,   ///< \Chinese COUNTER2END触发信号								\English Counter 2 end trigger
	GX_TRIGGER_SOURCE_TRIGGER     = 6,   ///< \Chinese 触发信号											\English Trigger source
	GX_TRIGGER_SOURCE_MULTISOURCE = 7,   ///< \Chinese 多源触发											\English MultiSource
	GX_TRIGGER_SOURCE_CXPTRIGGER0 = 8,   ///< \Chinese CXP触发源0										\English CXP Trigger source 0
	GX_TRIGGER_SOURCE_CXPTRIGGER1 = 9,   ///< \Chinese CXP触发源1										\English CXP Trigger source 1
}GX_TRIGGER_SOURCE_ENTRY;

typedef enum GX_MULTISOURCE_SELECTOR_ENTRY
{
	GX_MULTISOURCE_SELECTOR_SOFTWARE = 0,   ///< \Chinese 软触发											\English Software MultiSource
	GX_MULTISOURCE_SELECTOR_LINE0    = 1,   ///< \Chinese 多源触发选择0									\English MultiSource selector 0
	GX_MULTISOURCE_SELECTOR_LINE2    = 3,   ///< \Chinese 多源触发选择2									\English MultiSource selector 2
	GX_MULTISOURCE_SELECTOR_LINE3    = 4,   ///< \Chinese 多源触发选择3									\English MultiSource selector 3
}GX_MULTISOURCE_SELECTOR_ENTRY;

typedef enum GX_TRIGGER_ACTIVATION_ENTRY
{
	GX_TRIGGER_ACTIVATION_FALLINGEDGE = 0,   ///< \Chinese 下降沿触发										\English Falling edge trigger
	GX_TRIGGER_ACTIVATION_RISINGEDGE  = 1,   ///< \Chinese 上升沿触发										\English Rising edge trigger
	GX_TRIGGER_ACTIVATION_ANYEDGE     = 2,   ///< \Chinese 上升或下降沿触发									\English Falling or Rising edge trigger
	GX_TRIGGER_ACTIVATION_LEVELHIGH   = 3,   ///< \Chinese 高电平触发										\English High trigger
	GX_TRIGGER_ACTIVATION_LEVELLOW    = 4,   ///< \Chinese 低电平触发										\English Low trigger
}GX_TRIGGER_ACTIVATION_ENTRY;

typedef enum GX_TRIGGER_SWITCH_ENTRY
{
	GX_TRIGGER_SWITCH_OFF = 0,   ///< \Chinese 关闭外触发										\English Switch off the external trigger
	GX_TRIGGER_SWITCH_ON  = 1,   ///< \Chinese 打开外触发										\English Switch on the external trigger
}GX_TRIGGER_SWITCH_ENTRY;

typedef enum GX_EXPOSURE_MODE_ENTRY
{
	GX_EXPOSURE_MODE_TIMED        = 1,   ///< \Chinese 曝光时间寄存器控制曝光时间						\English Control exposure time through exposure time register
	GX_EXPOSURE_MODE_TRIGGERWIDTH = 2,   ///< \Chinese 触发信号宽度控制曝光时间							\English Control exposure time through trigger signal width
}GX_EXPOSURE_MODE_ENTRY;

typedef enum GX_EXPOSURE_AUTO_ENTRY
{
	GX_EXPOSURE_AUTO_OFF        = 0,   ///< \Chinese 关闭自动曝光										\English Switch off automatic exposure
	GX_EXPOSURE_AUTO_CONTINUOUS = 1,   ///< \Chinese 连续自动曝光										\English Continuous automatic exposure
	GX_EXPOSURE_AUTO_ONCE       = 2,   ///< \Chinese 单次自动曝光										\English Single automatic exposure
}GX_EXPOSURE_AUTO_ENTRY;

typedef enum GX_USER_OUTPUT_SELECTOR_ENTRY
{
	GX_USER_OUTPUT_SELECTOR_OUTPUT0 = 1,   ///< \Chinese 输出0											\English Output 0
	GX_USER_OUTPUT_SELECTOR_OUTPUT1 = 2,   ///< \Chinese 输出1											\English Output 1
	GX_USER_OUTPUT_SELECTOR_OUTPUT2 = 4,   ///< \Chinese 输出2											\English Output 2
	GX_USER_OUTPUT_SELECTOR_OUTPUT3 = 5,   ///< \Chinese 输出3											\English Output 3
	GX_USER_OUTPUT_SELECTOR_OUTPUT4 = 6,   ///< \Chinese 输出4											\English Output 4
	GX_USER_OUTPUT_SELECTOR_OUTPUT5 = 7,   ///< \Chinese 输出5											\English Output 5
	GX_USER_OUTPUT_SELECTOR_OUTPUT6 = 8,   ///< \Chinese 输出6											\English Output 6
}GX_USER_OUTPUT_SELECTOR_ENTRY;

typedef enum GX_USER_OUTPUT_MODE_ENTRY
{
	GX_USER_OUTPUT_MODE_STROBE      = 0,   ///< \Chinese 闪光灯											\English Strobe light
	GX_USER_OUTPUT_MODE_USERDEFINED = 1,   ///< \Chinese 用户自定义										\English User defined
}GX_USER_OUTPUT_MODE_ENTRY;

typedef enum GX_STROBE_SWITCH_ENTRY
{
	GX_STROBE_SWITCH_OFF = 0,   ///< \Chinese 关闭闪光灯开关									\English Switch off the strobe light
	GX_STROBE_SWITCH_ON  = 1,   ///< \Chinese 打开闪光灯开关									\English Switch on the strobe light
}GX_STROBE_SWITCH_ENTRY;

typedef enum GX_GAIN_AUTO_ENTRY
{
	GX_GAIN_AUTO_OFF        = 0,   ///< \Chinese 关闭自动增益										\English Switch off automatic gain
	GX_GAIN_AUTO_CONTINUOUS = 1,   ///< \Chinese 连续自动增益										\English Continuous automatic gain
	GX_GAIN_AUTO_ONCE       = 2,   ///< \Chinese 单次自动增益										\English Single automatic gain
}GX_GAIN_AUTO_ENTRY;

typedef enum GX_GAIN_SELECTOR_ENTRY
{
	GX_GAIN_SELECTOR_ALL         = 0,   ///< \Chinese 所有增益通道										\English All gain channels
	GX_GAIN_SELECTOR_RED         = 1,   ///< \Chinese 红通道增益										\English Red channel gain
	GX_GAIN_SELECTOR_GREEN       = 2,   ///< \Chinese 绿通道增益										\English Green channel gain
	GX_GAIN_SELECTOR_BLUE        = 3,   ///< \Chinese 蓝通道增益										\English Blue channel gain
	GX_GAIN_SELECTOR_DIGITAL_ALL = 4,   ///< \Chinese 数字增益											\English Digital gain
}GX_GAIN_SELECTOR_ENTRY;

typedef enum GX_BLACKLEVEL_AUTO_ENTRY
{
	GX_BLACKLEVEL_AUTO_OFF        = 0,   ///< \Chinese 关闭自动黑电平									\English Switch off automatic black level
	GX_BLACKLEVEL_AUTO_CONTINUOUS = 1,   ///< \Chinese 连续自动黑电平									\English Continuous automatic black level
	GX_BLACKLEVEL_AUTO_ONCE       = 2,   ///< \Chinese 单次自动黑电平									\English Single automatic black level
}GX_BLACKLEVEL_AUTO_ENTRY;

typedef enum GX_BLACKLEVEL_SELECTOR_ENTRY
{
	GX_BLACKLEVEL_SELECTOR_ALL   = 0,   ///< \Chinese 所有黑电平通道									\English All black level channels
	GX_BLACKLEVEL_SELECTOR_RED   = 1,   ///< \Chinese 红通道黑电平										\English Red channel black level
	GX_BLACKLEVEL_SELECTOR_GREEN = 2,   ///< \Chinese 绿通道黑电平										\English Green channel black level
	GX_BLACKLEVEL_SELECTOR_BLUE  = 3,   ///< \Chinese 蓝通道黑电平										\English Blue channel black level
	GX_BLACKLEVEL_SELECTOR_RB    = 7,
}GX_BLACKLEVEL_SELECTOR_ENTRY;

typedef enum GX_BALANCE_WHITE_AUTO_ENTRY
{
	GX_BALANCE_WHITE_AUTO_OFF        = 0,   ///< \Chinese 关闭自动白平衡									\English Switch off automatic white balance
	GX_BALANCE_WHITE_AUTO_CONTINUOUS = 1,   ///< \Chinese 连续自动白平衡									\English Continuous automatic white balance
	GX_BALANCE_WHITE_AUTO_ONCE       = 2,   ///< \Chinese 单次自动白平衡									\English Single automatic white balance
}GX_BALANCE_WHITE_AUTO_ENTRY;

typedef enum GX_BALANCE_RATIO_SELECTOR_ENTRY
{
	GX_BALANCE_RATIO_SELECTOR_RED   = 0,   ///< \Chinese 红通道											\English Red channel
	GX_BALANCE_RATIO_SELECTOR_GREEN = 1,   ///< \Chinese 绿通道											\English Red channel
	GX_BALANCE_RATIO_SELECTOR_BLUE  = 2,   ///< \Chinese 蓝通道											\English Blue channel
}GX_BALANCE_RATIO_SELECTOR_ENTRY;

typedef enum GX_COLOR_CORRECT_ENTRY
{
	GX_COLOR_CORRECT_OFF = 0,   ///< \Chinese 关闭自动颜色校正									\English Switch off automatic color correction
	GX_COLOR_CORRECT_ON  = 1,   ///< \Chinese 打开自动颜色校正									\English Switch on automatic color correction
}GX_COLOR_CORRECT_ENTRY;

typedef enum GX_DEAD_PIXEL_CORRECT_ENTRY
{
	GX_DEAD_PIXEL_CORRECT_OFF = 0,   ///< \Chinese 关闭自动坏点校正									\English Switch off automatic bad point correction
	GX_DEAD_PIXEL_CORRECT_ON  = 1,   ///< \Chinese 打开自动坏点校正									\English Switch on automatic bad point correction
}GX_DEAD_PIXEL_CORRECT_ENTRY;

typedef enum GX_AA_LIGHT_ENVIRMENT_ENTRY
{
	GX_AA_LIGHT_ENVIRMENT_NATURELIGHT = 0,   ///< \Chinese 自然光											\English Natural light
	GX_AA_LIGHT_ENVIRMENT_AC50HZ      = 1,   ///< \Chinese 50赫兹日光灯										\English 50 Hz fluorescent light
	GX_AA_LIGHT_ENVIRMENT_AC60HZ      = 2,   ///< \Chinese 60赫兹日光灯										\English 60 Hz fluorescent light
}GX_AA_LIGHT_ENVIRMENT_ENTRY;

typedef enum GX_DEVICE_TAP_GEOMETRY_ENTRY
{
	GX_DEVICE_TAP_GEOMETRY_GEOMETRY_1X_1Y  = 0,    ///< \Chinese Geometry_1X_1Y									\English Geometry_1X_1Y
	GX_DEVICE_TAP_GEOMETRY_GEOMETRY_1X_1Y2 = 9,    ///< \Chinese Geometry_1X_1Y2									\English Geometry_1X_1Y2
	GX_DEVICE_TAP_GEOMETRY_GEOMETRY_1X_2YE = 10,   ///< \Chinese Geometry_1X_2YE									\English Geometry_1X_2YE
}GX_DEVICE_TAP_GEOMETRY_ENTRY;

typedef enum GX_USER_SET_SELECTOR_ENTRY
{
	GX_ENUM_USER_SET_SELECTOR_DEFAULT  = 0,   ///< \Chinese 默认参数组										\English Default parameter group
	GX_ENUM_USER_SET_SELECTOR_USERSET0 = 1,   ///< \Chinese 用户参数组0										\English User parameter group 0
	GX_ENUM_USER_SET_SELECTOR_USERSET1 = 2,   ///< \Chinese 用户参数组1										\English User parameter group 1
}GX_USER_SET_SELECTOR_ENTRY;

typedef enum GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY
{
	GX_IMAGE_GRAY_RAISE_SWITCH_OFF = 0,   ///< \Chinese 图像亮度拉伸开关									\English Switch off the image brightness stretch
	GX_IMAGE_GRAY_RAISE_SWITCH_ON  = 1,   ///< \Chinese 图像亮度拉伸开关									\English Switch on the image brightness stretch
}GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY;

typedef enum GX_FIXED_PATTERN_NOISE_CORRECT_MODE
{
	GX_FIXEDPATTERNNOISECORRECT_OFF = 0,   ///< \Chinese 关闭模板噪声										\English Switch off the pattern noise correction
	GX_FIXEDPATTERNNOISECORRECT_ON  = 1,   ///< \Chinese 开启模板噪声										\English Switch on the pattern noise correction

}GX_FIXED_PATTERN_NOISE_CORRECT_MODE;

typedef enum GX_AWB_LAMP_HOUSE_ENTRY
{
	GX_AWB_LAMP_HOUSE_ADAPTIVE     = 0,   ///< \Chinese 自适应光源										\English Adaptive light source
	GX_AWB_LAMP_HOUSE_D65          = 1,   ///< \Chinese 指定色温6500k										\English Color temperature 6500k
	GX_AWB_LAMP_HOUSE_FLUORESCENCE = 2,   ///< \Chinese 指定荧光灯										\English Fluorescent
	GX_AWB_LAMP_HOUSE_INCANDESCENT = 3,   ///< \Chinese 指定白炽灯										\English Incandescent
	GX_AWB_LAMP_HOUSE_D75          = 4,   ///< \Chinese 指定色温7500k										\English Color temperature 7500k
	GX_AWB_LAMP_HOUSE_D50          = 5,   ///< \Chinese 指定色温5000k										\English Color temperature 5000k
	GX_AWB_LAMP_HOUSE_U30          = 6,   ///< \Chinese 指定色温3000k										\English Color temperature 3000k
}GX_AWB_LAMP_HOUSE_ENTRY;

typedef enum GX_TEST_PATTERN_ENTRY
{
 GX_ENUM_TEST_PATTERN_OFF                    = 0,   ///< \Chinese 关闭												\English Off
 GX_ENUM_TEST_PATTERN_GRAY_FRAME_RAMP_MOVING = 1,   ///< \Chinese 静止灰度递增										\English Static gray increment
 GX_ENUM_TEST_PATTERN_SLANT_LINE_MOVING      = 2,   ///< \Chinese 滚动斜条纹										\English Rolling slant line
 GX_ENUM_TEST_PATTERN_VERTICAL_LINE_MOVING   = 3,   ///< \Chinese 滚动竖条纹										\English Rolling vertical line
 GX_ENUM_TEST_PATTERN_HORIZONTAL_LINE_MOVING = 4,   ///< \Chinese 滚动横条纹										\English Rolling horizontal stripe
 GX_ENUM_TEST_PATTERN_SLANT_LINE             = 6,   ///< \Chinese 静止斜条纹										\English Static slant line
}GX_TEST_PATTERN_ENTRY;

typedef enum GX_TRIGGER_SELECTOR_ENTRY
{
 GX_ENUM_TRIGGER_SELECTOR_FRAME_START       = 1,   ///< \Chinese 采集一帧											\English Capture single frame
 GX_ENUM_TRIGGER_SELECTOR_FRAME_BURST_START = 2,   ///< \Chinese 采集多帧											\English High speed continuous shooting
} GX_TRIGGER_SELECTOR_ENTRY;

typedef enum GX_LINE_SELECTOR_ENTRY
{
	GX_ENUM_LINE_SELECTOR_LINE0       = 0,    ///< \Chinese 引脚0												\English Line 0
	GX_ENUM_LINE_SELECTOR_LINE1       = 1,    ///< \Chinese 引脚1												\English Line 1
	GX_ENUM_LINE_SELECTOR_LINE2       = 2,    ///< \Chinese 引脚2												\English Line 2
	GX_ENUM_LINE_SELECTOR_LINE3       = 3,    ///< \Chinese 引脚3												\English Line 3
	GX_ENUM_LINE_SELECTOR_LINE4       = 4,    ///< \Chinese 引脚4												\English Line 4
	GX_ENUM_LINE_SELECTOR_LINE5       = 5,    ///< \Chinese 引脚5												\English Line 5
	GX_ENUM_LINE_SELECTOR_LINE6       = 6,    ///< \Chinese 引脚6												\English Line 6
	GX_ENUM_LINE_SELECTOR_LINE7       = 7,    ///< \Chinese 引脚7												\English Line 7
	GX_ENUM_LINE_SELECTOR_LINE8       = 8,    ///< \Chinese 引脚8												\English Line 8
	GX_ENUM_LINE_SELECTOR_LINE9       = 9,    ///< \Chinese 引脚9												\English Line 9
	GX_ENUM_LINE_SELECTOR_LINE10      = 10,   ///< \Chinese 引脚10											\English Line 10
	GX_ENUM_LINE_SELECTOR_LINE_STROBE = 11,   ///< \Chinese 专用闪光灯引脚									\English Dedicated flash pin
	GX_ENUM_LINE_SELECTOR_LINE11      = 12,   ///< \Chinese 引脚11											\English Line 11
	GX_ENUM_LINE_SELECTOR_LINE12      = 13,   ///< \Chinese 引脚12											\English Line 12
	GX_ENUM_LINE_SELECTOR_LINE13      = 14,   ///< \Chinese 引脚13											\English Line 13
	GX_ENUM_LINE_SELECTOR_LINE14      = 15,   ///< \Chinese 引脚14											\English Line 14
	GX_ENUM_LINE_SELECTOR_TRIGGER     = 16,   ///< \Chinese 硬件触发输入										\English Trigger input
	GX_ENUM_LINE_SELECTOR_IO1         = 17,   ///< \Chinese GPIO输入											\English GPIO output
	GX_ENUM_LINE_SELECTOR_IO2         = 18,   ///< \Chinese GPIO输入											\English GPIO input
	GX_ENUM_LINE_SELECTOR_FLASH_P     = 19,   ///< \Chinese 闪光灯flash_B输出									\English flash_B output
	GX_ENUM_LINE_SELECTOR_FLASH_W     = 20,   ///< \Chinese 闪光灯flash_W输出									\English flash_W output
} GX_LINE_SELECTOR_ENTRY;

typedef enum GX_LINE_MODE_ENTRY
{
	GX_ENUM_LINE_MODE_INPUT  = 0,   ///< \Chinese 输入												\English Input
	GX_ENUM_LINE_MODE_OUTPUT = 1,   ///< \Chinese 输出												\English Output
} GX_LINE_MODE_ENTRY;

typedef enum GX_LINE_SOURCE_ENTRY
{
	GX_ENUM_LINE_SOURCE_OFF                      = 0,    ///< \Chinese 关闭										\English Off
	GX_ENUM_LINE_SOURCE_STROBE                   = 1,    ///< \Chinese 闪光灯									\English Strobe light
	GX_ENUM_LINE_SOURCE_USEROUTPUT0              = 2,    ///< \Chinese 用户自定义输出0							\English UserDefined output 0
	GX_ENUM_LINE_SOURCE_USEROUTPUT1              = 3,    ///< \Chinese 用户自定义输出1							\English UserDefined output 1
	GX_ENUM_LINE_SOURCE_USEROUTPUT2              = 4,    ///< \Chinese 用户自定义输出2							\English UserDefined output 2
	GX_ENUM_LINE_SOURCE_EXPOSURE_ACTIVE          = 5,    ///< \Chinese 曝光激活									\English Exposure active
	GX_ENUM_LINE_SOURCE_FRAME_TRIGGER_WAIT       = 6,    ///< \Chinese 帧触发等待								\English Frame trigger wait
	GX_ENUM_LINE_SOURCE_ACQUISITION_TRIGGER_WAIT = 7,    ///< \Chinese 采集触发等待								\English Acquisition trigger wait
	GX_ENUM_LINE_SOURCE_TIMER1_ACTIVE            = 8,    ///< \Chinese 计时器1激活								\English Timer 1 active
	GX_ENUM_LINE_SOURCE_USEROUTPUT3              = 9,    ///< \Chinese 用户自定义输出3							\English User defined output 3
	GX_ENUM_LINE_SOURCE_USEROUTPUT4              = 10,   ///< \Chinese 用户自定义输出4							\English User defined output 4
	GX_ENUM_LINE_SOURCE_USEROUTPUT5              = 11,   ///< \Chinese 用户自定义输出5							\English User defined output 5
	GX_ENUM_LINE_SOURCE_USEROUTPUT6              = 12,   ///< \Chinese 用户自定义输出6							\English User defined output 6
	GX_ENUM_LINE_SOURCE_TIMER2_ACTIVE            = 13,   ///< \Chinese 计时器2激活								\English Timer 2 active
	GX_ENUM_LINE_SOURCE_TIMER3_ACTIVE            = 14,   ///< \Chinese 计时器3激活								\English Timer 3 active
	GX_ENUM_LINE_SOURCE_FRAME_TRIGGER            = 15,   ///< \Chinese 帧触发									\English frame trigger
	GX_ENUM_LINE_SOURCE_Flash_W                  = 16,   ///< \Chinese Flash_w									\English Flash_w
	GX_ENUM_LINE_SOURCE_Flash_P                  = 17,   ///< \Chinese Flash_P									\English Flash_P
	GX_ENUM_LINE_SOURCE_SERIAL_PORT_0            = 18,   ///< \Chinese SerialPort0								\English SerialPort0
} GX_LINE_SOURCE_ENTRY;

typedef enum GX_ENCODER_SELECTOR_ENTRY
{
	GX_ENUM_ENCODER0 = 0,   ///< \Chinese 编码器选择器0										\English Encoder selector 0
	GX_ENUM_ENCODER1 = 1,   ///< \Chinese 编码器选择器1										\English Encoder selector 1
	GX_ENUM_ENCODER2 = 2,   ///< \Chinese 编码器选择器2										\English Encoder selector 2
} GX_ENCODER_SELECTOR_ENTRY;

typedef enum GX_ENCODER_SOURCEA_ENTRY
{
	GX_ENUM_SOURCEA_OFF   = 0,   ///< \Chinese 编码器A相关闭输入								\English Encoder A related closed input
	GX_ENUM_SOURCEA_LINE0 = 1,   ///< \Chinese 编码器A相输入Line0								\English Encoder phase A input Line0
	GX_ENUM_SOURCEA_LINE1 = 2,   ///< \Chinese 编码器A相输入Line1								\English Encoder phase A input Line1
	GX_ENUM_SOURCEA_LINE2 = 3,   ///< \Chinese 编码器A相输入Line2								\English Encoder phase A input Line2
	GX_ENUM_SOURCEA_LINE3 = 4,   ///< \Chinese 编码器A相输入Line3								\English Encoder phase A input Line3
	GX_ENUM_SOURCEA_LINE4 = 5,   ///< \Chinese 编码器A相输入Line4								\English Encoder phase A input Line4
	GX_ENUM_SOURCEA_LINE5 = 6,   ///< \Chinese 编码器A相输入Line5								\English Encoder phase A input Line5
} GX_ENCODER_SOURCEA_ENTRY;

typedef enum GX_ENCODER_SOURCEB_ENTRY
{
	GX_ENUM_SOURCEB_OFF   = 0,   ///< \Chinese 编码器B相关闭输入								\English Encoder phase B close input
	GX_ENUM_SOURCEB_LINE0 = 1,   ///< \Chinese 编码器B相输入Line0								\English Encoder phase B input Line0
	GX_ENUM_SOURCEB_LINE1 = 2,   ///< \Chinese 编码器B相输入Line1								\English Encoder phase B input Line1
	GX_ENUM_SOURCEB_LINE2 = 3,   ///< \Chinese 编码器B相输入Line2								\English Encoder phase B input Line2
	GX_ENUM_SOURCEB_LINE3 = 4,   ///< \Chinese 编码器B相输入Line3								\English Encoder phase B input Line3
	GX_ENUM_SOURCEB_LINE4 = 5,   ///< \Chinese 编码器B相输入Line4								\English Encoder phase B input Line4
	GX_ENUM_SOURCEB_LINE5 = 6,   ///< \Chinese 编码器B相输入Line5								\English Encoder phase B input Line5
} GX_ENCODER_SOURCEB_ENTRY;

typedef enum GX_ENCODER_MODE_ENTRY
{
	GX_ENUM_HIGH_RESOLUTION = 0,   ///< \Chinese 编码器模式										\English Encoder Mode
} GX_ENCODER_MODE_ENTRY;


typedef enum GX_ENCODER_DIRECTION_ENTRY
{
	GX_ENUM_FORWARD  = 0,   ///< \Chinese 编码器方向向前									\English Encoder direction forward
	GX_ENUM_BACKWARD = 1,   ///< \Chinese 编码器方向向后									\English Encoder direction backward
} GX_ENCODER_DIRECTION_ENTRY;

typedef enum GX_EVENT_SELECTOR_ENTRY
{
	GX_ENUM_EVENT_SELECTOR_EXPOSUREEND                 = 0x0004,   ///< \Chinese 曝光结束							\English End of exposure
	GX_ENUM_EVENT_SELECTOR_BLOCK_DISCARD               = 0x9000,   ///< \Chinese 图像帧丢弃						\English Image frame discarding
	GX_ENUM_EVENT_SELECTOR_EVENT_OVERRUN               = 0x9001,   ///< \Chinese 事件队列溢出						\English Event queue overflow
	GX_ENUM_EVENT_SELECTOR_FRAMESTART_OVERTRIGGER      = 0x9002,   ///< \Chinese 触发信号溢出						\English Trigger signal overflow
	GX_ENUM_EVENT_SELECTOR_BLOCK_NOT_EMPTY             = 0x9003,   ///< \Chinese 图像帧存不为空					\English Image frame memory is not empty
	GX_ENUM_EVENT_SELECTOR_INTERNAL_ERROR              = 0x9004,   ///< \Chinese 内部错误事件						\English Internal error events
	GX_ENUM_EVENT_SELECTOR_FRAMEBURSTSTART_OVERTRIGGER = 0x9005,   ///< \Chinese 多帧触发屏蔽事件					\English Multi frame trigger mask event
	GX_ENUM_EVENT_SELECTOR_FRAMESTART_WAIT             = 0x9006,   ///< \Chinese 帧等待事件						\English Frame Wait Event
	GX_ENUM_EVENT_SELECTOR_FRAMEBURSTSTART_WAIT        = 0x9007,   ///< \Chinese 多帧等待事件 						\English Multi frame wait event
} GX_EVENT_SELECTOR_ENTRY;

typedef enum GX_EVENT_NOTIFICATION_ENTRY
{
	GX_ENUM_EVENT_NOTIFICATION_OFF = 0,   ///< \Chinese 关闭事件 							\English Turn off event
	GX_ENUM_EVENT_NOTIFICATION_ON  = 1,   ///< \Chinese 开启事件 							\English Turn on event
} GX_EVENT_NOTIFICATION_ENTRY;

typedef enum GX_EVENT_SIMPLE_MODE_ENTRY
{
	GX_EVENT_SIMPLE_MODE_OFF = 0,   ///< \Chinese 关闭事件帧ID使能					 \English Turn off block ID enable
	GX_EVENT_SIMPLE_MODE_ON  = 1,   ///< \Chinese 打开事件帧ID使能					 \English Turn on block ID enable
}GX_EVENT_SIMPLE_MODE_ENTRY;

typedef enum GX_LUT_SELECTOR_ENTRY
{
	GX_ENUM_LUT_SELECTOR_LUMINANCE = 0,   ///< \Chinese 亮度								\English Luminance
} GX_LUT_SELECTOR_ENTRY;

typedef enum GX_TRANSFERDELAY_MODE_ENTRY
{
	GX_ENUM_TRANSFERDELAY_MODE_OFF = 0,   ///< \Chinese 禁用传输延迟						\English Disable transmission delay
	GX_ENUM_TRANSFERDELAY_MODE_ON  = 1,   ///< \Chinese 开启传输延迟						\English Enable transmission delay
} GX_TRANSFERDELAY_MODE_ENTRY;

typedef enum GX_COVER_FRAMESTORE_MODE_ENTRY
{
	GX_ENUM_COVER_FRAMESTORE_MODE_OFF = 0,   ///< \Chinese 禁用帧存覆盖						\English Disable Frame Memory Coverage
	GX_ENUM_COVER_FRAMESTORE_MODE_ON  = 1,   ///< \Chinese 开启帧存覆盖						\English Enable Frame Memory Coverage
} GX_COVER_FRAMESTORE_MODE_ENTRY;

typedef enum GX_USER_SET_DEFAULT_ENTRY
{
	GX_ENUM_USER_SET_DEFAULT_DEFAULT  = 0,   ///< \Chinese 出厂参数组						\English Default parameter group
	GX_ENUM_USER_SET_DEFAULT_USERSET0 = 1,   ///< \Chinese 用户参数组0						\English User parameter group 0
} GX_USER_SET_DEFAULT_ENTRY;

typedef enum GX_TRANSFER_CONTROL_MODE_ENTRY
{
	GX_ENUM_TRANSFER_CONTROL_MODE_BASIC         = 0,   ///< \Chinese 关闭传输控制模式					\English Turn off transmission control
	GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED = 1,   ///< \Chinese 用户控制传输控制模式				\English User-controlled transmission control mode
} GX_TRANSFER_CONTROL_MODE_ENTRY;

typedef enum GX_TRANSFER_OPERATION_MODE_ENTRY
{
	GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK = 0,   ///< \Chinese 指定发送帧数						\English Specify the number of frames to be sent
} GX_TRANSFER_OPERATION_MODE_ENTRY;

typedef enum GX_DS_RESEND_MODE_ENTRY
{
	GX_DS_RESEND_MODE_OFF = 0,   ///< \Chinese 关闭重传模式						\English Turn off resend mode
	GX_DS_RESEND_MODE_ON  = 1,   ///< \Chinese 启重传模式						\English Turn on resend mode
} GX_DS_RESEND_MODE_ENTRY;

typedef enum GX_DS_MULTI_RESEND_MODE_ENTRY
{
	GX_DS_MULTI_RESEND_MODE_OFF = 0,   ///< \Chinese 关闭二次重传模式					\English Turn off multiple resend mode
	GX_DS_MULTI_RESEND_MODE_ON  = 1,   ///< \Chinese 开启二次重传模式					\English Turn on multiple resend mode
} GX_DS_MULTI_RESEND_MODE_ENTRY;

typedef enum GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY
{
	GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST           = 1,   ///< \Chinese OldestFirst 模式					\English OldestFirst Mode
	GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST_OVERWRITE = 2,   ///< \Chinese OldestFirstOverwrite模式			\English OldestFirstOverwrite Mode
	GX_DS_STREAM_BUFFER_HANDLING_MODE_NEWEST_ONLY            = 3,   ///< \Chinese NewestOnly模式					\English NewestOnly Mode

}GX_DS_STREAM_BUFFER_HANDLING_MODE_ENTRY;

typedef enum GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY
{
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_OFF = 0,   ///< \Chinese 关闭设备带宽限制模式				\English Turn off bandwidth limitation mode
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ON  = 1    ///< \Chinese 开启设备带宽限制模式				\English Turn on bandwidth limitation mode
}GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY;

typedef enum GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY
{
	GX_TEST_PATTERN_GENERATOR_SELECTOR_SENSOR  = 0,   ///< \Chinese sensor 的测试图					\English Test pattern generator of sensor
	GX_TEST_PATTERN_GENERATOR_SELECTOR_REGION0 = 1,   ///< \Chinese FPGA的测试图						\English Test pattern generator of FPGA
}GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY;


typedef enum GX_CHUNK_SELECTOR_ENTRY
{
	GX_CHUNK_SELECTOR_CHUNK_FRAME_ID      = 1,   ///< \Chinese 帧号								\English Frame ID
	GX_CHUNK_SELECTOR_CHUNK_TIME_STAMP    = 2,   ///< \Chinese 时间戳							\English Timestamp
	GX_CHUNK_SELECTOR_CHUNK_COUNTER_VALUE = 3    ///< \Chinese 计数器值							\English Counter value
}GX_CHUNK_SELECTOR_ENTRY;

typedef enum GX_ACQUISITION_FRAME_RATE_MODE_ENTRY
{
	GX_ACQUISITION_FRAME_RATE_MODE_OFF = 0,   ///< \Chinese 关闭帧率控制功能					\English Turn off frame rate control mode
	GX_ACQUISITION_FRAME_RATE_MODE_ON  = 1    ///< \Chinese 开启帧率控制功能					\English Turn on frame rate control mode
}GX_ACQUISITION_FRAME_RATE_MODE_ENTRY;

typedef enum GX_REGION_SEND_MODE
{
    GX_REGION_SEND_SINGLE_ROI_MODE = 0,   ///< \Chinese 单ROI								\English Single ROI
    GX_REGION_SEND_MULTI_ROI_MODE  = 1    ///< \Chinese 多ROI								\English Multi ROI
}GX_REGION_SEND_MODE;

typedef enum GX_REGION_MODE
{
    GX_REGION_MODE_OFF = 0,   ///< \Chinese 关闭当前选择的区域				\English Close currently selected region
    GX_REGION_MODE_ON  = 1    ///< \Chinese 打开当前选择的区域				\English Open currently selected region
}GX_REGION_MODE;

typedef enum GX_REGION_SELECTOR_ENTRY
{
    GX_REGION_SELECTOR_REGION0 = 0,   ///< \Chinese Region 0							\English Region 0
    GX_REGION_SELECTOR_REGION1 = 1,   ///< \Chinese Region 1							\English Region 1
    GX_REGION_SELECTOR_REGION2 = 2,   ///< \Chinese Region 2							\English Region 2
    GX_REGION_SELECTOR_REGION3 = 3,   ///< \Chinese Region 3							\English Region 3
    GX_REGION_SELECTOR_REGION4 = 4,   ///< \Chinese Region 4							\English Region 4
    GX_REGION_SELECTOR_REGION5 = 5,   ///< \Chinese Region 5							\English Region 5
    GX_REGION_SELECTOR_REGION6 = 6,   ///< \Chinese Region 6							\English Region 6
    GX_REGION_SELECTOR_REGION7 = 7    ///< \Chinese Region 7							\English Region 7
}GX_REGION_SELECTOR_ENTRY;

typedef enum GX_SHARPNESS_MODE_ENTRY
{
	GX_SHARPNESS_MODE_OFF = 0,   ///< \Chinese 关闭锐化功能						\English Turn off sharpness mode
	GX_SHARPNESS_MODE_ON  = 1    ///< \Chinese 开启锐化功能						\English Turn on sharpness mode
}GX_SHARPNESS_MODE_ENTRY;

typedef enum GX_NOISE_REDUCTION_MODE_ENTRY
{
	GX_NOISE_REDUCTION_MODE_OFF = 0,   ///< \Chinese 关闭降噪功能						\English Turn off the noise reduction function
	GX_NOISE_REDUCTION_MODE_ON  = 1    ///< \Chinese 开启降噪功能						\English Turn on the noise reduction function
}GX_NOISE_REDUCTION_MODE_ENTRY;

typedef enum GX_SHADING_CORRECTION_MODE_ENTRY
{
	GX_SHADING_CORRECTION_MODE_FLAT_FIELD_CORRECTION        = 0,   ///< \Chinese 平场校正							\English Flat field correction
	GX_SHADING_CORRECTION_MODE_PARALLAX_CORRECTION          = 1,   ///< \Chinese 视差校正							\English Parallax correction
	GX_SHADING_CORRECTION_MODE_TAILOR_FLAT_FIELD_CORRECTION = 2,   ///< \Chinese 定制平场校正						\English Custom flat field correction
	GX_SHADING_CORRECTION_MODE_DEVICE_FLAT_FIELD_CORRECTION = 3    ///< \Chinese 设备平场校正						\English Equipment flat field correction
}GX_SHADING_CORRECTION_MODE_ENTRY;

typedef enum GX_FFC_GENERATE_STATUS_ENTRY
{
	GX_FFC_GENERATE_STATUS_IDLE          = 0,   ///< \Chinese 闲置								\English idle
	GX_FFC_GENERATE_STATUS_WAITING_IMAGE = 1,   ///< \Chinese 等待图像							\English Waiting image
	GX_FFC_GENERATE_STATUS_FINISH        = 2,   ///< \Chinese 完成								\English complete
}GX_FFC_GENERATE_STATUS_ENTRY;

typedef enum GX_FFC_COEFFICIENT_ENTRY
{
	GX_FFC_COEFFICIENT_SET0  = 0,    ///< \Chinese 平场校正系数Set0					\English Flat field correction coefficient Set0
	GX_FFC_COEFFICIENT_SET1  = 1,    ///< \Chinese 平场校正系数Set1					\English Flat field correction coefficient Set1
	GX_FFC_COEFFICIENT_SET2  = 2,    ///< \Chinese 平场校正系数Set2					\English Flat field correction coefficient Set2
	GX_FFC_COEFFICIENT_SET3  = 3,    ///< \Chinese 平场校正系数Set3					\English Flat field correction coefficient Set3
	GX_FFC_COEFFICIENT_SET4  = 4,    ///< \Chinese 平场校正系数Set4					\English Flat field correction coefficient Set4
	GX_FFC_COEFFICIENT_SET5  = 5,    ///< \Chinese 平场校正系数Set5					\English Flat field correction coefficient Set5
	GX_FFC_COEFFICIENT_SET6  = 6,    ///< \Chinese 平场校正系数Set6					\English Flat field correction coefficient Set6
	GX_FFC_COEFFICIENT_SET7  = 7,    ///< \Chinese 平场校正系数Set7					\English Flat field correction coefficient Set7
	GX_FFC_COEFFICIENT_SET8  = 8,    ///< \Chinese 平场校正系数Set8					\English Flat field correction coefficient Set8
	GX_FFC_COEFFICIENT_SET9  = 9,    ///< \Chinese 平场校正系数Set9					\English Flat field correction coefficient Set9
	GX_FFC_COEFFICIENT_SET10 = 10,   ///< \Chinese 平场校正系数Set10					\English Flat field correction coefficient Set10
	GX_FFC_COEFFICIENT_SET11 = 11,   ///< \Chinese 平场校正系数Set11					\English Flat field correction coefficient Set11
	GX_FFC_COEFFICIENT_SET12 = 12,   ///< \Chinese 平场校正系数Set12					\English Flat field correction coefficient Set12
	GX_FFC_COEFFICIENT_SET13 = 13,   ///< \Chinese 平场校正系数Set13					\English Flat field correction coefficient Set13
	GX_FFC_COEFFICIENT_SET14 = 14,   ///< \Chinese 平场校正系数Set14					\English Flat field correction coefficient Set14
	GX_FFC_COEFFICIENT_SET15 = 15,   ///< \Chinese 平场校正系数Set15					\English Flat field correction coefficient Set15
}GX_FFC_COEFFICIENT_ENTRY;

typedef enum GX_FFC_EXPECTED_GRAY_VALUE_ENABLE_ENTRY
{
	GX_FFC_EXPECTED_GRAY_VALUE_ENABLE_OFF = 0,   ///< \Chinese 关闭平场校正期望灰度值使能		\English Disable flat field correction expected gray value
	GX_FFC_EXPECTED_GRAY_VALUE_ENABLE_ON  = 1,   ///< \Chinese 开启平场校正期望灰度值使能		\English Enable flat field correction expected gray value
}GX_FFC_EXPECTED_GRAY_VALUE_ENABLE_ENTRY;

typedef enum GX_DSNU_SELECTOR_ENTRY
{
	GX_DSNU_SELECTOR_DEFAULT = 0,    ///< \Chinese 暗场校正系数Default				\English Dark field correction coefficient Default
	GX_DSNU_SELECTOR_SET0    = 1,    ///< \Chinese 暗场校正系数Set0					\English Dark field correction coefficient Set0
	GX_DSNU_SELECTOR_SET1    = 2,    ///< \Chinese 暗场校正系数Set1					\English Dark field correction coefficient Set1
	GX_DSNU_SELECTOR_SET2    = 3,    ///< \Chinese 暗场校正系数Set2					\English Dark field correction coefficient Set2
	GX_DSNU_SELECTOR_SET3    = 4,    ///< \Chinese 暗场校正系数Set3					\English Dark field correction coefficient Set3
	GX_DSNU_SELECTOR_SET4    = 5,    ///< \Chinese 暗场校正系数Set4					\English Dark field correction coefficient Set4
	GX_DSNU_SELECTOR_SET5    = 6,    ///< \Chinese 暗场校正系数Set5					\English Dark field correction coefficient Set5
	GX_DSNU_SELECTOR_SET6    = 7,    ///< \Chinese 暗场校正系数Set6					\English Dark field correction coefficient Set6
	GX_DSNU_SELECTOR_SET7    = 8,    ///< \Chinese 暗场校正系数Set7					\English Dark field correction coefficient Set7
	GX_DSNU_SELECTOR_SET8    = 9,    ///< \Chinese 暗场校正系数Set8					\English Dark field correction coefficient Set8
	GX_DSNU_SELECTOR_SET9    = 10,   ///< \Chinese 暗场校正系数Set9					\English Dark field correction coefficient Set9
	GX_DSNU_SELECTOR_SET10   = 11,   ///< \Chinese 暗场校正系数Set10					\English Dark field correction coefficient Set10
	GX_DSNU_SELECTOR_SET11   = 12,   ///< \Chinese 暗场校正系数Set11					\English Dark field correction coefficient Set11
	GX_DSNU_SELECTOR_SET12   = 13,   ///< \Chinese 暗场校正系数Set12					\English Dark field correction coefficient Set12
	GX_DSNU_SELECTOR_SET13   = 14,   ///< \Chinese 暗场校正系数Set13					\English Dark field correction coefficient Set13
	GX_DSNU_SELECTOR_SET14   = 15,   ///< \Chinese 暗场校正系数Set14					\English Dark field correction coefficient Set14
	GX_DSNU_SELECTOR_SET15   = 16,   ///< \Chinese 暗场校正系数Set15					\English Dark field correction coefficient Set15
}GX_DSNU_SELECTOR_ENTRY;

typedef enum GX_DSNU_GENERATE_STATUS_ENTRY
{
	GX_DSNU_GENERATE_STATUS_IDLE          = 0,   ///< \Chinese 闲置								\English idle
	GX_DSNU_GENERATE_STATUS_WAITING_IMAGE = 1,   ///< \Chinese 等待图像							\English Waiting image
	GX_DSNU_GENERATE_STATUS_FINISH        = 2,   ///< \Chinese 完成								\English complete
}GX_DSNU_GENERATE_STATUS_ENTRY;


typedef enum GX_PRNU_SELECTOR_ENTRY
{
	GX_PRNU_SELECTOR_DEFAULT = 0,    ///< \Chinese 暗场校正系数Default				\English Bright field correction coefficient Default
	GX_PRNU_SELECTOR_SET0    = 1,    ///< \Chinese 暗场校正系数Set0					\English Bright field correction coefficient Set0
	GX_PRNU_SELECTOR_SET1    = 2,    ///< \Chinese 暗场校正系数Set1					\English Bright field correction coefficient Set1
	GX_PRNU_SELECTOR_SET2    = 3,    ///< \Chinese 暗场校正系数Set2					\English Bright field correction coefficient Set2
	GX_PRNU_SELECTOR_SET3    = 4,    ///< \Chinese 暗场校正系数Set3					\English Bright field correction coefficient Set3
	GX_PRNU_SELECTOR_SET4    = 5,    ///< \Chinese 暗场校正系数Set4					\English Bright field correction coefficient Set4
	GX_PRNU_SELECTOR_SET5    = 6,    ///< \Chinese 暗场校正系数Set5					\English Bright field correction coefficient Set5
	GX_PRNU_SELECTOR_SET6    = 7,    ///< \Chinese 暗场校正系数Set6					\English Bright field correction coefficient Set6
	GX_PRNU_SELECTOR_SET7    = 8,    ///< \Chinese 暗场校正系数Set7					\English Bright field correction coefficient Set7
	GX_PRNU_SELECTOR_SET8    = 9,    ///< \Chinese 暗场校正系数Set8					\English Bright field correction coefficient Set8
	GX_PRNU_SELECTOR_SET9    = 10,   ///< \Chinese 暗场校正系数Set9					\English Bright field correction coefficient Set9
	GX_PRNU_SELECTOR_SET10   = 11,   ///< \Chinese 暗场校正系数Set10					\English Bright field correction coefficient Set10
	GX_PRNU_SELECTOR_SET11   = 12,   ///< \Chinese 暗场校正系数Set11					\English Bright field correction coefficient Set11
	GX_PRNU_SELECTOR_SET12   = 13,   ///< \Chinese 暗场校正系数Set12					\English Bright field correction coefficient Set12
	GX_PRNU_SELECTOR_SET13   = 14,   ///< \Chinese 暗场校正系数Set13					\English Bright field correction coefficient Set13
	GX_PRNU_SELECTOR_SET14   = 15,   ///< \Chinese 暗场校正系数Set14					\English Bright field correction coefficient Set14
	GX_PRNU_SELECTOR_SET15   = 16,   ///< \Chinese 暗场校正系数Set15					\English Bright field correction coefficient Set15
}GX_PRNU_SELECTOR_ENTRY;

typedef enum GX_PRNU_GENERATE_STATUS_ENTRY
{
	GX_PRNU_GENERATE_STATUS_IDLE          = 0,   ///< \Chinese 闲置								\English idle
	GX_PRNU_GENERATE_STATUS_WAITING_IMAGE = 1,   ///< \Chinese 等待图像							\English Waiting image
	GX_PRNU_GENERATE_STATUS_FINISH        = 2,   ///< \Chinese 完成								\English complete
}GX_PRNU_GENERATE_STATUS_ENTRY;

typedef enum GX_CXP_LINK_CONFIGURATION_ENTRY
{
	GX_CXP_LINK_CONFIGURATION_CXP6_X1  = 0x00010048,   ///< \Chinese 连接配置CXP6_X1					\English Connection configuration status CXP6_X1
	GX_CXP_LINK_CONFIGURATION_CXP12_X1 = 0x00010058,   ///< \Chinese 连接配置CXP12_X1					\English Connection configuration status CXP12_X1
	GX_CXP_LINK_CONFIGURATION_CXP6_X2  = 0x00020048,   ///< \Chinese 连接配置CXP6_X2					\English Connection configuration status CXP6_X2
	GX_CXP_LINK_CONFIGURATION_CXP12_X2 = 0x00020058,   ///< \Chinese 连接配置CXP12_X2					\English Connection configuration status CXP12_X2
	GX_CXP_LINK_CONFIGURATION_CXP6_X4  = 0x00040048,   ///< \Chinese 连接配置CXP6_X4					\English Connection configuration status CXP6_X4
	GX_CXP_LINK_CONFIGURATION_CXP12_X4 = 0x00040058,   ///< \Chinese 连接配置CXP12_X4					\English Connection configuration status CXP12_X4
	GX_CXP_LINK_CONFIGURATION_CXP3_X1  = 0x00010038,   ///< \Chinese CXP连接配置状态CXP3_X1			\English CXP Connection configuration status CXP3_X1
	GX_CXP_LINK_CONFIGURATION_CXP3_X2  = 0x00020038,   ///< \Chinese CXP连接配置状态CXP3_X2			\English CXP Connection configuration status CXP3_X2
	GX_CXP_LINK_CONFIGURATION_CXP3_X4  = 0x00040038,   ///< \Chinese CXP连接配置状态CXP3_X4			\English CXP Connection configuration status CXP3_X4

}GX_CXP_LINK_CONFIGURATION_ENTRY;

typedef enum GX_CXP_LINK_CONFIGURATION_PREFERRED_ENTRY
{
	GX_CXP_LINK_CONFIGURATION_PREFERRED_CXP12_X4 = 0x00040058,   ///< \Chinese 预设连接配置CXP12_X4				\English Preset connection configuration CXP12_X4
}GX_CXP_LINK_CONFIGURATION_PREFERRED_ENTRY;

typedef enum GX_CXP_LINK_CONFIGURATION_STATUS_ENTRY
{
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP6_X1  = 0x00010048,   ///< \Chinese CXP连接配置状态CXP6_X1			\English CXP Connection configuration status CXP6_X1
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP12_X1 = 0x00010058,   ///< \Chinese CXP连接配置状态CXP12_X1			\English CXP Connection configuration status CXP12_X1
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP6_X2  = 0x00020048,   ///< \Chinese CXP连接配置状态CXP6_X2			\English CXP Connection configuration status CXP6_X2
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP12_X2 = 0x00020058,   ///< \Chinese CXP连接配置状态CXP12_X2			\English CXP Connection configuration status CXP12_X2
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP6_X4  = 0x00040048,   ///< \Chinese CXP连接配置状态CXP6_X4			\English CXP Connection configuration status CXP6_X4
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP12_X4 = 0x00040058,   ///< \Chinese CXP连接配置状态CXP12_X4			\English CXP Connection configuration status CXP12_X4
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP3_X1  = 0x00010038,   ///< \Chinese CXP连接配置状态CXP3_X1			\English CXP Connection configuration status CXP3_X1
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP3_X2  = 0x00020038,   ///< \Chinese CXP连接配置状态CXP3_X2			\English CXP Connection configuration status CXP3_X2
	GX_CXP_LINK_CONFIGURATION_STATUS_CXP3_X4  = 0x00040038,   ///< \Chinese CXP连接配置状态CXP3_X4			\English CXP Connection configuration status CXP3_X4
}GX_CXP_LINK_CONFIGURATION_STATUS_ENTRY;

typedef enum GX_CXP_CONNECTION_SELECTOR_ENTRY
{
	GX_CXP_CONNECTION_SELECTOR_0 = 0,   ///< \Chinese 连接选择0							\English Connection selection 0
	GX_CXP_CONNECTION_SELECTOR_1 = 1,   ///< \Chinese 连接选择1							\English Connection selection 1
	GX_CXP_CONNECTION_SELECTOR_2 = 2,   ///< \Chinese 连接选择2							\English Connection selection 2
	GX_CXP_CONNECTION_SELECTOR_3 = 3,   ///< \Chinese 连接选择3							\English Connection selection 3
}GX_CXP_CONNECTION_SELECTOR_ENTRY;

typedef enum GX_CXP_CONNECTION_TEST_MODE_ENTRY
{
	GX_CXP_CONNECTION_TEST_MODE_OFF   = 0,   ///< \Chinese 关闭连接测试模式					\English Turn off connection test mode
	GX_CXP_CONNECTION_TEST_MODE_MODE1 = 1,   ///< \Chinese 触发连接测试模式					\English Trigger connection test mode
}GX_CXP_CONNECTION_TEST_MODE_ENTRY;

typedef enum GX_SEQUENCER_MODE_ENTRY
{
	GX_SEQUENCER_MODE_OFF = 0,   ///< \Chinese 关闭序列器模式					\English Close sequencer mode
	GX_SEQUENCER_MODE_ON  = 1,   ///< \Chinese 打开序列器模式					\English Open sequencer mode
}GX_SEQUENCER_MODE_ENTRY;

typedef enum GX_SEQUENCER_CONFIGURATION_MODE_ENTRY
{
	GX_SEQUENCER_CONFIGURATION_MODE_OFF = 0,   ///< \Chinese 关闭序列器配置模式				\English Turn off sequencer configuration mode
	GX_SEQUENCER_CONFIGURATION_MODE_ON  = 1,   ///< \Chinese 打开序列器配置模式				\English Open sequencer configuration mode
}GX_SEQUENCER_CONFIGURATION_MODE_ENTRY;

typedef enum GX_SEQUENCER_FEATURE_SELECTOR_ENTRY
{
	GX_SEQUENCER_FEATURE_SELECTOR_FLAT_FIELD_CORRECTION = 0,   ///< \Chinese 序列功能选择						\English Sequence function selection
}GX_SEQUENCER_FEATURE_SELECTOR_ENTRY;

typedef enum GX_SEQUENCER_TRIGGER_SOURCE_ENTRY
{
	GX_SEQUENCER_TRIGGER_SOURCE_FRAME_START = 7,   ///< \Chinese 序列触发源 FrameStart				\English Sequence Trigger FrameStart

}GX_SEQUENCER_TRIGGER_SOURCE_ENTRY;

typedef enum GX_BINNING_SELECTOR_ENTRY
{
	GX_BINNING_SELECTOR_REGION0 = 0,   ///< \Chinese BINNING模式为Region0              \English BINNING mode is Region0
	GX_BINNING_SELECTOR_SENSOR  = 1,   ///< \Chinese BINNING模式为Sensor               \English BINNING mode is Sensor
}GX_BINNING_SELECTOR_ENTRY;

typedef enum GX_BINNING_HORIZONTAL_MODE_ENTRY
{
    GX_BINNING_HORIZONTAL_MODE_SUM     = 0,   ///< \Chinese BINNING水平值和					\English Horizontal value sum of BINNING
    GX_BINNING_HORIZONTAL_MODE_AVERAGE = 1,   ///< \Chinese BINNING水平值平均值				\English Average horizontal value of BINNING
}GX_BINNING_HORIZONTAL_MODE_ENTRY;

typedef enum GX_BINNING_VERTICAL_MODE_ENTRY
{
    GX_BINNING_VERTICAL_MODE_SUM     = 0,   ///< \Chinese BINNING垂直值和					\English Vertical value sum of BINNING
    GX_BINNING_VERTICAL_MODE_AVERAGE = 1,   ///< \Chinese BINNING垂直值平均值				\English Average Vertical value of BINNING
}GX_BINNING_VERTICAL_MODE_ENTRY;

typedef enum GX_SENSOR_SHUTTER_MODE_ENTRY
{
	GX_SENSOR_SHUTTER_MODE_GLOBAL      = 0,   ///< \Chinese 所有的像素同时曝光且曝光时间相等					\English All pixels are exposed at the same time and the exposure time is equal
	GX_SENSOR_SHUTTER_MODE_ROLLING     = 1,   ///< \Chinese 所有的像素曝光时间相等，但曝光起始时间不同		\English All pixels have the same exposure time, but the exposure start time is different
	GX_SENSOR_SHUTTER_MODE_GLOBALRESET = 2,   ///< \Chinese 所有的像素曝光起始时间相同，但曝光时间不想等		\English The exposure start time of all pixels is the same, but the exposure time is not the same
}GX_SENSOR_SHUTTER_MODE_ENTRY;


typedef enum GX_ACQUISITION_STATUS_SELECTOR_ENTRY
{
    GX_ACQUISITION_STATUS_SELECTOR_ACQUISITION_TRIGGER_WAIT = 0,   ///< \Chinese 采集触发等待						\English Acquisition trigger waiting
    GX_ACQUISITION_STATUS_SELECTOR_FRAME_TRIGGER_WAIT       = 1,   ///< \Chinese 帧触发等待						\English Frame trigger waiting
}GX_ACQUISITION_STATUS_SELECTOR_ENTRY;

typedef enum GX_GAMMA_MODE_ENTRY
{
    GX_GAMMA_SELECTOR_SRGB = 0,   ///< \Chinese 默认Gamma校正						\English Default gamma correction
    GX_GAMMA_SELECTOR_USER = 1,   ///< \Chinese 用户自定义Gamma校正				\English User-defined gamma correction
}GX_GAMMA_MODE_ENTRY;

typedef enum GX_LIGHT_SOURCE_PRESET_ENTRY
{
	GX_LIGHT_SOURCE_PRESET_OFF                     = 0,
	GX_LIGHT_SOURCE_PRESET_CUSTOM                  = 1,
	GX_LIGHT_SOURCE_PRESET_DAYLIGHT_6500K          = 2,
	GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K          = 3,
	GX_LIGHT_SOURCE_PRESET_COOL_WHITE_FLUORESCENCE = 4,
	GX_LIGHT_SOURCE_PRESET_INCA                    = 5,
}GX_LIGHT_SOURCE_PRESET_ENTRY;

typedef enum GX_COLOR_TRANSFORMATION_MODE_ENTRY
{
    GX_COLOR_TRANSFORMATION_SELECTOR_RGB_TO_RGB = 0,   ///< \Chinese 默认颜色校正						\English Default color transformation
    GX_COLOR_TRANSFORMATION_SELECTOR_USER       = 1,   ///< \Chinese 用户自定义颜色校正				\English User-defined color transformation
}GX_COLOR_TRANSFORMATION_MODE_ENTRY;

typedef enum GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_ENTRY
{
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN00 = 0,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN01 = 1,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN02 = 2,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN10 = 3,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN11 = 4,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN12 = 5,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN20 = 6,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN21 = 7,
    GX_COLOR_TRANSFORMATION_VALUE_SELECTOR_GAIN22 = 8,
}GX_COLOR_TRANSFORMATION_VALUE_ENTRY;

typedef enum GX_TIMER_SELECTOR_ENTRY
{
	GX_TIMER_SELECTOR_TIMER1 = 1,   ///< \Chinese 定时器1								\English Timer1
	GX_TIMER_SELECTOR_TIMER2 = 2,   ///< \Chinese 定时器2								\English Timer2
	GX_TIMER_SELECTOR_TIMER3 = 3,   ///< \Chinese 定时器3								\English Timer3
}GX_TIMER_SELECTOR_ENTRY;

typedef enum GX_TIMER_TRIGGER_SOURCE_ENTRY
{
	GX_TIMER_TRIGGER_SOURCE_EXPOSURE_START = 1,    ///< \Chinese 接收曝光信号开始计时					\English Exposure start
	GX_TIMER_TRIGGER_SOURCE_LINE10         = 10,   ///< \Chinese 接收引脚10信号开始计时				\English Receive pin 10 signal start timing
	GX_TIMER_TRIGGER_SOURCE_LINE14         = 14,   ///< \Chinese 接收引脚14信号开始计时				\English Receive pin 14 signal start timing
	GX_TIMER_TRIGGER_SOURCE_STROBE         = 16,   ///< \Chinese 接收闪光灯信号开始计时				\English Start timing when receiving flash signal
}GX_TIMER_TRIGGER_SOURCE_ENTRY;

typedef enum GX_COUNTER_SELECTOR_ENTRY
{
	GX_COUNTER_SELECTOR_COUNTER1 = 1,   ///< \Chinese 计时器1								\English Counter1
	GX_COUNTER_SELECTOR_COUNTER2 = 2,   ///< \Chinese 计时器2								\English Counter2
}GX_COUNTER_SELECTOR_ENTRY;

typedef enum GX_COUNTER_EVENT_SOURCE_ENTRY
{
	GX_COUNTER_EVENT_SOURCE_FRAME_START         = 1,   ///< \Chinese 统计 "帧开始" 事件的数量				\English Frame start
	GX_COUNTER_EVENT_SOURCE_FRAME_TRIGGER       = 2,   ///< \Chinese 统计 "帧触发" 事件的数量				\English Count the number of "frame triggered" events
	GX_COUNTER_EVENT_SOURCE_ACQUISITION_TRIGGER = 3,   ///< \Chinese 统计 "采集触发" 事件的数量			\English Count the number of "Collection Triggered" events
	GX_COUNTER_EVENT_SOURCE_OFF                 = 4,   ///< \Chinese 关闭									\English Close
	GX_COUNTER_EVENT_SOURCE_SOFTWARE            = 5,   ///< \Chinese 统计 "软触发" 事件的数量				\English Count the number of "soft trigger" events
	GX_COUNTER_EVENT_SOURCE_LINE0               = 6,   ///< \Chinese 统计 "Line 0 触发" 事件的数量			\English Count the number of "Line 0 Triggered" events
	GX_COUNTER_EVENT_SOURCE_LINE1               = 7,   ///< \Chinese 统计 "Line 1 触发" 事件的数量			\English Count the number of "Line 1 Triggered" events
	GX_COUNTER_EVENT_SOURCE_LINE2               = 8,   ///< \Chinese 统计 "Line 2 触发" 事件的数量			\English Count the number of "Line 2 Triggered" events
	GX_COUNTER_EVENT_SOURCE_LINE3               = 9,   ///< \Chinese 统计 "Line 3 触发" 事件的数量			\English Count the number of "Line 3 Triggered" events

}GX_COUNTER_EVENT_SOURCE_ENTRY;

typedef enum GX_COUNTER_RESET_SOURCE_ENTRY
{
	GX_COUNTER_RESET_SOURCE_OFF         = 0,   ///< \Chinese 关闭									\English Counter reset off
	GX_COUNTER_RESET_SOURCE_SOFTWARE    = 1,   ///< \Chinese 软触发								\English Software
	GX_COUNTER_RESET_SOURCE_LINE0       = 2,   ///< \Chinese 引脚 0								\English Line 0
	GX_COUNTER_RESET_SOURCE_LINE1       = 3,   ///< \Chinese 引脚 1								\English Line 1
	GX_COUNTER_RESET_SOURCE_LINE2       = 4,   ///< \Chinese 引脚 2								\English Line 2
	GX_COUNTER_RESET_SOURCE_LINE3       = 5,   ///< \Chinese 引脚 3								\English Line 3
	GX_COUNTER_RESET_SOURCE_COUNTER2END = 6,   ///< \Chinese Counter2End							\English Counter2End
	GX_COUNTER_RESET_SOURCE_CXPTRIGGER0 = 8,   ///< \Chinese CxpTrigger0							\English CxpTrigger0
	GX_COUNTER_RESET_SOURCE_CXPTRIGGER1 = 9,   ///< \Chinese CxpTrigger1							\English CxpTrigger1

}GX_COUNTER_RESET_SOURCE_ENTRY;

typedef enum GX_COUNTER_TRIGGER_SOURCE_ENTRY
{
	GX_COUNTER_TRIGGER_SOURCE_OFF      = 0,   ///< \Chinese 关闭									\English Close
	GX_COUNTER_TRIGGER_SOURCE_SOFTWARE = 1,   ///< \Chinese 软触发								\English Software trigger
	GX_COUNTER_TRIGGER_SOURCE_LINE0    = 2,   ///< \Chinese 引脚 0								\English Line 0
	GX_COUNTER_TRIGGER_SOURCE_LINE1    = 3,   ///< \Chinese 引脚 1								\English Line 1
	GX_COUNTER_TRIGGER_SOURCE_LINE2    = 4,   ///< \Chinese 引脚 2								\English Line 2
	GX_COUNTER_TRIGGER_SOURCE_LINE3    = 5,   ///< \Chinese 引脚 3								\English Line 3
}GX_COUNTER_TRIGGER_SOURCE_ENTRY;

typedef enum GX_COUNTER_RESET_ACTIVATION_ENTRY
{
	GX_COUNTER_RESET_ACTIVATION_RISING_EDGE = 1,   ///< \Chinese 在信号的上升沿重置计数器				\English Rising edge counter reset.
}GX_COUNTER_RESET_ACTIVATION_ENTRY;

typedef enum GX_USER_DATA_FILED_SELECTOR_ENTRY
{
	GX_USER_DATA_FILED_0 = 0,   ///< \Chinese Flash数据区域0						\English Flash data area 0
	GX_USER_DATA_FILED_1 = 1,   ///< \Chinese Flash数据区域1						\English Flash data area 1
	GX_USER_DATA_FILED_2 = 2,   ///< \Chinese Flash数据区域2						\English Flash data area 2
	GX_USER_DATA_FILED_3 = 3,   ///< \Chinese Flash数据区域3						\English Flash data area 3
}GX_USER_DATA_FILED_SELECTOR_ENTRY;

typedef enum GX_REMOVE_PARAMETER_LIMIT_ENTRY
{
	GX_ENUM_REMOVE_PARAMETER_LIMIT_OFF = 0,   ///< \Chinese 关闭									\English Close
	GX_ENUM_REMOVE_PARAMETER_LIMIT_ON  = 1,   ///< \Chinese 开启									\English Open
}GX_REMOVE_PARAMETER_LIMIT_ENTRY;

typedef enum GX_FLAT_FIELD_CORRECTION_ENTRY
{
	GX_ENUM_FLAT_FIELD_CORRECTION_OFF = 0,   ///< \Chinese 关闭									\English Close
	GX_ENUM_FLAT_FIELD_CORRECTION_ON  = 1,   ///< \Chinese 开启									\English Open
}GX_FLAT_FIELD_CORRECTION_ENTRY;

typedef enum GX_DEVICE_TEMPERATURE_SELECTOR_ENTRY
{
 GX_DEVICE_TEMPERATURE_SELECTOR_SENSOR    = 1,   ///< \Chinese 传感器								\English Sensor
 GX_DEVICE_TEMPERATURE_SELECTOR_MAINBOARD = 2,   ///< \Chinese 主板									\English Mainboard
}GX_DEVICE_TEMPERATURE_SELECTOR_ENTRY;

typedef enum GX_STOP_ACQUISITION_MODE_ENTRY
{
    GX_STOP_ACQUISITION_MODE_GENERAL = 0,   ///< \Chinese 普通停采								\English general stop acquisition mode
    GX_STOP_ACQUISITION_MODE_LIGHT   = 1,   ///< \Chinese 轻量级停采							\English light stop acquisition mode
} GX_STOP_ACQUISITION_MODE_ENTRY;

typedef enum GX_EXPOSURE_TIME_MODE_ENTRY
{
	GX_EXPOSURE_TIME_MODE_ULTRASHORT = 0,   ///< \Chinese 极小曝光								\English Ultrashort exposure
	GX_EXPOSURE_TIME_MODE_STANDARD   = 1,   ///< \Chinese 标准									\English Standard exposure
} GX_EXPOSURE_TIME_MODE_ENTRY;

typedef enum GX_ENUM_SATURATION_MODE_ENTRY
{
	GX_ENUM_SATURATION_OFF = 0,   ///< \Chinese 关闭									\English Close
	GX_ENUM_SATURATION_ON  = 1,   ///< \Chinese 开启									\English Open
}GX_ENUM_SATURATION_MODE_ENTRY;

typedef enum GX_ENUM_STATIC_DEFECT_CORRECTION_ENTRY
{
	GX_ENUM_STATIC_DEFECT_CORRECTION_OFF = 0,   ///< \Chinese 关闭									\English Close
	GX_ENUM_STATIC_DEFECT_CORRECTION_ON  = 1,   ///< \Chinese 开启									\English Open
}GX_ENUM_STATIC_DEFECT_CORRECTION_ENTRY;

typedef enum GX_2D_NOISE_REDUCTION_MODE_ENTRY
{
    GX_2D_NOISE_REDUCTION_MODE_OFF    = 0,   ///< \Chinese 关闭2D降噪模式						\English Close 2d noise reduction mode
    GX_2D_NOISE_REDUCTION_MODE_LOW    = 1,   ///< \Chinese 低									\English low
    GX_2D_NOISE_REDUCTION_MODE_MIDDLE = 2,   ///< \Chinese 中									\English middle
    GX_2D_NOISE_REDUCTION_MODE_HIGH   = 3,   ///< \Chinese 高									\English high
}GX_2D_NOISE_REDUCTION_MODE_ENTRY;

typedef enum GX_3D_NOISE_REDUCTION_MODE_ENTRY
{
    GX_3D_NOISE_REDUCTION_MODE_OFF    = 0,   ///< \Chinese 关闭3D降噪模式						\English Close 3d noise reduction mode
    GX_3D_NOISE_REDUCTION_MODE_LOW    = 1,   ///< \Chinese 低									\English low
    GX_3D_NOISE_REDUCTION_MODE_MIDDLE = 2,   ///< \Chinese 中									\English middle
    GX_3D_NOISE_REDUCTION_MODE_HIGH   = 3,   ///< \Chinese 高									\English high
}GX_3D_NOISE_REDUCTION_MODE_ENTRY;

typedef enum GX_HDR_MODE_ENTRY
{
    GX_HDR_MODE_OFF        = 0,   ///< \Chinese 关闭HDR模式							\English Close HDR mode
    GX_HDR_MODE_CONTINUOUS = 1,   ///< \Chinese 连续HDR模式							\English continue HDR mode
}GX_HDR_MODE_ENTRY;

typedef enum GX_MGC_MODE_ENTRY
{
    GX_MGC_MODE_OFF        = 0,   ///< \Chinese 关闭多帧灰度控制模式					\English Close multi-frame grey scale control mode
    GX_MGC_MODE_TWO_FRAME  = 1,   ///< \Chinese 两帧灰度控制模式						\English Two frames of grey scale control
    GX_MGC_MODE_FOUR_FRAME = 2,   ///< \Chinese 四帧灰度控制模式						\English Four frames of grey scale control
}GX_MGC_CONTROL_MODE_ENTRY;

typedef enum GX_TIMER_TRIGGER_ACTIVATION_ENTRY
{
    GX_TIMER_TRIGGER_ACTIVATION_RISINGEDGE = 0,   ///< \Chinese 定时器上升沿触发						\English Trigger of rising edge of timer
}GX_TIMER_TRIGGER_ACTIVATION_ENTRY;

typedef enum GX_ACQUISITION_BURST_MODE_ENTRY
{
	GX_ENUM_ACQUISITION_BURST_MODE_STANDARD   = 0,   ///< \Chinese 标准模式								\English Standard mode
	GX_ENUM_ACQUISITION_BURST_MODE_HIGH_SPEED = 1,   ///< \Chinese 高速模式								\English High-speed mode
}GX_ACQUISITION_BURST_MODE_ENTRY;

typedef enum GX_LOWPOWER_MODE_ENTRY
{
    GX_LOWPOWER_MODE_OFF = 0,   ///< \Chinese 未进入低功耗模式						\English Exit low power consumption mode
    GX_LOWPOWER_MODE_ON  = 1,   ///< \Chinese 进入低功耗模式						\English Enter low power consumption mode
}GX_LOWPOWER_MODE_ENTRY;

typedef enum GX_CLOSE_CCD_ENTRY
{
    GX_CLOSE_CCD_OFF = 0,   ///< \Chinese 正常模式								\English Normal mode
    GX_CLOSE_CCD_ON  = 1,   ///< \Chinese 进入CCD低功耗模式						\English Enter CCD low power consumption mode
}GX_CLOSE_CCD_ENTRY;

typedef enum GX_SENSOR_SELECTOR_ENTRY
{
    GX_SENSOR_SELECTOR_CMOS1 = 0,   ///< \Chinese 选择CMOS1传感器						\English Select CMOS 1 Sensor
    GX_SENSOR_SELECTOR_CCD1  = 1,   ///< \Chinese 选择CCD1传感器						\English Select CCD 1 Sensor
}GX_SENSOR_SELECTOR_ENTRY;

typedef enum GX_IMU_CONFIG_ACC_RANGE_ENTRY
{
    GX_IMU_CONFIG_ACC_RANGE_16G = 2,   ///< \Chinese 加速计测量范围为16g					\English The measurement range of the accelerometer is 16g
    GX_IMU_CONFIG_ACC_RANGE_8G  = 3,   ///< \Chinese 加速计测量范围为8g					\English The measurement range of the accelerometer is 8g
    GX_IMU_CONFIG_ACC_RANGE_4G  = 4,   ///< \Chinese 加速计测量范围为4g					\English The measurement range of the accelerometer is 4g
    GX_IMU_CONFIG_ACC_RANGE_2G  = 5,   ///< \Chinese 加速计测量范围为5g					\English The measurement range of the accelerometer is 5g
}GX_IMU_CONFIG_ACC_RANGE_ENTRY;

typedef enum GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_SWITCH_ENTRY
{
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_ON  = 0,   ///< \Chinese 打开加速计低通滤波开关				\English Open the accelerometer low-pass filter switch
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_OFF = 1,   ///< \Chinese 关闭加速计低通滤波开关				\English Close the accelerometer low-pass filter switch
}GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_SWITCH_ENTRY;

typedef enum GX_IMU_CONFIG_ACC_ODR_ENTRY
{
    GX_IMU_CONFIG_ACC_ODR_1000HZ = 0,    ///< \Chinese 加速计输出数据率为1000Hz				\English The accelerometer output data rate is 1000Hz
    GX_IMU_CONFIG_ACC_ODR_500HZ  = 1,    ///< \Chinese 加速计输出数据率为500Hz				\English The accelerometer output data rate is 500Hz
    GX_IMU_CONFIG_ACC_ODR_250Hz  = 2,    ///< \Chinese 加速计输出数据率为250Hz				\English The accelerometer output data rate is 250Hz
    GX_IMU_CONFIG_ACC_ODR_125Hz  = 3,    ///< \Chinese 加速计输出数据率为125Hz				\English The accelerometer output data rate is 125Hz
    GX_IMU_CONFIG_ACC_ODR_63Hz   = 4,    ///< \Chinese 加速计输出数据率为63Hz				\English The accelerometer output data rate is 63Hz
    GX_IMU_CONFIG_ACC_ODR_31Hz   = 5,    ///< \Chinese 加速计输出数据率为31Hz				\English The accelerometer output data rate is 31Hz
    GX_IMU_CONFIG_ACC_ODR_16Hz   = 6,    ///< \Chinese 加速计输出数据率为16Hz				\English The accelerometer output data rate is 16Hz
    GX_IMU_CONFIG_ACC_ODR_2000Hz = 8,    ///< \Chinese 加速计输出数据率为2000Hz				\English The accelerometer output data rate is 2000Hz
    GX_IMU_CONFIG_ACC_ODR_4000Hz = 9,    ///< \Chinese 加速计输出数据率为4000Hz				\English The accelerometer output data rate is 4000Hz
    GX_IMU_CONFIG_ACC_ODR_8000Hz = 10,   ///< \Chinese 加速计输出数据率为8000Hz				\English The accelerometer output data rate is 8000Hz
}GX_IMU_CONFIG_ACC_ODR_ENTRY;

typedef enum GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY
{
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ODR040 = 0,   ///< \Chinese 加速计加速计低通截止频率为ODR×0.40	\English Accelerometer The accelerometer low-pass cutoff frequency is ODR 0.40
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ODR025 = 1,   ///< \Chinese 加速计加速计低通截止频率为ODR×0.25	\English Accelerometer The accelerometer low-pass cutoff frequency is ODR 0.25
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ODR011 = 2,   ///< \Chinese 加速计加速计低通截止频率为ODR×0.11	\English Accelerometer The accelerometer low-pass cutoff frequency is ODR 0.11
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ODR004 = 3,   ///< \Chinese 加速计加速计低通截止频率为ODR×0.04	\English Accelerometer The accelerometer low-pass cutoff frequency is ODR 0.04
    GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ODR002 = 4,   ///< \Chinese 加速计加速计低通截止频率为ODR×0.02	\English Accelerometer The accelerometer low-pass cutoff frequency is ODR 0.02
}GX_IMU_CONFIG_ACC_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY;

typedef enum GX_IMU_CONFIG_GYRO_RANGE_ENTRY
{
    GX_IMU_CONFIG_GYRO_RANGE_125DPS  = 2,   ///< \Chinese 陀螺仪X方向测量范围为125dps			\English The measuring range of gyroscope in X direction is 125dps
    GX_IMU_CONFIG_GYRO_RANGE_250DPS  = 3,   ///< \Chinese 陀螺仪X方向测量范围为250dps			\English The measuring range of gyroscope in X direction is 250dps
    GX_IMU_CONFIG_GYRO_RANGE_500DPS  = 4,   ///< \Chinese 陀螺仪X方向测量范围为500dps			\English The measuring range of gyroscope in X direction is 500dps
    GX_IMU_CONFIG_GYRO_RANGE_1000DPS = 5,   ///< \Chinese 陀螺仪X方向测量范围为1000dps			\English The measuring range of gyroscope in X direction is 1000dps
    GX_IMU_CONFIG_GYRO_RANGE_2000DPS = 6,   ///< \Chinese 陀螺仪X方向测量范围为2000dps			\English The measuring range of gyroscope in X direction is 2000dps
}GX_IMU_CONFIG_GYRO_RANGE_ENTRY;

typedef enum GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_SWITCH_ENTRY
{
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_ON  = 0,   ///< \Chinese 开启陀螺仪低通滤波					\English Turn on the gyro low-pass filter
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_OFF = 1,   ///< \Chinese 关闭陀螺仪低通滤波					\English Turn off the gyro low-pass filter
}GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_SWITCH_ENTRY;

typedef enum GX_IMU_CONFIG_GYRO_ODR_ENTRY
{
    GX_IMU_CONFIG_GYRO_ODR_1000HZ = 0,    ///< \Chinese 陀螺仪输出数据率为1000Hz				\English Gyroscope output data rate is 1000Hz
    GX_IMU_CONFIG_GYRO_ODR_500HZ  = 1,    ///< \Chinese 陀螺仪输出数据率为500Hz				\English Gyroscope output data rate is 500Hz
    GX_IMU_CONFIG_GYRO_ODR_250HZ  = 2,    ///< \Chinese 陀螺仪输出数据率为250Hz				\English Gyroscope output data rate is 250Hz
    GX_IMU_CONFIG_GYRO_ODR_125HZ  = 3,    ///< \Chinese 陀螺仪输出数据率为125Hz				\English Gyroscope output data rate is 125Hz
    GX_IMU_CONFIG_GYRO_ODR_63HZ   = 4,    ///< \Chinese 陀螺仪输出数据率为63Hz				\English Gyroscope output data rate is 63Hz
    GX_IMU_CONFIG_GYRO_ODR_31HZ   = 5,    ///< \Chinese 陀螺仪输出数据率为31Hz				\English Gyroscope output data rate is 31Hz
    GX_IMU_CONFIG_GYRO_ODR_4KHZ   = 9,    ///< \Chinese 陀螺仪输出数据率为4000Hz				\English Gyroscope output data rate is 4000Hz
    GX_IMU_CONFIG_GYRO_ODR_8KHZ   = 10,   ///< \Chinese 陀螺仪输出数据率为8000Hz				\English Gyroscope output data rate is 8000Hz
    GX_IMU_CONFIG_GYRO_ODR_16KHZ  = 11,   ///< \Chinese 陀螺仪输出数据率为16Hz				\English Gyroscope output data rate is 16Hz
    GX_IMU_CONFIG_GYRO_ODR_32KHZ  = 12,   ///< \Chinese 陀螺仪输出数据率为32Hz				\English Gyroscope output data rate is 32Hz
}GX_IMU_CONFIG_GYRO_ODR_ENTRY;

typedef enum GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY
{
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF2000HZ = 2000,   ///< \Chinese 加速计加速计低通截止频率为2000Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 2000Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF1600HZ = 1600,   ///< \Chinese 加速计加速计低通截止频率为1600Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 1600Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF1525HZ = 1525,   ///< \Chinese 加速计加速计低通截止频率为1525Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 1525Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF1313HZ = 1313,   ///< \Chinese 加速计加速计低通截止频率为1313Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 1313Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF1138HZ = 1138,   ///< \Chinese 加速计加速计低通截止频率为1138Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 1138Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF1000HZ = 1000,   ///< \Chinese 加速计加速计低通截止频率为1000Hz		\English Accelerometer The accelerometer low-pass cutoff frequency is 1000Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF863HZ  = 863,    ///< \Chinese 加速计加速计低通截止频率为863Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 863Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF638HZ  = 638,    ///< \Chinese 加速计加速计低通截止频率为638Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 638Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF438HZ  = 438,    ///< \Chinese 加速计加速计低通截止频率为438Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 438Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF313HZ  = 313,    ///< \Chinese 加速计加速计低通截止频率为313Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 313Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF213HZ  = 213,    ///< \Chinese 加速计加速计低通截止频率为213Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 213Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF219HZ  = 219,    ///< \Chinese 加速计加速计低通截止频率为219Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 219Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF363HZ  = 363,    ///< \Chinese 加速计加速计低通截止频率为363Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 363Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF320HZ  = 320,    ///< \Chinese 加速计加速计低通截止频率为320Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 320Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF250HZ  = 250,    ///< \Chinese 加速计加速计低通截止频率为250Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 250Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF200HZ  = 200,    ///< \Chinese 加速计加速计低通截止频率为200Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 200Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF181HZ  = 181,    ///< \Chinese 加速计加速计低通截止频率为181Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 181Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF160HZ  = 160,    ///< \Chinese 加速计加速计低通截止频率为160Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 160Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF125HZ  = 125,    ///< \Chinese 加速计加速计低通截止频率为125Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 125Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF100HZ  = 100,    ///< \Chinese 加速计加速计低通截止频率为100Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 100Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF90HZ   = 90,     ///< \Chinese 加速计加速计低通截止频率为90Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 90Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF80HZ   = 80,     ///< \Chinese 加速计加速计低通截止频率为80Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 80Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF63HZ   = 63,     ///< \Chinese 加速计加速计低通截止频率为63Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 63Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF50HZ   = 50,     ///< \Chinese 加速计加速计低通截止频率为50Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 50Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF45HZ   = 45,     ///< \Chinese 加速计加速计低通截止频率为45Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 45Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF40HZ   = 40,     ///< \Chinese 加速计加速计低通截止频率为40Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 40Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF31HZ   = 31,     ///< \Chinese 加速计加速计低通截止频率为31Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 31Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF25HZ   = 25,     ///< \Chinese 加速计加速计低通截止频率为25Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 25Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF23HZ   = 23,     ///< \Chinese 加速计加速计低通截止频率为23Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 23Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF20HZ   = 20,     ///< \Chinese 加速计加速计低通截止频率为20Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 20Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF15HZ   = 15,     ///< \Chinese 加速计加速计低通截止频率为15Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 15Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF13HZ   = 13,     ///< \Chinese 加速计加速计低通截止频率为13Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 13Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF11HZ   = 11,     ///< \Chinese 加速计加速计低通截止频率为11Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 11Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF10HZ   = 10,     ///< \Chinese 加速计加速计低通截止频率为10Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 10Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF8HZ    = 8,      ///< \Chinese 加速计加速计低通截止频率为8Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 8Hz
    GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_GYROLPF6HZ    = 6,      ///< \Chinese 加速计加速计低通截止频率为6Hz			\English Accelerometer The accelerometer low-pass cutoff frequency is 6Hz
}GX_IMU_CONFIG_GYRO_ODR_LOW_PASS_FILTER_FREQUENCY_ENTRY;

typedef enum GX_IMU_TEMPERATURE_ODR_ENTRY
{
    GX_IMU_TEMPERATURE_ODR_500HZ = 0,   ///< \Chinese 温度计输出数据率为500Hz					\English The output data rate of the thermometer is 500Hz
    GX_IMU_TEMPERATURE_ODR_250HZ = 1,   ///< \Chinese 温度计输出数据率为250Hz 					\English The output data rate of the thermometer is 250Hz
    GX_IMU_TEMPERATURE_ODR_125HZ = 2,   ///< \Chinese 温度计输出数据率为125Hz					\English The output data rate of the thermometer is 125Hz
    GX_IMU_TEMPERATURE_ODR_63HZ  = 3,   ///< \Chinese 温度计输出数据率为63Hz					\English The output data rate of the thermometer is 63Hz
}GX_IMU_TEMPERATURE_ODR_ENTRY;

typedef enum GX_SERIALPORT_SELECTOR_ENTRY
{
	GX_SERIALPORT_SERIALPORT_0 = 0,   ///< \Chinese 串口0										\English Serial port 0

}GX_SERIALPORT_SELECTOR_ENTRY;

typedef enum GX_SERIALPORT_SOURCE_ENTRY
{
	GX_SERIALPORT_SERIALPORT_SOURCE_OFF    = 0,   ///< \Chinese 串口输入源开关							\English Serial port input source switch
	GX_SERIALPORT_SERIALPORT_SOURCE_LINE_0 = 1,   ///< \Chinese 串口输入源0								\English Serial port input source 0
	GX_SERIALPORT_SERIALPORT_SOURCE_LINE_1 = 2,   ///< \Chinese 串口输入源1								\English Serial port input source 1
	GX_SERIALPORT_SERIALPORT_SOURCE_LINE_2 = 3,   ///< \Chinese 串口输入源2								\English Serial port input source 2
	GX_SERIALPORT_SERIALPORT_SOURCE_LINE_3 = 4,   ///< \Chinese 串口输入源3								\English Serial port input source 3

}GX_SERIALPORT_SOURCE_ENTRY;

typedef enum GX_SERIALPORT_BAUNDRATE_ENTRY
{
	GX_SERIALPORT_BAUNDRATE_9600   = 5,   ///< \Chinese 串口波特率为9600Hz						\English Serial port baud rate is 9600Hz
	GX_SERIALPORT_BAUNDRATE_19200  = 6,   ///< \Chinese 串口波特率为19200Hz						\English Serial port baud rate is 19200Hz
	GX_SERIALPORT_BAUNDRATE_38400  = 7,   ///< \Chinese 串口波特率为38400Hz						\English Serial port baud rate is 38400Hz
	GX_SERIALPORT_BAUNDRATE_76800  = 8,   ///< \Chinese 串口波特率为76800Hz						\English Serial port baud rate is 76800Hz
	GX_SERIALPORT_BAUNDRATE_115200 = 9,   ///< \Chinese 串口波特率为115200Hz						\English Serial port baud rate is 115200Hz
}GX_SERIALPORT_BAUNDRATE_ENTRY;

typedef enum GX_SERIALPORT_STOP_BITS_ENTRY
{
	GX_SERIALPORT_STOP_BITS_ONE        = 0,   ///< \Chinese Bit1										\English Bit1
	GX_SERIALPORT_STOP_BITS_ONEANDHALF = 1,   ///< \Chinese Bit1AndHalf								\English Bit1AndHalf
	GX_SERIALPORT_STOP_BITS_TWO        = 2,   ///< \Chinese Bit2										\English Bit2
}GX_SERIALPORT_STOP_BITS_ENTRY;

typedef enum GX_SERIALPORT_PARITY_ENTRY
{
	GX_SERIALPORT_PARITY_NONE  = 0,   ///< \Chinese None										\English None
	GX_SERIALPORT_PARITY_ODD   = 1,   ///< \Chinese 奇数										\English Odd number
	GX_SERIALPORT_PARITY_EVEN  = 2,   ///< \Chinese 偶数										\English even numbers
	GX_SERIALPORT_PARITY_MARK  = 3,   ///< \Chinese 标记										\English sign
	GX_SERIALPORT_PARITY_SPACE = 4,   ///< \Chinese 空白										\English blank
}GX_SERIALPORT_PARITY_ENTRY;

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取所有节点名称
\param        [in]hPort                    Handle句柄
\param        [in]pFeatureNameList        节点名称存储位置
\param        [in|out]pCount              给定pFeatureNameList数组容量, 返回实际写入的个数
\return     GX_STATUS_SUCCESS                操作成功，没有发生错误
            GX_STATUS_NOT_INIT_API            没有调用GXInitLib初始化库
            GX_STATUS_INVALID_PARAMETER        用户输入的指针为NULL
            GX_STATUS_INVALID_HANDLE        用户传入非法的句柄，或者关闭已经被关闭的设备
            其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the read and write attributes of the node.
\param        [in]hPort                    The handle.
\param        [in]pFeatureNameList        Node name storage location.
\param        [in|out]pCount                Given the pFeatureNameList array capacity, return the number of actual writes.
\return     GX_STATUS_SUCCESS                The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API            The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_PARAMETER        The pointer that the user input is NULL.
            GX_STATUS_INVALID_HANDLE        The illegal handle that the user introduces, or reclose the device.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetFeatureNameList(GX_PORT_HANDLE hPort, GX_FEATURE_NAME *pFeatureNameList, uint32_t *pCount);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取功能控制码对应的字符串描述
\attention  此接口专门用来获取功能名称描述信息，方便用户做UI程序
\param		[in]hDevice     设备句柄
\param		[in]featureID   功能码ID
\param		[out]pszName    用户输入的字符串缓冲区地址,字符串长度包含末尾结束符'\0'
\param		[in,out]pnSize  用户输入的表示字符串缓冲区地址的长度,单位字节。
						如果用户输入的pszName为NULL：
						[out]pnSize返回需要的实际长度。
						如果用户输入的pszName非NULL：
						[in]pnSize为用户分配的buffer大小；
						[out]pnSize返回实际填充buffer大小；
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
           其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the string description for the feature code.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pszName            The character string buffer address that the user inputs. The character string
									length includes the end terminator '\0'.
\param      [in,out]pnSize          The length of the character string buffer address that the user inputs. Unit: byte.
									If pszName is NULL:
									[out]pnSize   Return the actual size of the character string.
									If pszName is not NULL:
									[in]pnSize    The size of the buffer that the user allocated.
									[out]pnSize   Return the actual filled buffer size.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_NEED_MORE_BUFFER      The buffer that the user allocated is too small.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetFeatureName(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, char* pszName, size_t* pnSize);

// ---------------------------------------------------------------------------
/**
\Chinese：
\brief      获取节点的类型
\param		[in]hPort					Handle句柄
\param		[in]strName					节点名称
\param		[out]type      				节点类型
\return     GX_STATUS_SUCCESS				操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API			没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER		用户输入的指针为NULL
			GX_STATUS_INVALID_HANDLE		用户传入非法的句柄，或者关闭已经被关闭的设备
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the read and write attributes of the node.
\param		[in]hPort					The handle.
\param		[in]strName					Node name.
\param		[in|out]type				type.
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER		The pointer that the user input is NULL.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
// ---------------------------------------------------------------------------
GX_API GXGetFeatureType(GX_PORT_HANDLE hPort, const char *strName, GX_FEATURE_TYPE *type);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      查询当前相机是否支持某功能
\attention  不支持某功能有两个情况： 1、通过查询相机寄存器，查到当前相机当前不支持此功能
                                     2、相机XML描述文件中没有此功能的描述节点
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pbIsImplemented 如果支持则返回true，如果不支持则返回false
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Inquire the current camera whether support a special feature. Usually the camera does not support a
			feature means that:
			1) By inquiring the camera register, the current camera really does not support this feature.
			2) There is no description of this feature in the current camera description file.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pbIsImplemented    To return the result whether is support this feature. If support, then returns
									true, if not support, false will return.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXIsImplemented(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsImplemented);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      查询某功能码当前是否可读
\attention  某些功能的可读属性是随着其它节点的当前值改变的，可用此接口实时查询功能当前是否可读
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pbIsReadable 用来返回结果，如果可读则返回true，如果不可读则返回false。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Inquire if a feature code is currently readable.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pbIsReadable       To return the result whether the feature code ID is readable. If readable,
									then will return true, if not readable, false will return.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXIsReadable(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsReadable);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      查询某功能码当前是否可写
\attention  某些功能的可写属性是随着其它节点的当前值改变的，可用此接口实时查询功能当前是否可写
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pbIsWritable 用来返回结果，如果可写则返回true，如果不可写则返回false。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Inquire if a feature code is currently writable.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pbIsWritable       To return the result whether the feature code ID is writable. If writable,
									then will return true, if not writable, false will return.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXIsWritable(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbIsWritable);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取布尔型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pbValue 用来返回当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the value of bool type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pbValue            The pointer that point to the bool value returned.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetBool(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool* pbValue);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置布尔型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[in]pbValue   用户设置的当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the value of bool type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]bValue              The bool value that the user will set.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetBool(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, bool bValue);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取Int类型值的最小值、最大值、步长等描述信息
\attention  某些属性的范围可能受其他功能的影响，可用调用此接口查询当前实际范围
\param		[in]hDevice    设备句柄
\param		[in]featureID  功能码ID
\param		[out]pIntRange 范围描述结构体
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the minimum value, maximum value and steps of the int type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pIntRange          The structure of range description. Reference GX_INT_RANGE.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not read the int range.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetIntRange(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, GX_INT_RANGE* pIntRange);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnValue 用来返回当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the current value of the int type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnValue            Point to the pointer of the current value returned.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not read.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetInt(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t* pnValue);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[in]pnValue   用户设置的当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_OUT_OF_RANGE        用户传入值越界
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the value of int type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]pnValue             The value that the user will set.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_OUT_OF_RANGE          The value that the user introduces is across the border, smaller
											than the minimum, or larger than the maximum, or is not an integer
											multiple of the step.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetInt(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t nValue);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取Float类型值的最小值、最大值、步长等信息
\attention  某些属性的范围可能受其他功能的影响，可用调用此接口查询当前实际范围
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pFloatRange 范围描述结构体
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the minimum value, maximum value, steps and unit of the float type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pFloatRange        The description structure pointer of float type. Reference the GX_FLOAT_RANGE.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not read the range of the float type.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetFloatRange(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, GX_FLOAT_RANGE* pFloatRange);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取浮点类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pdValue  用来返回当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the value of float type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pdValue            Point to the pointer of the float value returned.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetFloat(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, double* pdValue);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置浮点类型值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[in]dValue    设置值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_OUT_OF_RANGE        用户传入值越界
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the value of float type.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]dValue              The float value that the user will set.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_OUT_OF_RANGE          The value that the user introduces is across the border, smaller
											than the minimum, or larger than the maximum, or is not an integer
											multiple of the step.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetFloat(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, double dValue);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取枚举类型值有多少项
\attention  某枚举功能类型的项数是需要查询的，头文件中所罗列的是所有可能的项，但是
            当前相机实际支出多少项，建议用户先查后用。
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnEntryNums 指向项个数的指针
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the number of the options for the enumeration item.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnEntryNums        The pointer that point to the number returned.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnumEntryNums(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, uint32_t* pnEntryNums);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取枚举类型功能的枚举项的每一项的描叙信息和值
\attention  用户做UI程序的时候需要枚举功能项的描述信息；枚举功能项的值建议用户先查
            后用，因为值可能是离散的值，每个枚举功能的可选值，在头文件中都有定义。
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pEnumDescription GX_ENUM_DESCRIPTION数组指针，返回的枚举描述信息
\param		[in,out]pBufferSize 用户传入的GX_ENUM_DESCRIPTION数组的大小，大为字节
							如果pEnumDescription为NULL：
							[out]pnBufferSize为实际需要的buffer大小
							如果pEnumDescription非NULL：
							[in]pnBufferSize为用户分配的buffer大小
							[out]pnBufferSize返回实际填充buffer大小
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the description information of the enumerated type values: the number of enumerated items
			and the value and descriptions of each item, please reference GX_ENUM_DESCRIPTION.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pEnumDescription   The array pointer, used for the enumeration description information returned.
\param      [in,out]pBufferSize     The size of the GX_ENUM_DESCRIPTION array that the user introduces, unit: byte.
									If pEnumDescription is NULL:
									[out]pnBufferSize   The actual size of the buffer needed.
									If pEnumDescription is not NULL:
									[in]pnBufferSize   The size of the buffer that the user allocated.
									[out]pnBufferSize  Return the actual filled buffer size.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_NEED_MORE_BUFFER      The buffer that the user allocates is too small.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnumDescription(GX_DEV_HANDLE hDevice,
                                   GX_FEATURE_ID_CMD featureID,
                                   GX_ENUM_DESCRIPTION* pEnumDescription,
                                   size_t* pBufferSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取枚举型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnValue 用来返回当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the current enumeration value.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnValue            The pointer that point to the return values.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetEnum(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t* pnValue);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置枚举型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[in]pnValue   用户设置的当前值
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户传入值非法
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the enumeration value.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]nValue              The enumeration values that the user will set. The value range can be got
									by the nValue of the GX_ENUM_DESCRIPTION.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetEnum(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t nValue);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取字符串类型值的长度
\attention  此接口与GxGetString接口组合使用,便于用户申请buffer
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnSize   用来返回字符串当前值长度，包含字符串末尾结束符'\0'。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the current value length of the character string type. Unit: byte. The user can allocate the buffer size
			according to the length information that is get from the function, and then call the GXGetString to get the
			character string information.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnSize             The pointer that point to the length value returned. The length value is end with '\0', unit: byte.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetStringLength(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取字符串类型值的最大长度
\attention  此接口与GxGetString接口组合使用,便于用户申请buffer
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnSize   用来返回字符串最大长度，包含字符串末尾结束符'\0'。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the maximum length of the string type value. Unit: byte. The user allocates buffer according to
			the length information obtained, then call the GXGetString to get the string information. This interface
			can get the maximum possible length of the string(including the terminator '\ 0'), but the actual length
			of the string might not be that long, if the user wants to allocate buffer according to the actual string
			length, the user can call the GXGetStringLength interface to get the actual string length.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnSize             The pointer that point to the length value returned. The length value is end with '\0', unit: byte.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetStringMaxLength(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取字符串
\attention  读取字符串之前需要调用GXGetStringLength接口获取长度
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pszContent 用户输入的字符串缓冲区地址,末尾包含结束符'\0'
\param		[in,out]pnSize 表示用户输入的字符串缓冲区地址的长度
						如果pszContent为NULL：
						[out]pnSize为实际需要的buffer大小
						如果pszContent非NULL：
						[in]pnSize为用户分配的buffer大小
						[out]pnSize返回实际填充buffer大小
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the content of the string type value.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pszContent         Point to the string buffer address that the user allocated.
\param      [in,out]pnSize          The length of the string buffer address that the user inputs.
									If pszContent is NULL:
									[out]pnSize  Return the actual size of the buffer needed.
									If pszContent is not NULL:
									[in]pnSize  The size of the buffer that the user allocated.
									[out]pnSize Return the actual filled buffer size.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			GX_STATUS_NEED_MORE_BUFFER      The buffer that the user allocates is too small.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetString(GX_DEV_HANDLE hDevice,
								   GX_FEATURE_ID_CMD featureID,
								   char* pszContent,
								   size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置字符串
\attention  无
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[in]pszContent 用户输入的字符串，字符串末尾包含结束符'\0'
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			 GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
			GX_STATUS_OUT_OF_RANGE        用户写入内容超过字符串最大长度
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the content of the string value.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]pszContent          The string address that the user will set. The string is end with '\0'.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
			GX_STATUS_OUT_OF_RANGE          The maximum length that the content the user writes
											exceeds the string size.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetString(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, char* pszContent);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取buffer类型值的长度
\attention  此接口与GxGetBuffer接口组合使用,便于用户申请buffer
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[out]pnSize   用来返回长度值。
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the length of the chunk data and the unit is byte, the user can apply the buffer based on the
			length obtained, and then call the GXGetBuffer to get the chunk data.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pnSize             The pointer that points to the length value returned. Unit: byte.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetBufferLength(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取buffer数据块
\attention  读取buffer数据块之前需要调用GXGetBufferLength接口获取长度
\param		[in]hDevice 设备句柄
\param		[in]featureID 功能码ID
\param		[out]pBuffer 用户输入的缓冲区地址
\param		[in,out]pnSize 表示用户输入的缓冲区地址的长度
						如果pBuffer为NULL：
						[out]pnSize为实际需要的buffer大小
						如果pBuffer非NULL：
						[in]pnSize为用户分配的buffer大小
						[out]pnSize返回实际填充buffer大小
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			GX_STATUS_INVALID_ACCESS      当前不可访问
			GX_STATUS_NEED_MORE_BUFFER    用户分配的buffer过小
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Get the chunk data.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [out]pBuffer            The pointer that point to the chunk data buffer address that the user applied.
\param      [in,out]pnSize          The length of the buffer address that the user inputs.
									If pBuffer is NULL:
									[out]pnSize  Return the actual size of the buffer needed.
									If pBuffer is not NULL:
									[in]pnSize   The size of the buffer that the user allocated.
									[out]pnSize  Return the actual filled buffer size.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user input is NULL.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			GX_STATUS_NEED_MORE_BUFFER      The buffer that the user allocates is too small.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetBuffer(GX_DEV_HANDLE hDevice,
								   GX_FEATURE_ID_CMD featureID,
								   uint8_t* pBuffer,
								   size_t* pnSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      设置buffer数据块
\attention  无
\param		[in]hDevice   设备句柄
\param		[in]featureID 功能码ID
\param		[in]pBuffer   用户输入的缓冲区地址
\param		[in]nSize     表示用户输入的缓冲区地址的长度
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
			GX_STATUS_OUT_OF_RANGE        用户写入内容超过字符串最大长度
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Set the chunk data.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\param      [in]pBuffer             The pointer that point to the chunk data buffer address that the user will set.
\param      [in]nSize               The length of the buffer address that the user inputs.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
			GX_STATUS_OUT_OF_RANGE          The maximum length that the content the user writes exceeds
											the string size.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSetBuffer(GX_DEV_HANDLE hDevice,
								   GX_FEATURE_ID_CMD featureID,
								   uint8_t* pBuffer,
								   size_t nSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      发送控制命令
\attention  无
\param		[in]hDevice    设备句柄
\param		[in]featureID  功能码ID
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Send the command.
\param      [in]hDevice             The handle of the device.
\param      [in]featureID           The feature code ID.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_NOT_IMPLEMENTED       The feature that is not support currently.
			GX_STATUS_ERROR_TYPE            The featureID type that the user introduces is error.
			GX_STATUS_INVALID_ACCESS        Currently inaccessible, can not write.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXSendCommand(GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注册属性更新回调函数
\attention  用户可通过此接口获取事件数据，详见示例程序
\param		[in]hDevice     设备句柄
\param		[in]pUserParam  用户私有数据
\param		[in]callBackFun 用户注册的回调函数
\param		[in]featureID   功能码
\param		[out]pHCallBack  回调函数句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER   用户传入回调函数非法
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Register the capture callback function, corresponding to GXUnregisterCaptureCallback.
\attention  After sending the start acquisition command, the user can not register the callback function.
			Otherwise, it will return GX_STATUS_INVALID_CALL.
\param      [in]hDevice             The handle of the device.
\param      [in]pUserParam          The private data pointer that the user will use in the callback function.
\param      [in]callBackFun         The callback function that the user will register, for the function type,
									see GXCaptureCallBack.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_INVALID_PARAMETER     The pointer that the user introduces is NULL.
			GX_STATUS_INVALID_CALL          After sending the start acquisition command, the user can
											not register the capture callback function.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXRegisterFeatureCallback(GX_DEV_HANDLE hDevice,
								   void* pUserParam,
								   GXFeatureCallBack  callBackFun,
								   GX_FEATURE_ID_CMD  featureID,
								   GX_FEATURE_CALLBACK_HANDLE *pHCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      注销属性更新回调函数
\attention  与GXRegisterFeatureCallback配套使用，每次注册都必须有相应的注销与之对应
\param		[in]hDevice     设备句柄
\param		[in]featureID   功能码
\param		[out]pHCallBack  回调函数句柄
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Unregister the capture callback function, corresponding to GXRegisterCaptureCallback.
\attention  Before sending the stop acquisition command, the user can not unregister the callback function.
			Otherwise, it will return GX_STATUS_INVALID_CALL.
\param      [in]hDevice             The handle of the device.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			GX_STATUS_INVALID_CALL          After sending the stop acquisition command, the user can not unregister
											the capture callback function.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXUnregisterFeatureCallback(GX_DEV_HANDLE  hDevice, GX_FEATURE_ID_CMD featureID, GX_FEATURE_CALLBACK_HANDLE  hCallBack);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief     枚举所有设备并且获取设备个数,对于千兆网设备此接口仅能枚举同网段设备
\attention 此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
           所以调用GXGetAllDeviceBaseInfo和GXOpenDevice之前需要调用此接口。
           如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回
\param     [out]punNumDevices 返回设备个数
\param     [in]unTimeOut      枚举的超时时间(单位ms)。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST

\English:
\brief      Enumerating currently all available devices in subnet and gets the number of devices.
\param      [out] punNumDevices   The address pointer used to return the number of devices, and the pointer can not be NULL.
\param      [in] unTimeOut        The timeout time of enumeration(unit: ms). If the device is successfully
								  enumerated within the specified timeout time, the value returns immediately.
								  If the device is not enumerated within the specified timeout time, then it
								  waits until the specified timeout time is over and then it returns.
\return     GX_STATUS_SUCCESS              The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API         The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER    The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API  GXUpdateDeviceList(uint32_t* punNumDevices, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief		通过序号打开设备
\param		nDeviceIndex 设备序号，从1开始，例如：1、2、3、4...
\param		phDevice 返回设备句柄
\return		GX_STATUS,捕获底层调用产生的异常，根据异常类型返回不同的错误码

\English:
\brief      Open the device by index, starting from 1.
\param      nDeviceIndex          The index of the device starts from 1, for example: 1, 2, 3, 4...
\param      phDevice              Device handle returned by the interface.
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			GX_STATUS_OUT_OF_RANGE        The index of the user input is bigger than the available devices number.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXOpenDeviceByIndex(uint32_t nDeviceIndex, GX_DEV_HANDLE* phDevice);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取所有设备的基础信息
\attention  此接口调用之前需要调用GXUpdateDeviceList接口，更新库内部设备列表
\param		[out] pDeviceInfo   设备信息结构体指针
\param		[in|out]pBufferSize 设备信息结构体缓冲区大小，单位字节
						   如果pDeviceInfo为NULL：
						   [out]pnBufferSize返回实际大小
						   如果pDeviceInfo非NULL：
						   [in]pnBufferSize为用户分配buffer大小
						   [out]pnBufferSize返回实际填充buffer大小
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the basic information of all devices.
\attention  You should call the GxUpdateDeviceList() interface for an enumeration before calling the function to
			get the device information. Otherwise, the device information that the user gets is inconsistent with
			the device that is currently connected.
\param      [out] pDeviceInfo     The structure pointer of the device information.
\param      [in,out]pBufferSize   The buffer size of device information structure, unit: byte.
								  If pDeviceInfo is NULL:
								  [out]pnBufferSize  Return the actual size of the device information.
								  If pDeviceInfo is not NULL:
								  [in]pnBufferSize   The size of the buffer that the user allocated.
								  [out]pnBufferSize  Return the actual allocated buffer size.
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The pointer that the user input is NULL.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetAllDeviceBaseInfo(GX_DEVICE_BASE_INFO* pDeviceInfo, size_t* pBufferSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      指定设备序号获取设备的网络信息
\attention  此接口调用之前需要调用GXUpdateDeviceList接口，更新库内部设备列表
\param		[in]  nIndex  设备序号，从1开始，例如：1、2、3、4...
\param		[out] pstDeviceIPInfo   设备信息结构体指针
\return		GX_STATUS_SUCCESS    操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
			其它错误情况请参见GX_STATUS_LIST

\English:
\brief      To get the network information of all devices.
\attention  You should call the GxUpdateDeviceList() interface for an enumeration before calling the function to
			get the device information. Otherwise, the device information that the user gets is inconsistent with
			the device that is currently connected.
\param      [in]  nIndex          The serial number of the device.
\param      [out] pstDeviceIPInfo The structure pointer of the device information.
\return     GX_STATUS_SUCCESS             The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API        The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_PARAMETER   The index that the user input is cross the border.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetDeviceIPInfo(uint32_t nIndex, GX_DEVICE_IP_INFO* pstDeviceIPInfo);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      读用户指定寄存器的值
\param		[in]hDevice         设备句柄
\param		[in]ui64Address     寄存器地址
\param		[out]pBuffer        返回寄存器的值，不能为NULL
\param		[in, out]piSize     [in]用户申请的Buffer大小
								[out]成功读取寄存器的值后，返回实际大小
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Read the value of the specified register
\param      [in]hDevice             The handle of the device.
\param      [in]ui64Address         Register address.
\param      [out]pBuffer            Return the value of the register, can not be NULL.
\param      [in, out]piSize         [in]The size of the buffer that the user allocated.
									[out]Return the actual filled buffer size.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXReadRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, void *pBuffer, size_t *piSize);


//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      向用户指定的寄存器中写入用户给定的数据
\param		[in]hDevice         设备句柄
\param		[in]ui64Address     寄存器地址
\param		[in]pBuffer         返回寄存器的值，不能为NULL
\param		[in, out]piSize     [in]用户要写入的Buffer长度
								[out]返回实际写入寄存器的长度
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Write the given data to the specified register.
\param      [in]hDevice             The handle of the device.
\param      [in]ui64Address         Register address.
\param      [in]pBuffer             The value to be written to the register, cannot be NULL.
\param      [in, out]piSize         [in]Buffer size to be written to the register.
									[out]Returns the size actually written to the register.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXWriteRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, const void *pBuffer, size_t *piSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      批量读用户指定寄存器的值（仅限命令值为4字节长度的寄存器,其余类型不保证数据有效性）
\param		[in]hDevice         设备句柄
\param		[in|out]pstEntries  [in]批量读取寄存器的地址及值
								[out]读取到对应寄存器的数据
\param		[in|out]piSize      [in]读取设备寄存器的个数
								[out]成功读取寄存器的个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Batch read the value of the user specified register(only the command value is 4 bytes long register, other types do not guarantee data validity)
\param		[in]hDevice         The device handle
\param		[in|out]pstEntries  [in]Read register addresses and values in batches
								[out]The data is read from the corresponding register
\param		[in|out]piSize      [in]Read the number of device registers
								[out]The number of registers read successfully
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXReadRemoteDevicePortStacked(GX_DEV_HANDLE hDevice, GX_REGISTER_STACK_ENTRY* pstEntries, size_t *piSize);

//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      批量向用户指定的寄存器中写入用户给定的数据（仅限命令值为4字节长度的寄存器）
            调用当前接口后，使用GXGetEnum、GXGetInt、GXGetBool等接口获取到的节点值为修改前值
            可使用GXReadRemoteDevicePortStacked接口获取最新的寄存器值
\param		[in]hDevice         设备句柄
\param		[in]pstEntries      [in]批量写寄存器的地址及值
\param		[in|out]piSize      [in]设置设备寄存器的个数
								[out]成功写寄存器的个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

\English:
\brief      Write user-specified data in batches to user-specified registers(only registers with 4-byte command values)
			After the current interface is invoked, the node value obtained by using interfaces such as GXGetEnum, GXGetInt, and GXGetBool is the previous value
			The latest register values can be obtained using the GXReadRemoteDevicePortStacked interface
\param		[in]hDevice         The device handle
\param		[in]pstEntries      [in]Batch write register addresses and values
\param		[in|out]piSize      [in]Sets the number of device registers
								[out]The number of successful register writes
\return     GX_STATUS_SUCCESS				The operation is successful and no error occurs.
			GX_STATUS_NOT_INIT_API			The GXInitLib initialization library is not called.
			GX_STATUS_INVALID_HANDLE		The illegal handle that the user introduces, or reclose the device.
			The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXWriteRemoteDevicePortStacked(GX_DEV_HANDLE hDevice, const GX_REGISTER_STACK_ENTRY* pstEntries, size_t *piSize);

#ifndef _WIN32
//----------------------------------------------------------------------------------
/**
\Chinese：
\brief      获取请求buffer数
\attention  无
\param		[in]hDevice    设备句柄
\param		[out]pBufferNum  采集缓存的当前buffer数目
\return		GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
			GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
			GX_STATUS_INVALID_ACCESS      当前不可访问
			其它错误情况请参见GX_STATUS_LIST 

\English:
\brief      Get the number of the acquisition buffers.
\param      [in]hDevice             The handle of the device.
\param      [out]pBufferNum         Current number of the acquisition buffers.
\return     GX_STATUS_SUCCESS               The operation is successful and no error occurs.
            GX_STATUS_NOT_INIT_API          The GXInitLib initialization library is not called.
            GX_STATUS_INVALID_HANDLE        The handle that the user introduces is illegal.
            GX_STATUS_INVALID_PARAMETER     The input parameter that the user introduces is invalid.
            The errors that are not covered above please reference GX_STATUS_LIST.
*/
//----------------------------------------------------------------------------------
GX_API GXGetAcqusitionBufferNumber (GX_DEV_HANDLE hDevice, uint64_t *pBufferNum);
#endif

#endif //GX_LEGACY_H