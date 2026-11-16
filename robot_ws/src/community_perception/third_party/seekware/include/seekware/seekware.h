/*Copyright (c) [2019] [Seek Thermal, Inc.]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The Software may only be used in combination with Seek cores/products.

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _SEEKWARE_SDK_H_
#define _SEEKWARE_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifdef SEEKWARE_API_EXPORT
#	ifdef __cplusplus
#		if defined(WIN32)
#			define SEEKWARE_API extern "C" __declspec(dllexport)
#		else
#			define SEEKWARE_API extern "C" __attribute__((__visibility__("default")))
#		endif
#	else
#		if defined(WIN32)
#			define SEEKWARE_API extern  __declspec(dllexport)
#		else
#			define SEEKWARE_API extern  __attribute__((__visibility__("default")))
#		endif
#	endif
#else
#	ifdef __cplusplus
#		if defined(WIN32)
#			define SEEKWARE_API extern "C" __declspec(dllimport)
#		else
#			define SEEKWARE_API extern "C"
#		endif
#	else
#		if defined(WIN32)
#			define SEEKWARE_API extern __declspec(dllimport)
#		else
#			define SEEKWARE_API extern
#		endif
#	endif
#endif

#if defined(WIN32)
#	define DEPRECATED __declspec(deprecated)
#else
#	define DEPRECATED __attribute__((deprecated))
#endif

#define SEEKWARE_VERSION_MAJOR		  3
#define SEEKWARE_VERSION_MINOR		  4
#define SEEKWARE_VERSION_REVISION	  0
#define SEEKWARE_VERSION_BUILD		  5

// Device related constants
#define RAW_COLS_206                208
#define RAW_ROWS_206                156
#define COLS_206                    206
#define ROWS_206                    156
#define RAW_PIXEL_SIZE_206          2

#define RAW_COLS_320                342
#define RAW_ROWS_320                260
#define COLS_320                    320
#define ROWS_320                    240
#define RAW_PIXEL_SIZE_320          2

#define FRAMEPIXELS_206             (RAW_COLS_206 * RAW_ROWS_206)
#define FRAMEPIXELS_320             (RAW_COLS_320 * RAW_ROWS_320)
#define FRAMESIZE_206               (FRAMEPIXELS_206 * RAW_PIXEL_SIZE_206)
#define FRAMESIZE_320               (FRAMEPIXELS_320 * RAW_PIXEL_SIZE_320)

#define MAX_COLS                    RAW_COLS_320
#define MAX_ROWS                    RAW_ROWS_320
#define MAX_PIXEL_SIZE              RAW_PIXEL_SIZE_320
#define MAXFRAMESIZE                FRAMESIZE_320

#define NR_LUTCOLORS                256

typedef struct seekware_ctx_t seekware_ctx_t;


// Definitions for the sw 'model' field.
// WFOV=Wide Field of View, NFOV=Narrow Field of View, FF=Fast Frame

/*  The sw_model enum below is only used for cameras with firmware versions of 2.X.X.X and below. These enums are deprecated for any camera with
    a newer firmware version, and instead will simply return a sw struct model of SEEK_MODEL_UNSUPPORTED. For cameras that are unsupported, please
    refer to the sw struct modelNumber to access the camera part number.
*/

DEPRECATED typedef enum sw_model {
    SEEK_MODEL_206_WFOV = 0,
    SEEK_MODEL_206_WFOV_FF,
    SEEK_MODEL_206_NFOV,
    SEEK_MODEL_206_NFOV_FF,
    SEEK_MODEL_320_WFOV,
    SEEK_MODEL_320_WFOV_FF,
    SEEK_MODEL_320_NFOV,
    SEEK_MODEL_320_NFOV_FF,
    SEEK_MODEL_UNSUPPORTED
} sw_model;

/**
 Definition for the info param of Seekware_GetSdkInfo
  */
typedef struct sw_sdk_info {
    uint8_t sdk_version_major; // SDK version number
    uint8_t sdk_version_minor;
    uint8_t sdk_build_major;
    uint8_t sdk_build_minor;
    uint8_t lib_version_major; // Library version number
    uint8_t lib_version_minor;
    uint8_t lib_build_major;
    uint8_t lib_build_minor;
} sw_sdk_info;

/**
 * Defines the list of settings.
 * Seekware_SetSetting and Seekware_GetSeting are valid for settings less than or equal to SETTING_THERMOGRAPHY_VERSION
 * Seekware_SetSettingEx and Seekware_GetSettingEx are valid for all settings.
 */
typedef enum sw_settings {                      //  Size in Bytes:  Type:                           SET/GET:                            Description:
    SETTING_ACTIVE_LUT,                         //  4               int                             SET/GET                             display lut used to fill display buffer in GetFrame/(Ex)
    SETTING_TEMP_UNITS,                         //  4               int                             SET/GET                             temperature units
    SETTING_TIMEOUT,                            //  4               int                             SET/GET                             USB timeout
    SETTING_CONTROL,                            //  4               int                             SET/GET                             binary settings (legacy)
    SETTING_EMISSIVITY,                         //  4               int                             SET/GET                             emissivity
    SETTING_BACKGROUND,                         //  4               int                             SET/GET                             background temperature
    SETTING_THERMOGRAPHY_VERSION,               //  4               int                             GET                                 thermography version
    SETTING_TEMP_DIODE_ROOM,                    //  4               float                           GET                                 factory temperature used for estimating environment temp
    SETTING_TEMP_DIODE_SLOPE,                   //  4               float                           GET                                 slope of FPA therm diode
    SETTING_TEMP_DIODE_OFFSET,                  //  4               float                           GET                                 FPA therm diode offset
    SETTING_GLOBAL_THERM_ADJUST,                //  4               sw_global_therm_adjust_t        SET                                 global temperature offset
    SETTING_SCENE_THERM_ADJUST,                 //  8               sw_scene_therm_adjust_t         SET                                 temperature offset for a specific scene
    SETTING_ENVIRONMENT_THERM_ADJUST,           //  8               sw_environment_therm_adjust_t   SET                                 temperature offset for a specific environment
    SETTING_SPECIFIC_THERM_ADJUST,              //  12              sw_specific_therm_adjust_t      SET                                 temperature offset for a specific scene and environment
    SETTING_TRANSIENT_CORRECTION_ENABLE,        //  4               uint32_t                        SET/GET                             transient correction
    SETTING_TRANSIENT_CORRECTION_PARAMS,        //  8               sw_transient_adjust_t           SET/GET                             amplitude and decay for transient correction
    SETTING_SMOOTHING,                          //  4               uint32_t                        SET/GET                             image smoothing
    SETTING_AUTOSHUTTER,                        //  4               uint32_t                        SET/GET                             autoshutter
    SETTING_MINMAX,                             //  24              sw_minmax_t                     GET                                 MIN/MAX with coordinates
    SETTING_SHARPENING,                         //  4               uint32_t                        SET/GET                             image sharpening
    SETTING_ENABLE_TIMESTAMP,                   //  4               uint32_t                        SET/GET                             timestamping
    SETTING_RESET_TIMESTAMP,                    //  4               uint32_t                        SET                                 reset timestamping counter
    SETTING_TRIGGER_SHUTTER,                    //  4               uint32_t                        SET                                 shutter trigger

    // AGC Settings
    SETTING_AGC_MODE,                           //  4               uint32_t                        SET/GET                             AGC Mode (0=Legacy HistEQ, 1=Linear Min/Max, 2=HistEQ)
    SETTING_HISTEQ_BIN_COUNT,                   //  4               uint32_t                        GET                                 number of bins
    SETTING_HISTEQ_INPUT_BIT_DEPTH,             //  4               uint32_t                        GET                                 number of input bits before performing AGC calculation
    SETTING_HISTEQ_OUTPUT_BIT_DEPTH,            //  4               uint32_t                        GET                                 number of output bits after performing AGC calculation
    SETTING_HISTEQ_HIST_WIDTH_COUNTS,           //  4               uint32_t                        GET                                 width of the active histogram
    SETTING_HISTEQ_PLATEAU_VALUE,               //  4               float                           SET/GET                             max percentage of total pixels that can be assigned to a single bin
    SETTING_HISTEQ_GAIN_LIMIT,                  //  4               float                           SET/GET                             max percentage of output colors per sensor count
    SETTING_HISTEQ_GAIN_LIMIT_FACTOR_ENABLE,    //  4               uint32_t                        SET/GET                             gain limit factor
    SETTING_HISTEQ_GAIN_LIMIT_FACTOR,           //  4               float                           GET                                 gain limit factor value
    SETTING_HISTEQ_GAIN_LIMIT_FACTOR_XMAX,      //  4               uint32_t                        SET/GET                             max histogram width where gain limit factor will affect the total gain
    SETTING_HISTEQ_GAIN_LIMIT_FACTOR_YMIN,      //  4               float                           SET/GET                             min gain limit factor value
    SETTING_HISTEQ_ALPHA_TIME,                  //  4               float                           SET/GET                             number of seconds to blend current frame's histogram with previous frame's histogram
    SETTING_HISTEQ_TRIM_LEFT,                   //  4               float                           SET/GET                             percentage of outliers to trim from left side of histogram
    SETTING_HISTEQ_TRIM_RIGHT,                  //  4               float                           SET/GET                             percentage of outliers to trim from right side of histogram
    SETTING_LINMINMAX_MODE,                     //  4               uint32_t                        SET/GET                             linear min/max mode (0=Min and Max Auto, 1=Min and Max Locked, 2=Min Locked and Max Auto, 3=Min Auto and Max Locked)
    SETTING_LINMINMAX_MIN_LOCK,                 //  4               uint32_t                        SET/GET                             lower bound for linear min/max (only used when SETTING_LINMINMAX_MODE = 1 or 2)
    SETTING_LINMINMAX_MAX_LOCK,                 //  4               uint32_t                        SET/GET                             upper bound for linear min/max (only used when SETTING_LINMINMAX_MODE = 1 or 3)
    SETTING_LINMINMAX_ACTIVE_MIN_VALUE,         //  4               uint32_t                        GET                                 min sensor count value in the last scene
    SETTING_LINMINMAX_ACTIVE_MAX_VALUE,         //  4               uint32_t                        GET                                 max sensor count value in the last scene

    FEATURE_OEM = 1000,                         //  Size in bytes and type dependent on setting. Please contact Seek for details.       Use FEATURE_OEM + index with additional settings provided by Seek
    SETTING_LAST = 100000,
} sw_settings;

typedef enum sw_control {
    SEEKWARE_CTRL_SMOOTHING = 1,
    SEEKWARE_CTRL_AUTOSHUTTER = 2
} sw_control;

/**
  Defines the list of LUTs that can be applied to display video frames.
  */
typedef enum sw_display_lut {
    SW_LUT_FIRST = 1,
    SW_LUT_WHITE = 1,
    SW_LUT_BLACK = 17,
    SW_LUT_IRON = 20,
    SW_LUT_COOL = 2,
    SW_LUT_AMBER = 9,
    SW_LUT_INDIGO = 10,
    SW_LUT_TYRIAN = 16,
    SW_LUT_GLORY = 8,
    SW_LUT_ENVY = 16,
     
    SW_LUT_WHITE_NEW = 100,
    SW_LUT_BLACK_NEW,
    SW_LUT_SPECTRA,
    SW_LUT_PRISM,
    SW_LUT_TYRIAN_NEW,
    SW_LUT_AMBER_NEW,
    SW_LUT_IRON_NEW,
    SW_LUT_HI,
    SW_LUT_HILO,

    SW_LUT_USER0 = 40000,
    SW_LUT_USER1,
    SW_LUT_USER2,
    SW_LUT_USER3,
    SW_LUT_USER4,
    SW_LUT_LAST = SW_LUT_USER4
} sw_display_lut;

typedef enum sw_temp_units {
    SW_TEMP_C, ///< Celsius
    SW_TEMP_F, ///< Fahrenheit
    SW_TEMP_K, ///< Kelvin
} sw_temp_units;

/**
  Error codes that are returned from Seekware function calls.
  */
typedef enum sw_retcode {
    SW_RETCODE_NONE,        	///< No error has been detected
    SW_RETCODE_NOTOPENED,   	///< Device is not opened
    SW_RETCODE_OPENEX,      	///< Device is already opened exclusively
    SW_RETCODE_BPARAM,      	///< Bad parameter
    SW_RETCODE_NOFRAME,     	///< Frame processing error
    SW_RETCODE_ERROR,       	///< Generic error
    SW_RETCODE_OFLOW,       	///< Some kind of overflow
    SW_RETCODE_USBERR,      	///< USB had an error; we need to restart the camera
    SW_RETCODE_SETONLY,     	///< Setting is write only
    SW_RETCODE_GETONLY,     	///< Setting is read only
    SW_RETCODE_NOTSUPPORTED,	///< Setting is not supported by version of camera firmware
    SW_RETCODE_INVALIDSETUP,	///< Another setting needs to be configured differently
    SW_RETCODE_DISCONNECTED,  ///< Device is disconnected from the host
    SW_RETCODE_LAST = 5000, 	///< OEM error codes are >= 1000
} sw_retcode;

//Used as the region parameter to Seekware_LoadAppResources / Seekware_StoreAppResources
typedef enum sw_resources_region_t{
	SW_APP_RESOURCES_0 = 11,
	SW_APP_RESOURCES_1,
	SWE_APP_RESOURCES_2,
}sw_resources_region_t;

//Use with SETTING_GLOBAL_THERM_ADJUST and Seekware_SetSettingEx
typedef struct sw_global_therm_adjust_t {
    float offset;
} sw_global_therm_adjust_t;

//Use with SETTING_SCENE_THERM_ADJUST and Seekware_SetSettingEx
typedef struct sw_scene_therm_adjust_t {
    float scene_temp;
    float offset;
} sw_scene_therm_adjust_t;

//Use with SETTING_ENVIRONMENT_THERM_ADJUST and Seekware_SetSettingEx
typedef struct sw_environment_therm_adjust_t {
    float environment_temp;
    float offset;
} sw_environment_therm_adjust_t;

//Use with SETTING_SPECIFIC_THERM_ADJUST and Seekware_SetSettingEx
typedef struct sw_specific_therm_adjust_t {
    float scene_temp;
    float environment_temp;
    float offset;
} sw_specific_therm_adjust_t;

//Use with SETTING_TRANSIENT_CORRECTION_PARAMS and Seekware_SetSettingEx
typedef struct sw_transient_adjust_t {
    float amplitude;
    float decay;
} sw_transient_adjust_t;

//Use with SETTING_MINMAX and Seekware_GetSettingEx/Seekware_SetSettingEx
typedef struct sw_minmax_t {
    float min;
    float max;
    uint32_t minX;
    uint32_t minY;
    uint32_t maxX;
    uint32_t maxY;
} sw_minmax_t;

/**
  This structure contains camera specific information to describe attached
  devices.
  */
typedef struct sw {
    // Device information
    uint16_t model;
    char serialNumber[13];
    char modelNumber[17];
    char manufactureDate[33];
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    uint8_t fw_build_major;
    uint8_t fw_build_minor;
    uint16_t frame_rows;
    uint16_t frame_cols;
    void* lusb_dev_handle;
    void* lusb_status;

    /// Latest return code
    sw_retcode retcode;

    /// Private SDK Context
    seekware_ctx_t* seekware_context;

    uint16_t rawframe_rows;
    uint16_t rawframe_cols;

    void* libusb_device;
} sw, * psw;

/// Find the connected Seekware devices
/*! Search the target environment for all connected devices.<br>
    Then, starting at index zero, fills the given array with a pointer to
    a device structure for each connected device up to length, then sets
    numfound to the number of devices found. If there are more than 'length'
    devices connected fills the array as much as possible, sets numfound = length,
    and returns the error code SW_RETCODE_OFLOW.
    \param  pswlist     A 'length' sized array of pointers. The pointers
                        in this list will be assigned to point to sw structs that describe the attached device.
                        The psw structs allocated by this function will be free'd automatically atexit on Windows,
                        or when the library is unloaded on Linux.
    \param  length      The size of the array 'pswlist'.
    \param  numfound    Returns the number of allocated objects. Note that the
                        remaining part of the given array is not modified by
                        this function.
 */
SEEKWARE_API sw_retcode Seekware_Find(psw pswlist[], int length, int * numfound);

// Preserved for legacy ABI compatibility.
// Notifies the SDK when a POSIX signal is delivered to the application.
SEEKWARE_API DEPRECATED void Seekware_OnSignal(int signal);

// Preserved for legacy ABI compatibility.
// Notifies the SDK when the application exits.
SEEKWARE_API DEPRECATED void Seekware_OnExit(void);

/**
  Open the device for use, populate internal elements in *sw,
  allocate any memory or events necessary for operation then return.
  Open devices are available exclusively to the SDK that opened
  them. A call to this function on a device that is already open
  should return SW_RETCODE_OPENEX.

  @param[in] id A pointer to a Seekware device structure
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_Open(psw id);

/**
  Close the device, release any memory and terminate any events.
  @param id A pointer to a Seekware device structure
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_Close(psw id);

/**
  Returns a structure containing information about the SDK.

  @param[in] id A pointer to a Seekware device structure.
  @param[out] info A pointer to a sw_dev_info structure.
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_GetSdkInfo(psw id, sw_sdk_info * info);

/**
 Updates the camera firmware. You must reboot the Seek device
 * after a sucessful call to this function.

  @param[in] id A pointer to a Seekware device structure.
  @param[in] filename A path to the firmware upgrade file.
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_UploadFirmware(psw id, const char* filename);

/**
  Gets the value of the requested setting.

  @param[in] id A pointer to a Seekware device structure
  @param[in] index The setting index. Must be less than or equal to SETTING_THERMOGRAPHY_VERSION
  @param[out] value A pointer to the location to write the setting value
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_GetSetting(psw id, sw_settings index, int * value);

/**
  Writes the requested setting into value

  @param[in] id A pointer to a Seekware device structure
  @param[in] index The setting index
  @param[out] value A pointer to the storage location of value
    @param[in] bytes size of value in bytes
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_GetSettingEx(psw id, uint32_t index, void* value, uint32_t bytes);

/**
  Sets the value of the requested setting.

  @param[in] id A pointer to a Seekware device structure
  @param[in] index The setting index. Must be less than or equal to SETTING_THERMOGRAPHY_VERSION
  @param[in] value The setting value
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_SetSetting(psw id, sw_settings index, int value);

/**
  Sets the value(s) of the requested setting using the provided value(s)

  @param[in] id A pointer to a Seekware device structure
  @param[in] setting The setting index
  @param[in] value A pointer to the setting value(s)
  @param[in] bytes the size of value in bytes
  @return sw_retcode SW_RETCODE_NONE on success
  */
SEEKWARE_API sw_retcode Seekware_SetSettingEx(psw id, uint32_t setting, void* value, uint32_t bytes);

/**
  Returns the spot temperature for the center 6x6 pixels.

  @param[in] id A pointer to a Seekware device structure
  @param[out] temp A float pointer to the place to put the spot temperature
  @param[out] min A float pointer to the place to put the min temperature
  @param[out] max A float pointer to the place to put the max temperature
 **/
SEEKWARE_API sw_retcode Seekware_GetSpot(psw id, float *temp, float *min, float *max);

/**
   Loads user LUT data to a USER LUT.  Only supports ARGB32.

   @param[in] id A pointer to a Seekware device structure
   @param[in] lut_index The index of the user LUT to set.  SW_LUT_USER0 <= lut_index <= SW_LUT_USER4. 
   @param[in] lut_data  A uint32_t array containing the LUT data to write.  Should be NR_LUTCOLORS elements.
   @param[in] length    The number of LUT elements to write.  Typically NR_LUTCOLORS.
**/
SEEKWARE_API sw_retcode Seekware_SetUserLUT(psw id, uint32_t lut_index, uint32_t *lut_data, uint32_t length);

/**
  @param[in] id A pointer to a Seekware device structure
  @param[out] binary - A pointer to a buffer to hold filtered image data
  @param[out] temperature – A pointer to a buffer to hold temperature data
  @param[out] display – A pointer to a buffer to hold display data
  @return sw_retcode SW_RETCODE_NONE on success

  @note: If a buffer is supplied, then it must be appropriately sized
  based on the data type and the number of pixels in the image. If
  a buffer is not supplied, then the computations and memory
  allocation required for that function shall not be performed.
  */
SEEKWARE_API sw_retcode Seekware_GetImage(psw id, uint16_t * binary, float * temperature, uint32_t * display);

/**
 @param[in] id A pointer to a Seekware device structure
 @param[out] binary - A pointer to a buffer to hold raw image data + 1 telemetry line
 @param[out] temperature – A pointer to a buffer to hold temperature data
 @param[out]  display – A pointer to a buffer to hold temperature data
 @return sw_retcode SW_RETCODE_NONE on success

 @note: If a buffer is supplied, then it must be appropriately sized
 based on the data type and the number of pixels in the image. If
 a buffer is not supplied, then the computations and memory
 allocation required for that function shall not be performed.
 The buffer shall be sized to contain one extra row.
 Sensor telemetry data shall be copied into the extra row following frame data.
 */
SEEKWARE_API sw_retcode Seekware_GetImageEx(psw id, uint16_t * binary, float * temperature, uint32_t * display);

/**
 * Returns a frame of fixed point uint16_t thermography values. 
 * To get temperatures in C apply the folowing formula: Temperature in C = (count / 64) - 40
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @param[out] thermography -- A pointer to a uint16_t frame buffer
  @param[in] num_elements -- number of elements in the display buffer
 @return sw_retcode SW_RETCODE_NONE on success
 */
SEEKWARE_API sw_retcode Seekware_GetThermographyImage(psw id, uint16_t * thermography, uint32_t num_elements);

/**
 * Returns a frame of ARGB display values with auto gain control enabled.
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @param[out] display -- A pointer to a uint32_t frame buffer
 @param[in] num_elements -- number of elements in the display buffer
 @return sw_retcode SW_RETCODE_NONE on success
 */
SEEKWARE_API sw_retcode Seekware_GetDisplayImage(psw id, uint32_t * display, uint32_t num_elements);

/**
 * Starts the Seekware imaging pipeline and wakes up the camera if it's sleeping in low power mode.
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @return sw_retcode SW_RETCODE_NONE on success
 */
SEEKWARE_API sw_retcode Seekware_Start(psw id);

/**
 * Stops the Seekware imaging pipeline and puts the camera into a low power sleep mode.
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @return sw_retcode SW_RETCODE_NONE on success.
 */
SEEKWARE_API sw_retcode Seekware_Stop(psw id);

/**
 * Loads user-specified resources from the camera's onboard flash.
 * NOTE: Image capture must be stopped with Seekware_Stop before calling this function.
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @param region -- The region to load the resources from (each region can store up to 64Kb of data)
 @param [in] data -- A pointer to a buffer where the resources will be loaded.
 @param bytes -- Number of bytes to load.
 @return sw_retcode SW_RETCODE_NONE on success.
 */
SEEKWARE_API sw_retcode Seekware_LoadAppResources(psw id , sw_resources_region_t region, void* data, size_t bytes);

/**
 * Stores user-specifed resources in the camera's onboard flash.
 * NOTE: Image capture must be stopped with Seekware_Stop before calling this function.
 * 
 @param[in] id -- A pointer to a Seekware device structure
 @param region -- The region to store the resources (each region can store up to 64Kb of data)
 @param data -- Data to be written to the specifed region.
 @param bytes -- Number of bytes to store.
 @return sw_retcode SW_RETCODE_NONE on success.
 */
SEEKWARE_API sw_retcode Seekware_StoreAppResources(psw id, sw_resources_region_t region, void* data, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif

