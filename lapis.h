#ifndef COMMANDS_H
#define COMMANDS_H
#include "stdint.h"

#if defined _WIN32 || defined __CYGWIN__
#define HELPER_DLL_IMPORT __declspec(dllimport)
#define HELPER_DLL_EXPORT __declspec(dllexport)
#define HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define HELPER_DLL_IMPORT __attribute__((visibility("default")))
#define HELPER_DLL_EXPORT __attribute__((visibility("default")))
#define HELPER_DLL_LOCAL __attribute__((visibility("hidden")))
#else
#define HELPER_DLL_IMPORT
#define HELPER_DLL_EXPORT
#define HELPER_DLL_LOCAL
#endif
#endif

#ifdef LAPIS_DLL_EXPORTS // defined if we are building the DLL (instead of using it)
#define LAPIS_API HELPER_DLL_EXPORT
#else
#define LAPIS_API HELPER_DLL_IMPORT
#endif // LAPIS_DLL_EXPORTS

#ifdef __cplusplus
extern "C" {
#endif

// Error codes
typedef enum {
    LAPIS_NO_ERROR = 0,
    LAPIS_ERROR_MISSING_ARGUMENT,
    LAPIS_ERROR_INVALID_ARGUMENT,
    LAPIS_ERROR_NO_DEVICE,
    LAPIS_ERROR_ALREADY_CONNECTED,
    LAPIS_ERROR_UNKNOWN_PACKET,
    LAPIS_ERROR_PACKET_OVERFLOW,
    LAPIS_ERROR_WHITELIST_BUSY,
    LAPIS_ERROR_WHITELIST_FAILURE,
    LAPIS_ERROR_USB,
    LAPIS_ERROR_EXCEPTION,
    LAPIS_ERROR_SYSTEM_EXCEPTION,
    LAPIS_ERROR_UNKNOWN
} lapis_error_t;

typedef enum {
	BLEpkt = 0x00,
	FMpkt = 0x01,
	SPECTRUMpkt = 0x02,
    WHITELISTpkt = 0x03,
    GAINpkt = 0x04,
    WHITELISTENTRIESpkt = 0x05,
    DIGITALpkt = 0x08,
    UART0pkt = 0x09,
    UART1pkt = 0x0A,
    UART2pkt = 0x0B,
    UART3pkt = 0x0C,
    BUFFSTATSpkt = 0x0D,
    SPURIOUS = 0xFF,
    NOT_WRITTEN = 0xFFFF
} lapis_packet_t;

/**
 * @brief Get message for last error
 * @return Internal pointer to null terminated string (do not free)
 */
LAPIS_API const char* lapis_error_msg();

/**
 * @brief The lapis_search_result_t struct
 *
 */
struct lapis_handle_t;
struct lapis_search_result_t {
    lapis_handle_t* handle; /**< Opaque pointer to a device for lapis_connect */
    uint32_t serial_number; /**< Serial number of device */
};

/**
 * @brief lapis_search - Search for MorpehLPs
 * @param[out] results - Returns a pointer to a list of search results
 * @param[out] N - Returns the number of results
 * @return Error code
 *
 * Any previous results are freed when lapis_search is called.
 */
LAPIS_API lapis_error_t lapis_search(lapis_search_result_t** results, int* N);

/**
 * @brief Free the last set of search results
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_free_search_results();

/**
 * @brief lapis_connect
 * @param[in] handle - Take from the search results
 * @return Error code
 *
 * This must be run before most other commands
 */
LAPIS_API lapis_error_t lapis_connect(lapis_handle_t* handle);

/**
 * @brief lapis_disconnect
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_disconnect();

/**
 * @brief Called for each packet of data
 */
typedef void (*lapis_data_callback_t)(uint8_t* data, uint32_t len, lapis_error_t err);

/**
 * @brief lapis_set_data_callback
 * @param[in] cb Pointer to callback function
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_data_callback(lapis_data_callback_t cb);

/**
 * @brief lapis_friendly_name
 * @param[out] name Pointer a char array where the friendly name will be written
 * @param[out] maxlen The length of the output buffer
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_friendly_name(char* name, unsigned maxlen);

/**
 * @brief lapis_manufacturing_string
 * @param[out] name
 * @param[out] maxlen
 * @return Error code
 *
 * See lapis_friendly_name
 */
LAPIS_API lapis_error_t lapis_manufacturing_string(char* name, unsigned maxlen);

/**
 * @brief morpehlp_serial_number
 * @param[out] serial Return serial number
 * @return Error code
 */
LAPIS_API lapis_error_t morpehlp_serial_number(uint32_t* serial);

/**
 * @brief lapis_set_detection_threshold
 * @param[in] rssi_uc Uncoded threshold
 * @param[in] rssi_c Coded threshold
 * @param[in] traffic analysis and ICO threshold
 * @return Error code
 *
 * Sets the detection threshold in dB. -128 means no threshold.
 */
LAPIS_API lapis_error_t lapis_set_detection_threshold(int8_t rssi_uc, int8_t rssi_c, int8_t search);

/**
 * @brief lapis_get_detection_threshold
 * @param[out] rssi_uc Returns uncoded threshold
 * @param[out] rssi_c Returns coded threshold
 * @param[out] traffic analysis and ICO threshold
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_detection_threshold(int8_t* rssi_uc, int8_t* rssi_c, int8_t *search);

/**
 * @brief lapis_set_whitelist_algorithm
 * @param[in] mask The bits within the mask determine which algorithms are active
 * @param[in] ttl Determines the time to live in units of 100ms for the age pruning algorithm
 * @param[in] nBad Number of bad CRCs with no good CRC before Search 1/2 or ICO entry is discarded
 * @return Error code
 *
 * Mask bits:
 * 0 Automatic pruning of entries between the same initiator and advertiser addresses
 * 1 Automatic pruning of entries which have seen no packets in the last time-to-live x 100ms
 * 2 Automatic pruning of Search 1/2 & ICO whitelist entries if no good CRC seen
 */
LAPIS_API lapis_error_t lapis_set_whitelist_algorithm(uint16_t mask, uint16_t ttl, uint16_t nBad);

/**
 * @brief lapis_get_whitelist_algorithm
 * @param[out] mask
 * @param[out] ttl
 * @param[out[ nBad
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_whitelist_algorithm(uint16_t* mask, uint16_t *ttl, uint16_t *nBad);

/**
 * @brief lapis_set_buffer_period
 * @param[in] period in 100ms units
 * @param[in] low_water {the low water buffer mark as a percentage. If the free buffer space
 * transitions
 * the low water mark whilst heading in the downwards direction, then a new Buffer Space Message
 * will be generated, irrespective of the reporting period. If this value is set to 0 then the low
 * water mark will be ignored.}
 * @param[in] high_water {the high water buffer mark as a percentage. If the free buffer space
 * transitions the high water mark whilst heading in the upwards direction, then a new Buffer Space
 * Message will be generated, irrespective of the reporting period. If this value is set to 100,
 * then the high water mark will be ignored.}
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_buffer_period(uint16_t period, uint16_t low_water,
                                                uint16_t high_water);

/**
 * @brief lapis_get_buffer_period
 * @param[out] period in 100ms units
 * @param[out] low_water {the low water buffer mark as a percentage. If the free buffer space
 * transitions
 * the low water mark whilst heading in the downwards direction, then a new Buffer Space Message
 * will be generated, irrespective of the reporting period. If this value is set to 0 then the low
 * water mark will be ignored.}
 * @param[out] high_water {the high water buffer mark as a percentage. If the free buffer space
 * transitions the high water mark whilst heading in the upwards direction, then a new Buffer Space
 * Message will be generated, irrespective of the reporting period. If this value is set to 100,
 * then the high water mark will be ignored.}
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_buffer_period(uint16_t* period, uint16_t* low_water,
                                                uint16_t* high_water);

/**
 * @brief lapis_set_wl_period
 * @param[in] period in 100ms units
 * @param[in] age If non-zero entries will be sorted by age, otherwise sorted by access address
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_wl_period(uint16_t period, uint8_t age);

/**
 * @brief lapis_get_wl_period
 * @param[out] period in 100ms units
 * @param[out] age Indicates whether entries are sorted by age or access address
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_wl_period(uint16_t* period, uint8_t *age);

/**
 * @brief lapis_set_spec_period
 * @param[in] period in 64us units
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_spec_period(uint16_t period);

/**
 * @brief lapis_get_spec_period
 * @param[out] period in 64us units
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_spec_period(uint16_t* period);

/**
 * @brief lapis_set_gain_period
 * @param[in] period in 100ms units
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_gain_period(uint16_t period);

/**
 * @brief lapis_get_gain_period
 * @param[out] period in 100ms units
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_gain_period(uint16_t* period);

/**
 * @brief lapis_features
 * @param[out] app
 * @param[out] fpga_version
 * @param[out] fpga_date
 * @param[out] fpga_time
 * @param[out] fpga_features
 * @param[out] version_str Pass in a preallocated char array
 * @param[in] version_str_len
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_features(uint32_t* app, uint32_t* fpga_version, uint32_t* fpga_date,
                                       uint32_t* fpga_time, uint32_t* fpga_features,
                                       char* version_str, unsigned version_str_len);

/**
 * @brief lapis_set_calibration
 * @param[in] offset Arbitrary offset added to reported RSSI and spectrum. 0.5dB units.
 * @param[in] lna LNA gain. 0.5dB units.
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_calibration(uint16_t offset, uint16_t lna);

/**
 * @brief lapis_get_calibration
 * @param[in] offset Arbitrary offset added to reported RSSI and spectrum. 0.5dB units.
 * @param[in] lna LNA gain. 0.5dB units.
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_calibration(uint16_t* offset, uint16_t* lna);

/**
 * @brief lapis_set_gain
 * @param[in] atten Set attenuation from 0-31.5dB in 0.5dB steps
 * @param[in] agc Enable AGC (Set to 0 or 1)
 * @param[in] lna Enables the LNA (Set to 0 or 1)
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_gain(uint8_t atten, uint8_t agc, uint8_t lna);

/**
 * @brief lapis_get_gain
 * @param[out] atten Attenuation in 0.5dB units
 * @param[out] agc Indicates whether the AGC has been enabled
 * @param[out] lna Indicates whether the LNA has been enabled
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_gain(uint8_t* atten, uint8_t *agc, uint8_t *lna);

/**
 * @brief lapis_set_correlator_bit_errors
 * @param[in] Nerr Array of 8 bytes (See below)
 * @return Error code
 *
 * The Set Correlator Bit Errors command allows the host to set the maximum number of bit errors
 * which are allowed in the Lapis preamble and access address correlators. The preamble and
 * access address have separate thresholds. Any bit sequence which has less than or equal to the
 * number of bit errors specified for both the preamble and access address is deemed a valid packet.
 * If more than one access address satisfies the thresholds, the access address with the lowest
 * number of bit errors is selected. If two or more access addresses have identical numbers of
 * errors, then the result of the selection is not deterministic.
 * For uncoded phys the LSB of the access address is corrected using the preamble prior to the
 * correlation operation being performed.
 * For coded phy packets, the correlation is performed after the inner spreading code has been
 * removed but before the outer convolutional code is removed. The number of bits being compared is
 * therefore 20 in the preamble and 64 in the access code.
 * Separate sets of thresholds are available for coded and uncoded phys.
 * For connectionless isochronous channels, the correlation is performed on the preamble and those
 * bits in the access address which are not modified by the AA_diversifier. Since the correlation is
 * performed over fewer bits, the number of permitted bit errors must be reduced if the same rate of
 * spurious detections is to be maintained. Therefore a second set of bit error thresholds are
 * defined for connectionless isochronous channels.
 * The payload consists of eight 8 bit fields which contain the maximum number of permissible bit
 * errors:
 * 1.	aa_uc : the maximum permissible number of errors in the uncoded phy access address.
 * Permissible values are in the range 0 to 2.
 * 2.	pre_uc : the maximum permissible number of errors in the uncoded phy preamble. Permissible
 * values are in the range 0 to 2.
 * 3.	aa_c : the maximum permissible number of errors in the coded phy access address.
 * Permissible values are in the range 0 to 6.
 * 4.	pre_c : the maximum permissible number of errors in the coded phy preamble. Permissible
 * values are in the range 0 to 6.
 * 5.	aa_uc_iso : the maximum permissible number of errors in the uncoded phy access address for
 * use on connectionless isochronous channels. Permissible values are in the range 0 to 2.
 * 6.	pre_uc_iso : the maximum permissible number of errors in the uncoded phy preamble for use
 * on connectionless isochronous channels. Permissible values are in the range 0 to 2.
 * 7.	aa_c_iso : the maximum permissible number of errors in the coded phy access address for
 * use on connectionless isochronous channels. Permissible values are in the range 0 to 6.
 * 8.	pre_c_iso : the maximum permissible number of errors in the coded phy preamble for use on
 * connectionless isochronous channels. Permissible values are in the range 0 to 6.
 *
 */
LAPIS_API lapis_error_t lapis_set_correlator_bit_errors(uint8_t* Nerr);

/**
 * @brief lapis_get_correlator_bit_errors
 * @param[out] Nerr Must pass pointer to array of 8 bytes. See above.
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_correlator_bit_errors(uint8_t* Nerr);

/**
 * @brief lapis_set_whitelist_mask
 * @param[in] mask
 * @return Error code
 * The Set Whitelist Update Mask host command determines who is able to perform update operations on
 * the whitelist.
 *
 * Mask bits:
 * 0 0x01 Connection request packets
 * 1 0x02 AUX_SYNC_IND packets establishing a connectionless isochronous stream
 * 2 0x04 Host via the Update Whitelist command
 * 3 0x08 Extended advertising packets containing a SyncInfo field describing periodic
 *        advertisements
 * 4 0x10 Traffic analysis of 1Mbps & 2Mbps packets
 * 5 0x20 Traffic analysis of coded phy packets
 * 6 0x40 Detection of ICO packets
 */
LAPIS_API lapis_error_t lapis_set_whitelist_mask(uint8_t mask);

/**
 * @brief lapis_get_whitelist_mask
 * @param[out] mask
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_whitelist_mask(uint8_t* mask);

/**
 * @brief lapis_whitelist_add
 * @param[in] access_address
 * @param[in] crc_seed
 * @param[in] flags
 * @return Error code
 *
 * Flags bits
 * 0	0x01	Digital output trigger will be generated on packet detection
 * 1	0x02	Access address is valid for 2Mbps phy
 * 2	0x04	Access address is valid for 1Mbps phy
 * 3	0x08	Access address is valid for 500kbps phy
 * 4	0x10	Access address is valid for 125kbps phy
 *
 * Bits 7:5 of flags denote:
 * 0    Access address derived from connection request
 * 1	Access address is for periodic advertisements
 * 2	Access address is base access address for ICL
 * 3    Access address belongs to an ICO stream
 * 4    Access address derived from 1Mbps & 2Mbps traffic analysis
 * 5    Access address derived from coded phy traffic analysis
 * 6    Access address added by host
 * 7    Access address is either the advertising address or the test address
 */
LAPIS_API lapis_error_t lapis_whitelist_add(uint32_t access_address, uint32_t crc_seed,
                                            uint8_t flags);

/**
 * @brief lapis_whitelist_crc
 * @param[in] access_address
 * @param[out] crc
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_whitelist_crc(uint32_t access_address, uint32_t* crc);

/**
 * @brief lapis_whitelist_flags
 * @param[in] access_address
 * @param[out] flags See lapis_whitelist_add
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_whitelist_flags(uint32_t access_address, uint8_t* flags);

/**
 * @brief lapis_whitelist_delete
 * @param[in] access_address
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_whitelist_delete(uint32_t access_address);

/**
 * @brief lapis_set_capture_mask
 * @param[out] mask
 * @return Error code
 *
 * Mask bits:
 * 0	0x0001	LE packet data
 * 1	0x0002	Not used
 * 2	0x0004	Spectrum data
 * 3	0x0008	Whitelist update messages
 * 4	0x0010	Digital input event messages
 * 5	0x0020	UART #0 data
 * 6	0x0040	UART #1 data
 * 7	0x0080	UART #2 data
 * 8	0x0100	UART #3 data
 * 9	0x0200	Periodic whitelist messages
 * 10	0x0400	Periodic gain control messages
 * 11	0x0800	Not used
 * 12	0x1000	Debug data
 * 13	0x2000	Buffer space messages
 *
 */
LAPIS_API lapis_error_t lapis_set_capture_mask(uint32_t mask);

/**
 * @brief lapis_set_whitelist_trig
 * @param[in] access_address
 * @param[in] on
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_whitelist_trig(uint32_t access_address, uint8_t on);

/**
 * @brief lapis_get_whitelist_trig
 * @param[in] access_address
 * @param[out] on
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_whitelist_trig(uint32_t access_address, uint8_t* on);

/**
 * @brief lapis_set_chan_mask
 * @param[in] mask
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_chan_mask(uint64_t mask);

/**
 * @brief lapis_get_chan_mask
 * @param[in] mask
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_chan_mask(uint64_t *mask);

/**
 * @brief lapis_get_capture_mask
 * @param[out] mask
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_capture_mask(uint32_t* mask);

/**
 * @brief lapis_set_dio
 * @param[in] mask 16 bit mask indicating which IO lines are special function
 * @param[in] state {Mask indicating output state of IO lines which are not special function and
 * which have been configured for output}
 * @param[in] up {Mask indicating whether a digital event should be generated if an up transition
 * is observed on the corresponding IO line}
 * @param[in] dn {Mask indicating whether a digital event should be generated if an dn transition
 * is observed on the corresponding IO line}
 * @param[in] cfg {Determines which IO lines are input and output and whether the IO voltage is
 * internal or external}
 *
 * cgf bits:
 * 0 0x1 If set, lower 8 IO lines are outputs, otherwise they are inputs
 * 1 0x2 If set, upper 8 IO lines are outputs, otherwise they are inputs
 * 2 0x4 If set, internal IO voltage is generated, otherwise an external IO voltage must be supplied
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_dio(uint16_t mask, uint16_t state, uint16_t up, uint16_t dn, uint8_t cfg);

/**
 * @brief lapis_get_dio
 * @param[out] mask 16 bit mask indicating which IO lines are special function
 * @param[out] state {Mask indicating output state of IO lines which are not special function and
 * which have been configured for output}
 * @param[out] up {Mask indicating whether a digital event should be generated if an up transition
 * is observed on the corresponding IO line}
 * @param[out] dn {Mask indicating whether a digital event should be generated if an dn transition
 * is observed on the corresponding IO line}
 * @param[out] cfg {Determines which IO lines are input and output and whether the IO voltage is
 * internal or external}
 *
 * cgf bits:
 * 0 0x1 If set, lower 8 IO lines are outputs, otherwise they are inputs
 * 1 0x2 If set, upper 8 IO lines are outputs, otherwise they are inputs
 * 2 0x4 If set, internal IO voltage is generated, otherwise an external IO voltage must be supplied
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_dio(uint16_t *mask, uint16_t *state, uint16_t *up, uint16_t *dn, uint8_t *cfg);

/**
 * @brief lapis_set_uart
 * @param[in] baud Baud rate
 * @param[in] databits Number of data bits.
 * @param[in] parity 0 = no parity, 1 = odd parity, 2 = even parity
 * @param[in] stopbits Number of stop bits. 0, 1 or 2.
 * @param[in] nUART The number of the UART to be configured. 0 to 3.
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_uart(uint32_t baud , uint8_t databits , uint8_t parity , uint8_t stopbits, uint8_t nUART);

/**
 * @brief lapis_get_uart
 * @param[out] baud Baud rate
 * @param[out] databits Number of data bits.
 * @param[out] parity 0 = no parity, 1 = odd parity, 2 = even parity
 * @param[out] stopbits Number of stop bits. 0, 1 or 2.
 * @param[in] nUART The number of the UART to be configured. 0 to 3.
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_uart(uint32_t *baud , uint8_t *databits , uint8_t *parity , uint8_t *stopbits, uint8_t nUART);

/**
 * @brief lapis_set_timestamping
 * @param[in] Hz External clock frequency in Hz. Set to 0 to use internal clock
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_timestamping(uint32_t Hz);

/**
 * @brief lapis_get_timestamping
 * @param[out] On input contains external clock frequency. On output contains timestamp clock frequency (nominal 4MHz).
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_timestamping(uint32_t *Hz);

/**
 * @brief lapis_set_ext_clock
 * @param[in] flags Determines whether a clock is output and if so, what clock it is
 *
 * flags bits:
 * 0 0x1 If set, enable output clock
 * 1 0x2 If set, output clock is determine by bit 2, otherwise clock is 10MHz
 * 2 0x4 If set, output clock is reference clock used to derive timestamp clock, otherwise is nominal 4MHz
 *
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_set_ext_clock(uint8_t flags);

/**
 * @brief lapis_get_ext_clock
 * @param[out] flags Indicates whether a clock is output and if so, what clock it is
 *
 * flags bits:
 * 0 0x1 If set, enable output clock
 * 1 0x2 If set, output clock is determine by bit 2, otherwise clock is 10MHz
 * 2 0x4 If set, output clock is reference clock used to derive timestamp clock, otherwise is nominal 4MHz
 *
 * @return Error code
 */
LAPIS_API lapis_error_t lapis_get_ext_clock(uint8_t *flags);

#ifdef __cplusplus
}
#endif

#endif // COMMANDS_H
