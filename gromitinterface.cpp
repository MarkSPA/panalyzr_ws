#include "gromitinterface.h"
#include "CRC32.h"
#include "morephtransport.h"
#include <ctime>

enum UsbCommand : uint8_t {
    USB_CMD_CAPTURE_MASK = 0,
    USB_CMD_WHITELIST_UPDATE = 1,
    USB_CMD_FREQUENCY = 1,
    USB_CMD_WHITELIST_UPDATE_MASK = 2,
    USB_CMD_DECIMATION = 2,
    USB_CMD_FM_WAVEFORM = 3,
    USB_CMD_RBW = 3,
    USB_CMD_CORRELATOR_BIT_ERRORS = 4,
    USB_CMD_RBW_FILTER = 4,
    USB_CMD_SHORT_PACKETS = 5,
    USB_CMD_TRACE = 5,
    USB_CMD_SEARCH_ENGINE_PARAMETERS = 6,
    USB_CMD_AVERAGE = 6,
    USB_CMD_DEWHITENING = 7,
    USB_CMD_ZEROSPAN = 7,
    USB_CMD_GAIN = 8,
    USB_CMD_CALIBRATION = 9,
    USB_CMD_DIO_TIMESTAMP = 10,
    USB_CMD_DIO_CHANNELS = 11,
    USB_CMD_DIO_ACCESS_ADDRESS = 12,
    USB_CMD_DIG_EVENTS = 13,
    USB_CMD_UART = 14,
    USB_CMD_TIME = 15,
    USB_CMD_GAIN_PERIOD = 16,
    USB_CMD_SPECTRUM_PERIOD = 17,
    USB_CMD_WHITELIST_PERIOD = 18,
    USB_CMD_ENVIORNMENTAL_PERIOD = 19,
    USB_CMD_AUTO_FM = 20,
    USB_CMD_WHITELIST_ALGORITHM = 21,
    USB_CMD_TRACE_OFFSET = 27,
    USB_CMD_FFT_LENGTH = 28,
    USB_CMD_TRACE_LENGTH = 29,
    USB_CMD_TRIGGER = 30,
    USB_CMD_FEATURES = 64,
    USB_CMD_ENV = 66,
    USB_CMD_READ_FLASH = 95,
    USB_CMD_START_FILE_DOWNLOAD = 96,
    USB_CMD_FILE_DOWNLOAD = 97,
    USB_CMD_ABORT_FILE_DOWNLOAD = 98,
    USB_CMD_MOVE_FILE_TO_RAM = 99,
    USB_CMD_UNLOCK_FLASH_PARTITION = 100,
    USB_CMD_MOVE_FILE_TO_FLASH = 101,
    USB_CMD_EXECUTE_FILE_IN_RAM = 102,
    USB_CMD_FRIENDLY_NAME = 103,
    USB_CMD_SERIAL_NUMBER = 104,
    USB_CMD_NUM_LICENCE = 105,
    USB_CMD_GET_LICENCE = 106,
    USB_CMD_ADD_LICENCE = 107,
    USB_CMD_DEL_LICENCE = 108,
    USB_CMD_CLR_LICENCE = 109,
    USB_CMD_MANUFACTURER_STRING = 110,
    USB_CMD_ADCPHASE = 114,
    USB_CMD_CHECKPWRLED = 115,
    USB_CMD_CHECKOVRLED = 116,
    USB_CMD_CHECK_DIO = 118,
    USB_CMD_CAPTURE_ENABLE = 120,
    USB_CMD_CAPTURE_DISABLE = 121,
    USB_CMD_CAL_CAPTURE = 122,
    USB_CMD_HOSTNAME = 122,
    USB_CMD_MACADDR = 123,
    USB_CMD_NETWORK = 124,
    USB_CMD_POWER = 125,
    USB_CMD_EXIT = 254,
    USB_CMD_RESET = 255,

    // Moreph LP
    USB_CMD_JTAG_SHIFT = 90,
    USB_CMD_LOAD_FPGA = 1,
    USB_CMD_REG_READ = 2,
    USB_CMD_REG_WRITE = 3,
    USB_CMD_TEST = 4,
    USB_CMD_EFUSE = 91,

    USB_CMD_GET = 128,
    USB_BAD_RESP = 127,
};

GromitInterface::GromitInterface(MorephTransport GromitInterface) : m(std::move(GromitInterface)) {}

MorephTransport GromitInterface::getTransport() { return m; }

void GromitInterface::download(std::vector<uint8_t> data) {
    uint32_t crc = crc32(0, NULL, 0);
    crc = crc32(crc, data.data(), data.size());

    m.command(USB_CMD_START_FILE_DOWNLOAD, uint32_t(data.size()), crc);

    auto first = data.begin();
    while (first != data.end()) {
        auto last = data.end() - first > 2048 ? first + 2048 : data.end();
        m.command(USB_CMD_FILE_DOWNLOAD, first, last);

        first = last;
    }

    m.command(USB_CMD_MOVE_FILE_TO_RAM);
}

void GromitInterface::writeFlash(std::vector<uint8_t> data) {
    uint8_t partition = 0; // Only allow firmware updates through Moreph (others in Mould)
    uint32_t crc = crc32(0, NULL, 0);
    crc = crc32(crc, data.data(), data.size());

    m.command(USB_CMD_START_FILE_DOWNLOAD, uint32_t(data.size()), crc);

    auto first = data.begin();
    while (first != data.end()) {
        auto last = data.end() - first > 2048 ? first + 2048 : data.end();
        m.command(USB_CMD_FILE_DOWNLOAD, first, last);

        first = last;
    }

    m.command(USB_CMD_UNLOCK_FLASH_PARTITION, partition);

    m.command(USB_CMD_MOVE_FILE_TO_FLASH, partition);
}

void GromitInterface::executeFileInRam() { m.command(USB_CMD_EXECUTE_FILE_IN_RAM); }

void GromitInterface::hardwareReset() { m.command(USB_CMD_RESET); }

void GromitInterface::exitApp() { m.command(USB_CMD_EXIT); }

std::string GromitInterface::getFriendlyName() {
    std::string name;
    m.command(USB_CMD_FRIENDLY_NAME | USB_CMD_GET)(name);
    return name;
}

void GromitInterface::getFriendlyNameAsync(std::function<void(std::string)> callback) {
    m.asyncCommand(callback, USB_CMD_FRIENDLY_NAME | USB_CMD_GET);
    return;
}

void GromitInterface::setFriendlyName(std::string name) {
    if (name.size() > 255)
        name.resize(255);
    m.command(USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(5));

    m.command(USB_CMD_FRIENDLY_NAME, name, '\0');
}

uint32_t GromitInterface::getSerialNumber() {
    uint32_t serial;
    m.command(USB_CMD_SERIAL_NUMBER | USB_CMD_GET)(serial);
    return serial;
}

void GromitInterface::getSerialNumberAsync(std::function<void(uint32_t)> callback) {
    m.asyncCommand(callback, USB_CMD_SERIAL_NUMBER | USB_CMD_GET);
    return;
}

void GromitInterface::setTime() {
    uint32_t t = uint32_t(time(nullptr));
    m.command(USB_CMD_TIME, t);
}

std::string GromitInterface::getManufacturingString() {
    std::string mfg;
    m.command(USB_CMD_MANUFACTURER_STRING | USB_CMD_GET)(mfg);
    return mfg;
}

void GromitInterface::getManufacturingStringAsync(std::function<void(std::string)> callback) {
    m.asyncCommand(callback, USB_CMD_MANUFACTURER_STRING | USB_CMD_GET);
    return;
}

GromitInterface::Features GromitInterface::getFeatures() {
    Features f;
    m.command(USB_CMD_FEATURES | USB_CMD_GET)(f);
    return f;
}

void GromitInterface::getFeaturesAsync(std::function<void(Features)> callback) {
    m.asyncCommand(callback, USB_CMD_FEATURES | USB_CMD_GET);
    return;
}

uint32_t GromitInterface::getNumLicence() {
    uint32_t n;
    m.command(USB_CMD_NUM_LICENCE | USB_CMD_GET)(n);
    return n;
}

std::vector<unsigned char> GromitInterface::getLicence(uint32_t n) {
    std::vector<unsigned char> v(72);
    m.command(USB_CMD_GET_LICENCE | USB_CMD_GET, n)(v);
    return v;
}

void GromitInterface::addLicence(std::vector<unsigned char> lic) {
    m.command(USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(4));

    m.command(USB_CMD_ADD_LICENCE, lic);
}

void GromitInterface::getLicenceAsync(uint32_t n,
                                      std::function<void(std::array<uint8_t, 72>)> callback) {
    m.asyncCommand(callback, USB_CMD_GET_LICENCE | USB_CMD_GET, n);
    return;
}

void GromitInterface::delLicence(uint32_t n) {
    m.command(USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(4));

    m.command(USB_CMD_DEL_LICENCE, n);
}

void GromitInterface::delLicenceAsync(uint32_t n, std::function<void()> callback) {
    m.asyncCommand(std::function<void()>(), USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(4));
    m.asyncCommand(callback, USB_CMD_DEL_LICENCE, n);
}

void GromitInterface::clearLicence() {
    m.command(USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(4));

    m.command(USB_CMD_CLR_LICENCE);
}

void GromitInterface::clearLicenceAsync(std::function<void()> callback) {
    m.asyncCommand(std::function<void()>(), USB_CMD_UNLOCK_FLASH_PARTITION, uint8_t(4));
    m.asyncCommand(callback, USB_CMD_CLR_LICENCE);
}

GromitInterface::Env GromitInterface::getEnv() {
    Env env;
    m.command(USB_CMD_ENV | USB_CMD_GET)(env);
    return env;
}

std::vector<uint8_t> GromitInterface::readFlash(uint32_t part, uint32_t offset, uint32_t num) {
    std::vector<uint8_t> v(num);
    m.command(USB_CMD_READ_FLASH | USB_CMD_GET, part, offset, num)(v);
    return v;
}

void GromitInterface::powerDown() {
    try {
        m.command(USB_CMD_POWER | USB_CMD_GET);
    } catch (ExceptionFormat) {
        /* Ignore */
    }
}

std::vector<uint8_t> GromitInterface::jtag_shift(unsigned count,
                                                 const std::vector<uint8_t>& tms_tdi) {
    std::vector<uint8_t> v(tms_tdi.size() / 2);
    m.command(USB_CMD_JTAG_SHIFT, count, tms_tdi)(v);
    return v;
}

void GromitInterface::load_fpga(std::vector<uint8_t> sig) { m.command(USB_CMD_LOAD_FPGA, sig); }

uint32_t GromitInterface::reg_read(uint32_t addr) {
    uint32_t value;
    m.command(USB_CMD_REG_READ, addr)(value);
    return value;
}

void GromitInterface::reg_write(uint32_t addr, uint32_t value) {
    m.command(USB_CMD_REG_WRITE, addr, value);
}

void GromitInterface::dll(uint32_t v) {
    m.command(USB_CMD_TEST, v);

    return;
}
