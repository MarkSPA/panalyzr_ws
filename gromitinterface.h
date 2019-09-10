#ifndef GROMITINTERFACE_H
#define GROMITINTERFACE_H
#include <string>
#include "bitfield.h"
#include "serialiser.h"
#include "morephtransport.h"

class GromitInterface {
public:

    struct Features {
        uint32_t app;
        uint32_t fpga_ver;
        uint32_t fpga_date;
        uint32_t fpga_time;
        uint32_t fpga_feat;
        std::string linux;
        std::string moreph;

        template <typename It, typename... Args>
        void deserialise(It& begin, It& end, Args&&... args) {
            Deserialiser::deserialise(begin, end, app, fpga_ver, fpga_date, fpga_time, fpga_feat, linux, ": ", moreph,
                                      std::forward<Args>(args)...);
        }

        const char* model() const {
            switch (EXTRACT(fpga_ver, 0, 8)) {
            case 97:
                return "Moreph20";
            case 98:
                return "Moreph30";
            case 99:
                return "Moreph30WB";
            default:
                return "Unknown";
            }
        }
    };

    struct Network {
        bool dhcp;
        uint32_t ip;
        uint32_t subnet;
        uint32_t gate;
        uint32_t cur_ip;
        uint32_t cur_subnet;
        uint32_t cur_gate;

        template <typename It, typename... Args>
        void deserialise(It& begin, It& end, Args&&... args) {
            uint32_t dhcp_val;
            Deserialiser::deserialise(begin, end, dhcp_val, ip, subnet, gate, cur_ip, cur_subnet, cur_gate, std::forward<Args>(args)...);
            dhcp = dhcp_val ? true : false;
        }
    };

    struct Env {
        uint32_t hdr;
        uint32_t time_msb;
        uint32_t time_lsb;
        uint16_t fanspeed;
        uint16_t flags;
        uint16_t stats[3 * 7];
        uint16_t err;
        uint32_t frame;

        template <typename It, typename... Args>
        void deserialise(It& begin, It& end, Args&&... args) {
            Deserialiser::deserialise(begin, end, time_msb, time_lsb, fanspeed, flags, stats, err, frame, std::forward<Args>(args)...);
        }
    };

    GromitInterface(MorephTransport morephInterface);
    virtual ~GromitInterface() {}

    MorephTransport getTransport();
    bool dead() { return m.dead(); }

    void download(std::vector<uint8_t> data);
    void executeFileInRam();
    void hardwareReset();
    void exitApp();
    std::string getFriendlyName();
    void getFriendlyNameAsync(std::function<void(std::string)> callback);
    void setFriendlyName(std::string name);
    uint32_t getSerialNumber();
    void getSerialNumberAsync(std::function<void(uint32_t)> callback = nullptr);
    void setTime();
    std::string getManufacturingString();
    void getManufacturingStringAsync(std::function<void(std::string)> callback = nullptr);
    Features getFeatures();
    void getFeaturesAsync(std::function<void(Features)> callback = nullptr);
    uint32_t getNumLicence();
    std::vector<unsigned char> getLicence(uint32_t n);
    void addLicence(std::vector<unsigned char> lic);
    void getLicenceAsync(uint32_t n, std::function<void(std::array<uint8_t, 72>)> callback = nullptr);
    void delLicence(uint32_t n);
    void delLicenceAsync(uint32_t n, std::function<void()> callback = nullptr);
    void clearLicence();
    void clearLicenceAsync(std::function<void()> callback = nullptr);
    Env getEnv();
    std::vector<uint8_t> readFlash(uint32_t part, uint32_t offset, uint32_t num);
    void writeFlash(std::vector<uint8_t> data);
    void powerDown();

    //Moreph LP stuff
    std::vector<uint8_t> jtag_shift(unsigned count, const std::vector<uint8_t>& tms_tdi);
    void load_fpga(std::vector<uint8_t> sig);
    uint32_t reg_read(uint32_t addr);
    void reg_write(uint32_t addr, uint32_t value);
    void dll(uint32_t v);

protected:
    MorephTransport m;
};

#endif  // MOREPH_H
