#include <fmt/format.h>
#include <thread>
#include <iostream>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#include "lapis.h"

using namespace std::chrono_literals;

// Callback to send captured packets to the host

static void data_callback(uint8_t* x, uint32_t n, lapis_error_t err) {
    if (err) {
        fmt::print_colored(fmt::RED, "ERROR: {}\n", lapis_error_msg());
    } else {
        uint32_t* pkt = (uint32_t*)x;
        uint32_t hdr = pkt[0];
        uint32_t pktLen = hdr & 0xFFFF;
        uint32_t pktTyp = hdr >> 24;
        uint32_t reason = pkt[3] & 0xFF;
        uint32_t aa = pkt[9] >> 16;
        aa |= pkt[10] << 16;
        switch (pktTyp) {
        case BLEpkt:
            if (pkt[5] == 0x8E89BED6) {
                uint32_t hdr = pkt[6] & 0xFFFF;
                uint32_t PDU = hdr & 0x000F;
                uint32_t len = hdr >> 8;
                if ((PDU == 5) && (len == 34))
                    fmt::print_colored(fmt::CYAN, "    Connection request {:04X} {:08X}\n", hdr,
                                       aa);
            }
            break;
        case SPECTRUMpkt:
            fmt::print_colored(fmt::GREEN, "Spectrum\n");
            break;
        case WHITELISTpkt:
            if ((reason == 0x00) || (reason == 0x03) || (reason == 0x07))
                fmt::print_colored(fmt::YELLOW, "    Whitelist update {:02X} {:08X} {:08X}\n",
                                   reason, pkt[3], pkt[4]);
            break;
        case FMpkt:
            fmt::print_colored(fmt::GREEN, "FM data\n");
            break;
        case GAINpkt:
            fmt::print_colored(fmt::GREEN, "Gain control\n");
            break;
        case WHITELISTENTRIESpkt:
            fmt::print_colored(fmt::GREEN, "Whitelist\n");
            break;
        case DIGITALpkt:
            fmt::print_colored(fmt::GREEN, "Digital input\n");
            break;
        case UART0pkt:
            fmt::print_colored(fmt::GREEN, "UART0\n");
            break;
        case UART1pkt:
            fmt::print_colored(fmt::GREEN, "UART1\n");
            break;
        case UART2pkt:
            fmt::print_colored(fmt::GREEN, "UART2\n");
            break;
        case UART3pkt:
            fmt::print_colored(fmt::GREEN, "UART3\n");
            break;
        case BUFFSTATSpkt:
            fmt::print_colored(fmt::GREEN, "Buffer Stats\n");
            break;
        case SPURIOUS:
            fmt::print_colored(fmt::GREEN, "Debug\n");
            break;
        default:
            fmt::print_colored(fmt::GREEN, "Unknown packet type {:08X}\n", hdr);
        }
    }
}

int main(int argc, char* argv []) {
#ifdef _WIN32
    { // Enable colour text output on windows console
        HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD mode;
        GetConsoleMode(h, &mode);
        mode |= 0x4; // ENABLE_VIRTUAL_TERMINAL_PROCESSING;
        SetConsoleMode(h, mode);
    }
#endif

    // Find a MorephLP and connect to it
    // This simply connects to the first MorephLP discovered

    lapis_search_result_t* results;

    while (true) {
        int N;
        lapis_search(&results, &N);
        for (int i = 0; i < N; i++) {
            fmt::print_colored(fmt::CYAN, "Found: {}\n", results[i].serial_number);
        }
        if (N) {
            fmt::print_colored(fmt::CYAN, "Connecting\n");
            if (lapis_connect(results[0].handle)) {
                fmt::print_colored(fmt::RED, "Failed to connect: {}\n", lapis_error_msg());
                return -1;
            }
            break;
        }
        std::this_thread::sleep_for(500ms);
    }

    // Set the capture mask to determine which packets are sent to the host

    lapis_set_capture_mask(0xFFF);

    // Register the callback to pass captured packets to the host

    lapis_set_data_callback(data_callback);

    fmt::print("\n\nPress Enter key to exit...\n\n");
    std::cin.get();

    lapis_disconnect();

    return 0;
}
