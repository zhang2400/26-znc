#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>

namespace {

constexpr int kDefaultListenPort = 2233;
constexpr size_t kPacketSize = 12;
constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;

const char* macro_name(int macro_id)
{
    switch (macro_id) {
    case 0: return "weapon";
    case 1: return "material";
    case 2: return "vehicle";
    case -1: return "unknown";
    default: return "unknown";
    }
}

uint8_t checksum(const uint8_t* data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    return sum;
}

int16_t read_i16_le(const uint8_t* src)
{
    const uint16_t value = static_cast<uint16_t>(src[0])
        | (static_cast<uint16_t>(src[1]) << 8);
    return static_cast<int16_t>(value);
}

struct TargetPacket {
    uint8_t seq = 0;
    int class_id = -1;
    uint8_t confidence = 0;
    int16_t target_x = -1;
    int16_t target_y = -1;
    uint16_t timestamp_ms = 0;
};

bool parse_target_packet(const uint8_t* packet, TargetPacket& out)
{
    if (packet[0] != kHeader0 || packet[1] != kHeader1) {
        return false;
    }
    if (checksum(packet, kPacketSize - 1) != packet[kPacketSize - 1]) {
        return false;
    }

    out.seq = packet[2];
    out.class_id = static_cast<int8_t>(packet[3]);
    out.confidence = packet[4];
    out.target_x = read_i16_le(packet + 5);
    out.target_y = read_i16_le(packet + 7);
    out.timestamp_ms = static_cast<uint16_t>(packet[9])
        | (static_cast<uint16_t>(packet[10]) << 8);
    return true;
}

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [listen_port]\n\n"
        << "Default:\n"
        << "  listen_port " << kDefaultListenPort << "\n\n"
        << "Expected binary packet, 12 bytes:\n"
        << "  AA 55 seq class confidence x_lo x_hi y_lo y_hi time_lo time_hi checksum\n\n"
        << "class:\n"
        << "  -1 unknown, 0 weapon, 1 material, 2 vehicle\n";
}

int create_server_socket(int port)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << std::endl;
        return -1;
    }

    int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind failed: " << std::strerror(errno) << std::endl;
        close(server_fd);
        return -1;
    }

    if (listen(server_fd, 4) < 0) {
        std::cerr << "listen failed: " << std::strerror(errno) << std::endl;
        close(server_fd);
        return -1;
    }

    return server_fd;
}

void handle_client(int client_fd, const sockaddr_in& client_addr)
{
    int nodelay = 1;
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
    std::cout << "client connected: " << ip << ":" << ntohs(client_addr.sin_port) << std::endl;

    std::vector<uint8_t> pending;
    uint8_t buffer[256];

    while (true) {
        ssize_t n = recv(client_fd, buffer, sizeof(buffer), 0);
        if (n == 0) {
            std::cout << "client disconnected" << std::endl;
            break;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "recv failed: " << std::strerror(errno) << std::endl;
            break;
        }

        pending.insert(pending.end(), buffer, buffer + n);

        while (pending.size() >= kPacketSize) {
            if (pending[0] != kHeader0 || pending[1] != kHeader1) {
                auto next = std::find(pending.begin() + 1, pending.end(), kHeader0);
                pending.erase(pending.begin(), next);
                continue;
            }

            TargetPacket packet;
            if (parse_target_packet(pending.data(), packet)) {
                std::cout << "received seq=" << static_cast<int>(packet.seq)
                          << " class=" << packet.class_id
                          << " name=" << macro_name(packet.class_id)
                          << " confidence=" << static_cast<int>(packet.confidence) << "%"
                          << " target=(" << packet.target_x << "," << packet.target_y << ")"
                          << " time=" << packet.timestamp_ms
                          << std::endl;
                pending.erase(pending.begin(), pending.begin() + kPacketSize);
            } else {
                std::cout << "invalid checksum/header packet:";
                for (size_t i = 0; i < kPacketSize; ++i) {
                    std::cout << " " << std::hex << std::uppercase << std::setw(2)
                              << std::setfill('0') << static_cast<int>(pending[i]);
                }
                std::cout << std::dec << std::setfill(' ') << std::endl;
                pending.erase(pending.begin());
            }
        }
    }

    close(client_fd);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        print_usage(argv[0]);
        return 0;
    }

    const int listen_port = argc > 1 ? std::atoi(argv[1]) : kDefaultListenPort;
    int server_fd = create_server_socket(listen_port);
    if (server_fd < 0) {
        return 1;
    }

    std::cout << "listening on 0.0.0.0:" << listen_port << std::endl;

    while (true) {
        sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "accept failed: " << std::strerror(errno) << std::endl;
            continue;
        }

        handle_client(client_fd, client_addr);
        std::cout << "waiting for next client..." << std::endl;
    }

    close(server_fd);
    return 0;
}
