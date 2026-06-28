#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

namespace {

constexpr int kDefaultListenPort = 2233;

const char* macro_name(int macro_id)
{
    switch (macro_id) {
    case 0: return "weapon";
    case 1: return "material";
    case 2: return "vehicle";
    case 255: return "invalid";
    default: return "unknown";
    }
}

bool parse_macro_packet(const std::string& line, int& macro_id)
{
    std::istringstream iss(line);
    std::string tag;
    int value = 255;

    if (!(iss >> tag >> value)) {
        return false;
    }
    if (tag != "MACRO") {
        return false;
    }
    if (!((value >= 0 && value <= 2) || value == 255)) {
        return false;
    }

    macro_id = value;
    return true;
}

void print_usage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " [listen_port]\n\n"
        << "Default:\n"
        << "  listen_port " << kDefaultListenPort << "\n\n"
        << "Expected packet format:\n"
        << "  MACRO 0\n"
        << "  MACRO 1\n"
        << "  MACRO 2\n"
        << "  MACRO 255\n";
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

    std::string pending;
    char buffer[256];

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

        pending.append(buffer, static_cast<size_t>(n));

        size_t pos = 0;
        while ((pos = pending.find('\n')) != std::string::npos) {
            std::string line = pending.substr(0, pos);
            pending.erase(0, pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            int macro_id = 255;
            if (parse_macro_packet(line, macro_id)) {
                std::cout << "received macro_id=" << macro_id
                          << " name=" << macro_name(macro_id)
                          << std::endl;
            } else {
                std::cout << "invalid packet: " << line << std::endl;
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
