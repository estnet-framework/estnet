//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics - 
// Julius-Maximilians-Universitaet Wuerzburg
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __UTILS_SOCKET_UTILS_H__
#define __UTILS_SOCKET_UTILS_H__

#include <stdexcept>
#include <sys/types.h>
#ifdef __linux
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#else
#include<winsock2.h>
#include <ws2tcpip.h>
#endif
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>

namespace estnet {

/** @brief opens a TCP client socket */
int open_tcp_client_socket(const char *host, unsigned short port) {
    int sockfd = -1;
    // make port a string
    char port_str[6];
    memset(port_str, '\0', 6);
    snprintf(port_str, 6, "%d", port);
    port_str[5] = '\0';
    // get sock_addr
    struct addrinfo hints, *infoptrs, *infoptr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC; // don't care if it's Ipv4 or Ipv6
    hints.ai_socktype = SOCK_STREAM; // we want a stream not datagram socket
    hints.ai_flags = 0; // no flags
    hints.ai_protocol = IPPROTO_TCP; // we want a tcp socket
    // null as the first argument and no passive flag means, we'll get
    // the best loopback address back
    int rtn = getaddrinfo(host, port_str, &hints, &infoptrs);
    if (rtn != 0) {
        fprintf(stderr, "ERROR getaddrinfo %d\n", rtn);
        return rtn;
    }
    for (infoptr = infoptrs; infoptr != NULL; infoptr = infoptr->ai_next) {
        sockfd = socket(infoptr->ai_family, infoptr->ai_socktype,
                infoptr->ai_protocol);
        if (sockfd < 0) {
            fprintf(stderr, "ERROR opening socket %d\n", sockfd);
            return sockfd;
        }
#ifdef __linux
    int so_reuseaddr_val = 1;
    rtn = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &so_reuseaddr_val, sizeof(int));
    if (rtn < 0) {
      fprintf(stderr, "setsockopt(SO_REUSEADDR) failed %d\n", rtn);
      return rtn;
    }
    rtn = fcntl(sockfd, F_SETFD, FD_CLOEXEC);
    if (rtn < 0) {
      fprintf(stderr, "fcntl(F_SETFD, FD_CLOEXEC) failed %d\n", rtn);
      return rtn;
    }
#endif
        rtn = connect(sockfd, infoptr->ai_addr, infoptr->ai_addrlen);
        if (rtn == 0) {
            // successfully connected
            // get the human readable version of the address we're connecting to
            char hbuf[NI_MAXHOST];
            rtn = getnameinfo(infoptr->ai_addr, infoptr->ai_addrlen, hbuf,
                    sizeof(hbuf), NULL, 0, NI_NAMEREQD);
            if (rtn != 0) {
                fprintf(stderr, "ERROR getnameinfo %d\n", rtn);
                return rtn;
            }
            fprintf(stdout, "connected to %s:%s\n", hbuf, port_str);
            // stop iterating our options, since we successfully connected
            break;
        }
        fprintf(stderr, "ERROR socket connect %d/%d, trying next option\n", rtn,
        errno);
        // close socket if no successful connection
        close(sockfd);
    }
    if (infoptr == NULL) {
        freeaddrinfo(infoptrs);
        fprintf(stderr, "ERROR connecting socket\n");
        return -42;
    }
    freeaddrinfo(infoptrs);
    return sockfd;
}

/** @brief sends given bytes over the given socket  */
int sendBytes(int sockfd, size_t numBytes, const char *bytes) {
    size_t sentBytes = 0;
    while (sentBytes < numBytes) {
        ssize_t sent = send(sockfd, bytes + sentBytes, numBytes - sentBytes, 0);
        if (sent <= 0) {
            throw std::runtime_error(
                    "Could not sent command string to compass");
        }
        sentBytes += sent;
    }
    return (sentBytes == numBytes) ? 0 : -1;
}

/** @brief receives numBytes from the given socket, memory must already be allocated */
int recvBytes(int sockfd, size_t numBytes, char *bytes) {
    size_t recvdBytes = 0;
    while (recvdBytes < numBytes) {
        ssize_t recvd = recv(sockfd, bytes + recvdBytes, numBytes - recvdBytes,
                0);
        if (recvd <= 0) {
            throw std::runtime_error(
                    "Could not receive command string from compass");
        }
        recvdBytes += recvd;
    }
    return (recvdBytes == numBytes) ? 0 : -1;
}

/** @brief checks if the given socket has data to receive */
int checkIfDataAvailable(int sockfd) {
    fd_set sockset;
    FD_ZERO(&sockset);
    FD_SET(sockfd, &sockset);
    struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
    int result = select(sockfd + 1, &sockset, NULL, NULL, &timeout);
    if (result < 0) {
        throw std::runtime_error("ERROR selecting socket");
    } else if (result == 1) {
        if (FD_ISSET(sockfd, &sockset)) {
            return 1;
        }
    }
    return 0;
}

/** @brief closes the given socket */
void close_socket(int sockfd) {
    if (sockfd > 0) {
        close(sockfd);
    }
}

}  // namespace estnet

#endif
