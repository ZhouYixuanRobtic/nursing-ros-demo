#ifndef NURSING_DRIVER_SOCKETCOMMUNICATOR_H
#define NURSING_DRIVER_SOCKETCOMMUNICATOR_H

#include <sys/socket.h>
#include <sys/types.h>

#include <netinet/in.h>
#include <poll.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>

#include <error.h>
#include <cstdio>
#include <cerrno>

#include <cstring>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "NursingMetaType.h"
#include "tr1/memory"
#define _BUFFER_SIZE_ 1024
namespace SocketCommunicator
{
    using std::tr1::shared_ptr;
    class SocketClient{
    private:

        const int _BACKLOG_ =12;
        nursing_namespace::PlanningState *ps_;

        struct sockaddr_in clientAddr_{};
        int clientFd_{};

        int sinSize_{};
        /* 从buffer中读取的位数 */
        int receivedBytes_{};
        /* buffer */
        char buffer_[_BUFFER_SIZE_]{};

        shared_ptr<boost::thread> clientReadThread_;
        bool isReadRegistered;
        unsigned short portNum_=0x8888;

    public:
        bool isConnectionOk_;
        bool isInitialized;
        explicit SocketClient(unsigned short portNum);
        ~SocketClient();
        bool initialize();
        bool waitForConnection();
        void clientReadThread(int rate);
        bool read();
        bool write(const void * data, int dataSize);

        void registerClientReadThread(int rate);
        const nursing_namespace::PlanningState& getPlanningState() const { return *ps_;};
        void start();
        bool beginListen();
        void closeClient();
    };

    class SocketServer{
    private:

        const int _BACKLOG_ =12;
        /* 在sockFd上进行监听，newFd 接受新的连接 */
        int serverFd_{}, newFd_{},tempFd_{};
        /* 自己的地址信息 */
        struct sockaddr_in serverAddr_{}, receiveAddr_{};
        fd_set  server_set{},serverNew_set{};
        int sinSize_{};
        /* 从buffer中读取的位数 */
        int receivedBytes_{};
        /* buffer */
        char * buffer_;
        char * write_buffer_;
        boost::mutex read_mutex_{},write_mutex_{};

        nursing_namespace::PlanningState * ps_;

        bool isReadRegistered,isWriteRegistered;
        shared_ptr<boost::thread> ServerReadThread_;
        shared_ptr<boost::thread> ServerWriteThread_;
        unsigned short portNum_=0x8888;

    public:
        bool isConnectionOk;
        bool isInitialized;
        bool isReceivedCommand;
        explicit SocketServer(unsigned short portNum);
        ~SocketServer();
        bool initialize();
        bool waitForConnection();
        bool write(const void *data,int dataSize);
        bool read();
        void serverReadThread(int rate);
        void serverWriteThread(nursing_namespace::PlanningState * ps_ptr,int rate);
        void registerServerReadThread(int rate);
        void registerServerWriteThread(nursing_namespace::PlanningState * ps_ptr,int rate);
        void closeServer();
        const nursing_namespace::PlanningState& getPlanningState() const { return *ps_;};
    };
}




#endif //NURSING_DRIVER_SOCKETCOMMUNICATOR_H
