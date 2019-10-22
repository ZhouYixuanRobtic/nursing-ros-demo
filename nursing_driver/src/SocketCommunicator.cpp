#include <sensor_msgs/JointState.h>
#include "SocketCommunicator.h"
namespace SocketCommunicator
{
    SocketClient::SocketClient(unsigned short portNum, std::string server_address):SERVER_ADDRESS_(server_address)
    {
        this->portNum_=portNum;
        this->isConnectionOk_=false;
        this->isReadRegistered=false;
        this->isInitialized=false;
        this->isReadRegistered=false;
        this->receivedBool_=false;
        ps_= new nursing_namespace::PlanningState;
        memset(buffer_, 0, _BUFFER_SIZE_);
    }
    SocketClient::~SocketClient()
    {
        if(isReadRegistered)
        {
            clientReadThread_->interrupt();
            clientReadThread_->join();
        }
        close(clientFd_);
        delete ps_;
    }
    bool SocketClient::initialize()
    {
        /* 如果调用 socket() 出错，则退出 */
        if((clientFd_ = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            /* 输出错误提示并退出 */
            perror(" socket fail !");
            return false;
        }

        bzero(&clientAddr_,sizeof(struct sockaddr_in));
        /* 主机字节顺序 */
        clientAddr_.sin_family = AF_INET;
        /* 网络字节顺序，短整型 */
        clientAddr_.sin_port = htons(portNum_);    //9054
        /* 将运行程序机器的IP填充入s_addr */
        clientAddr_.sin_addr.s_addr = inet_addr(SERVER_ADDRESS_.c_str());  //  INADDR_ANY  0.0.0.0 monitor all
        isInitialized=true;
        return true;
    }
    bool SocketClient::waitForConnection()
    {
        if(-1 == connect(clientFd_,(struct sockaddr *)(&clientAddr_), sizeof(struct sockaddr)))
        {
            perror("client connect fail!!");
            isConnectionOk_=false;
        }
        else
        {
            isConnectionOk_=true;
        }
        return isConnectionOk_;

    }
    bool SocketClient::write(const void * data, int dataSize)
    {
        if(isConnectionOk_)
        {
            if(send(clientFd_, data, dataSize, 0) == -1)
            {
                isConnectionOk_=false;
                close(clientFd_);
                printf("write fail!\r\n");
                if(isReadRegistered)
                {
                    clientReadThread_->interrupt();
                    isReadRegistered=false;
                }
            }
        }
        return isConnectionOk_;
    }
    bool SocketClient::read()
    {
        if(-1==(receivedBytes_ = recv(clientFd_,buffer_,_BUFFER_SIZE_,0)))
        {
            printf("read fail!!!");
            return false;
        }
        else
        {
            if(receivedBytes_ == 0)
            {
                printf("wrong\r\n");
                isConnectionOk_=false;
                close(clientFd_);
            }
            else if(receivedBytes_<0)
            {
                if(!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
                {
                    printf("wrong\r\n");
                    isConnectionOk_=false;
                    close(clientFd_);
                }
            }
            else
            {
                if(receivedBytes_> sizeof(bool)+1)
                    memcpy(ps_,buffer_, sizeof(nursing_namespace::PlanningState));
                else
                {
                    receivedBool_ = static_cast<bool> (buffer_[receivedBytes_ - 1]);
                    std::cout << "the received bool is" << receivedBool_ << std::endl;
                }
            }
        }
        return true;
    }
    void SocketClient::closeClient()
    {
        close(clientFd_);
    }
    void SocketClient::clientReadThread(int rate)
    {
        boost::this_thread::interruption_enabled();
        try
        {
            while(isConnectionOk_)
            {
                boost::this_thread::interruption_point();
                read();
                boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
            }
        }
        catch (boost::thread_interrupted&e )
        {
            std::cout<<"client thread interrupted!"<<std::endl;
        }
    }
    void SocketClient::registerClientReadThread(int rate)
    {
        clientReadThread_.reset(new boost::thread(boost::bind(&SocketClient::clientReadThread, this, rate)));
        isReadRegistered=true;
    }
    void SocketClient::start()
    {
        while(!isInitialized)
        {
            initialize();
            usleep(50000);
        }
    }
    bool SocketClient::beginListen()
    {
        if(waitForConnection())
        {
            registerClientReadThread(50);
            return true;
        }
        return false;
    }
    SocketServer::SocketServer(unsigned short portNum)
    {
        this->portNum_=portNum;
        this->isConnectionOk=false;
        this->isReadRegistered=false;
        this->isInitialized=false;
        buffer_=new char[_BUFFER_SIZE_];
        write_buffer_=new char[_BUFFER_SIZE_];
        ps_=new nursing_namespace::PlanningState;
        memset(write_buffer_,0,_BUFFER_SIZE_);
    }
    SocketServer::~SocketServer()
    {
        if(isReadRegistered)
        {
            ServerReadThread_->interrupt();
            ServerReadThread_->join();
        }
        if(isWriteRegistered)
        {
            ServerWriteThread_->interrupt();
            ServerWriteThread_->join();
        }
        close(newFd_);
        delete [] buffer_;
        delete[] write_buffer_;
        delete ps_;
    }
    bool SocketServer::initialize()
    {
        /* 如果调用 socket() 出错，则退出 */
        if((serverFd_ = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            /* 输出错误提示并退出 */
            printf(" socket fail !\r\n");
            return false;
        }
        printf("socket ok !\r\n");

        /* 主机字节顺序 */
        serverAddr_.sin_family = AF_INET;
        /* 网络字节顺序，短整型 */
        serverAddr_.sin_port = htons(portNum_);    //9054
        /* 将运行程序机器的IP填充入s_addr */
        serverAddr_.sin_addr.s_addr = INADDR_ANY;  //  INADDR_ANY  0.0.0.0 monitor all
        /* 将此结构的其余空间清零 */
        bzero(& (serverAddr_.sin_zero), 8);
        /* 这里是我们一直强调的错误检查！！ */
        int iSockOptVal = 1;
        /* 检测是否端口被占用 */
        if(setsockopt(serverFd_, SOL_SOCKET, SO_REUSEADDR, &iSockOptVal, sizeof(iSockOptVal)) == -1)
        {
            printf("set sock fail\r\n");
            close(serverFd_);
            return false;
        }
        if(bind(serverFd_, (struct sockaddr*) &serverAddr_, sizeof(struct sockaddr)) == -1)
        {
            /* 如果调用bind()失败，则给出错误提示，退出 */
            printf(" bind fail!\r\n ");
            return false;
        }
        printf("bind ok !\r\n");
        /* 这里是我们一直强调的错误检查！！ */
        if(listen(serverFd_, _BACKLOG_) == -1)
        {
            /* 如果调用 listen 失败，则给出错误提示，退出 */
            printf("listen fail!\r\n");
            return false;
        }
        printf("listen ok!\r\n");
        FD_ZERO(&server_set);
        FD_ZERO(&serverNew_set);
        FD_SET(serverFd_, &serverNew_set);
        tempFd_=serverFd_;
        isInitialized=true;
        return true;
    }
    bool SocketServer::waitForConnection()
    {
        server_set=serverNew_set;
        int nready = select(tempFd_ + 1, &server_set, nullptr, nullptr, nullptr);
        if(nready<=0)
        {
            isConnectionOk=false;
            return false;
        }
        else if(FD_ISSET(serverFd_,&server_set))
        {
            newFd_ = accept(serverFd_, (struct sockaddr*) &receiveAddr_, (socklen_t*) &sinSize_);
            /* 若accept返回为-1，则连接错误，重新等待连接 */
            if(newFd_ == -1)
            {
                printf(" server accept fail!!!\r\n ");
                isConnectionOk=false;
            }
            else
            {
                FD_SET(newFd_, &serverNew_set);
                if(newFd_ > tempFd_)
                    tempFd_=newFd_;
                char* ip = inet_ntoa(receiveAddr_.sin_addr);
                printf("Client@%s connected\n", ip);
                isConnectionOk=true;
                fcntl(newFd_, F_SETFL, O_NONBLOCK);
                close(serverFd_);
            }
            return  isConnectionOk;
        }
    }
    bool SocketServer::write(const void *data, int dataSize)
    {
        if(isConnectionOk)
        {
            if(send(newFd_, data, dataSize, 0) == -1)
            {
                isConnectionOk=false;
                close(newFd_);
                isInitialized=false;
                printf("write fail!\r\n");
                if(isReadRegistered)
                {
                    ServerReadThread_->interrupt();
                    isReadRegistered=false;
                }
                if(isWriteRegistered)
                {
                    ServerWriteThread_->interrupt();
                    isWriteRegistered=false;
                }


            }
        }
        return isConnectionOk;
    }
    bool SocketServer::read()
    {
        if(FD_ISSET(newFd_,&serverNew_set))
        {
            if(-1==(receivedBytes_ = recv(newFd_,buffer_,_BUFFER_SIZE_,0)))
            {
                if(!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
                {
                    isConnectionOk=false;
                    FD_CLR(newFd_, &serverNew_set);
                    close(newFd_);
                    isInitialized=false;
                    printf("read fail!!!\r\n");
                    return false;
                }
            }
            else
            {
                if(receivedBytes_ == 0)
                {
                    static bool first_time=true;
                    if(first_time)
                    {
                        printf("received empty\r\n");
                        first_time=false;
                    }
                    //isConnectionOk=false;
                    //isInitialized=false;
                    //FD_CLR(newFd_, &serverNew_set);
                    //close(newFd_);
                    return false;
                }
                else
                {
                   if(receivedBytes_> 6*sizeof(double)-1)
                   {
                       for(int i=0; i<receivedBytes_;i+=sizeof(double)*6)
                       {
                           memcpy(ps_->joint_pos_,&buffer_[i], sizeof(double)*6);
                           planning_state_buffer_.push(*ps_);
                       }
                   }
                }
            }
            return true;
        }

    }

    void SocketServer::closeServer()
    {
        close(newFd_);
    }
    void SocketServer::serverReadThread(int rate)
    {
        boost::this_thread::interruption_enabled();
        try
        {
            while(!isConnectionOk)
                boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
            while (isConnectionOk)
            {
                boost::this_thread::interruption_point();
                read_mutex_.lock();
                read();
                read_mutex_.unlock();
                boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
            }
        }
        catch (boost::thread_interrupted&e )
        {
            std::cout<<"client thread interrupted!"<<std::endl;
            //read_mutex_.unlock();
        }
    }
    void SocketServer::registerServerReadThread(int rate)
    {
        ServerReadThread_.reset(new boost::thread(boost::bind(&SocketServer::serverReadThread,this,rate)));
        isReadRegistered=true;
    }
    void SocketServer::serverWriteThread(nursing_namespace::PlanningState *ps_ptr,int rate)
    {
        boost::this_thread::interruption_enabled();
        try
        {
            while(!isConnectionOk)
                boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
            while (isConnectionOk)
            {
                boost::this_thread::interruption_point();
                write_mutex_.lock();
                memcpy(write_buffer_,ps_ptr,sizeof(nursing_namespace::PlanningState));
                write(write_buffer_,sizeof(nursing_namespace::PlanningState));
                write_mutex_.unlock();
                boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
            }
        }
        catch (boost::thread_interrupted&e )
        {
            std::cout<<"client thread interrupted!"<<std::endl;
            write_mutex_.unlock();
        }
    }
    void SocketServer::registerServerWriteThread(nursing_namespace::PlanningState *ps_ptr, int rate)
    {
        ServerWriteThread_.reset(new boost::thread(boost::bind(&SocketServer::serverWriteThread,this,ps_ptr,rate)));
        isWriteRegistered=true;
    }
    const nursing_namespace::PlanningState & SocketServer::getPlanningState()
    {
        if(!planning_state_buffer_.empty())
        {
            memcpy(ps_,&planning_state_buffer_.front(), sizeof(nursing_namespace::PlanningState));
            planning_state_buffer_.pop();
        }
        return *ps_;
    }
}
