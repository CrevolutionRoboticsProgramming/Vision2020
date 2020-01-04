#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <string>
#include <iostream>
#include <vector>

#include "Thread.hpp"

class UDPHandler : public Thread
{
public:
    // Defined later
    class MessageReceiver;

private:
    boost::asio::io_service mIoService;
    boost::asio::ip::udp::socket mSocket;
    boost::asio::ip::udp::endpoint mRemoteEndpoint;
    boost::array<char, 1024> mReceiveBuffer;
    std::string mReceivedMessage;

    std::vector<MessageReceiver *> mReceivers;

    void startReceiving();
    void handleReceive(const boost::system::error_code &error,
                       std::size_t bytesTransferred);
    void handleSend(boost::shared_ptr<std::string> /*message*/,
                    const boost::system::error_code & /*error*/,
                    std::size_t /*bytes_transferred*/);
    void run() override;
    void stop() override;

public:
    UDPHandler(int port);
    ~UDPHandler();

    class MessageReceiver
    {
    public:
        virtual void run(std::string message) = 0;
        virtual std::string getLabel() = 0;
    };

    void sendTo(std::string message, boost::asio::ip::udp::endpoint sendEndpoint);
    void reply(std::string message);
    void addReceiver(MessageReceiver *receiver);
    std::string getMessage();
    void clearMessage();
};
