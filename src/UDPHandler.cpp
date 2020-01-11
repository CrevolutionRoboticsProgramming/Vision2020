#include "UDPHandler.hpp"

UDPHandler::UDPHandler(int port) : mSocket{mIoService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)}
{
    startReceiving();
    start();
}

UDPHandler::~UDPHandler()
{
    stop();
}

void UDPHandler::run()
{
    mIoService.run();
}

void UDPHandler::stop()
{
    mIoService.stop();
    Thread::stop();
}

void UDPHandler::startReceiving()
{
    // Populates mRemoteEndpoint
    mSocket.async_receive_from(
        boost::asio::buffer(mReceiveBuffer), mRemoteEndpoint,
        boost::bind(&UDPHandler::handleReceive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void UDPHandler::sendTo(std::string message, boost::asio::ip::udp::endpoint sendEndpoint)
{
    boost::shared_ptr<std::string> messagePtr(new std::string(message));

    // Consumes sendEndpoint
    mSocket.async_send_to(boost::asio::buffer(*messagePtr), sendEndpoint,
                          boost::bind(&UDPHandler::handleSend, this, messagePtr,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
}

void UDPHandler::reply(std::string message)
{
    sendTo(message, mRemoteEndpoint);
}

void UDPHandler::handleReceive(const boost::system::error_code &error,
                               std::size_t bytesTransferred)
{
    if (!error || error == boost::asio::error::message_size)
    {
        mReceivedMessage = std::string{mReceiveBuffer.data()}.substr(0, bytesTransferred);

        reply("received");
    }
    startReceiving();
}

void UDPHandler::handleSend(boost::shared_ptr<std::string> /*message*/,
                            const boost::system::error_code & /*error*/,
                            std::size_t /*bytes_transferred*/)
{
}

std::string UDPHandler::getMessage()
{
    return mReceivedMessage;
}

void UDPHandler::clearMessage()
{
    mReceivedMessage.clear();
}
