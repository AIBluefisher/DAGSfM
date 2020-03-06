#include "map_reduce/communicator.h"

namespace GraphSfM {

Communicator::Communicator()
    : server_(rpc::server(options_.port))
{

}

Communicator::Communicator(const CommunicatorOptions& options)
    : options_(options),
      server_(rpc::server(options_.port))
{

}

void Communicator::Listening(const bool async)
{
    if (async) {
        server_.async_run();
    } else {
        server_.run();
    }
}

bool Communicator::IsReachable(const std::string& ip, const uint16_t port) const
{
    using namespace rpc;
    client c(ip, port);
    client::connection_state cs = c.get_connection_state();
    if (cs == client::connection_state::connected) {
        return true;
    }

    return false;
}

rpc::server& Communicator::Server()
{
    return server_;
}

} // namespace GraphSfM