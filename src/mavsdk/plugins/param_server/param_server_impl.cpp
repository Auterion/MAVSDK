#include "param_server_impl.h"

namespace mavsdk {

ParamServerImpl::ParamServerImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ParamServerImpl::ParamServerImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _parent->register_plugin(this);
}

ParamServerImpl::~ParamServerImpl()
{
    _parent->unregister_plugin(this);
}

void ParamServerImpl::init() {}

void ParamServerImpl::deinit() {}

void ParamServerImpl::enable() {}

void ParamServerImpl::disable() {}

std::pair<ParamServer::Result, int32_t> ParamServerImpl::retrieve_param_int(std::string name) const
{
    std::pair<MAVLinkParameters::Result, int> result = _parent->retrieve_server_param_int(name);

    if (result.first == MAVLinkParameters::Result::Success) {
        return {ParamServer::Result::Success, result.second};
    } else {
        return {ParamServer::Result::NotFound, -1};
    }
}

ParamServer::Result ParamServerImpl::provide_param_int(std::string name, int32_t value)
{
    _parent->provide_server_param_int(name, value);
    return ParamServer::Result::Success;
}

std::pair<ParamServer::Result, float> ParamServerImpl::retrieve_param_float(std::string name) const
{
    std::pair<MAVLinkParameters::Result, float> result = _parent->retrieve_server_param_float(name);

    if (result.first == MAVLinkParameters::Result::Success) {
        return {ParamServer::Result::Success, result.second};
    } else {
        return {ParamServer::Result::NotFound, NAN};
    }
}

ParamServer::Result ParamServerImpl::provide_param_float(std::string name, float value)
{
    _parent->provide_server_param_float(name, value);
    return ParamServer::Result::Success;
}

ParamServer::AllParams ParamServerImpl::retrieve_all_params() const
{
    auto tmp = _parent->retrieve_all_server_params();

    ParamServer::AllParams res{};

    for (auto const& parampair : tmp) {
        if (parampair.second.is<float>()) {
            ParamServer::FloatParam tmp_param;
            tmp_param.name = parampair.first;
            tmp_param.value = parampair.second.get<float>();
            res.float_params.push_back(tmp_param);
        } else if (parampair.second.is<int32_t>()) {
            ParamServer::IntParam tmp_param;
            tmp_param.name = parampair.first;
            tmp_param.value = parampair.second.get<int32_t>();
            res.int_params.push_back(tmp_param);
        }
    }

    return res;
}

ParamServer::Result
ParamServerImpl::result_from_mavlink_parameters_result(MAVLinkParameters::Result result)
{
    switch (result) {
        case MAVLinkParameters::Result::Success:
            return ParamServer::Result::Success;
        case MAVLinkParameters::Result::NotFound:
            return ParamServer::Result::NotFound;
        case MAVLinkParameters::Result::ParamNameTooLong:
            return ParamServer::Result::ParamNameTooLong;
        case MAVLinkParameters::Result::WrongType:
            return ParamServer::Result::WrongType;
        default:
            LogErr() << "Unknown param error";
            return ParamServer::Result::Unknown;
    }
}

} // namespace mavsdk
