#ifndef PERCEPTION_INTERFACE_PERCEPTION_INTERFACE_H
#define PERCEPTION_INTERFACE_PERCEPTION_INTERFACE_H

#include <exception>
#include <string>
#include <set>
#include <typeinfo>

#include "mhri_msgs/ReadData.h"
#include "mhri_msgs/WirteData.h"

namespace perception_interface
{
class PerceptionInterface
{
public:
    virtual PerceptionInterface():
    _is_paused(false);
    {

    }
    virtual ~PerceptionInterface() {}

    virtual bool init() const = 0;
    virtual bool start() const = 0;
    virtual bool pause() const = 0;
    virtual bool resume() const = 0;

private:
    bool read_data()
    {
        return true;
    }

    bool write_data()
    {
        return true;
    }

private:
    bool _is_paused;
};

class PerceptionInterfaceException: public std::exception
{
public:
    PerceptionInterfaceException(const std::string& message)
    : msg(message) {}

    virtual ~PerceptionInterfaceException() throw() {}

    virtual const char* what() const throw()
    {
        return msg.c_str();
    }

private:
    std::string msg;
};
}

#endif
