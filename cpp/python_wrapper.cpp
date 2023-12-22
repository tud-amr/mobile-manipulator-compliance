#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "dingo_driver.h"

namespace py = pybind11;
using namespace dingo_driver;

PYBIND11_MODULE(dingo_driver, handle)
{
    handle.doc() = "Dingo driver.";
    py::class_<DriverManager>(handle, "DriverManager")
        .def(py::init<std::string, std::string>())
        .def("connect_gateway", &DriverManager::connect_gateway)
        .def("start_canread_loop", &DriverManager::start_canread_loop)
        .def("start_update_loop", &DriverManager::start_update_loop)
        .def("set_command", &DriverManager::set_command)
        .def_readonly("position", &DriverManager::pos_)
        .def_readonly("torque", &DriverManager::tor_);
}