#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "dingo_driver.h"

namespace py = pybind11;
using namespace dingo_driver;

PYBIND11_MODULE(dingo_driver, handle)
{
    handle.doc() = "Dingo driver.";
    py::class_<DriverManager>(handle, "DriverManager")
        .def(py::init<std::string>())
        .def("connect_gateway", &DriverManager::connect_gateway)
        .def("add_actuator", &DriverManager::add_actuator)
        .def("set_mode", &DriverManager::set_mode)
        .def("initialize_encoders", &DriverManager::initialize_encoders)
        .def("initialize_encoders", &DriverManager::command)
        .def("canread", &DriverManager::canread)
        .def("get_states", &DriverManager::get_states);
}