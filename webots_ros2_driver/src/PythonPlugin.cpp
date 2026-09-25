#include "webots_ros2_driver/PythonPlugin.hpp"

#include <webots/robot.h>

static PyObject *gPyWebotsNode = NULL;

namespace {
  // The driver's main thread only holds the GIL while it runs Python plugin code.
  // Holding it from Py_Initialize on (upstream behaviour) starves every thread a
  // Python plugin starts: they only get the interpreter when the main thread enters
  // Python again, i.e. their work runs on the simulation step path in 5 ms slices.
  void initializePython() {
    if (Py_IsInitialized())
      return;
    Py_Initialize();
    PyEval_SaveThread();
  }

  class GilLock {
  public:
    GilLock() : mState(PyGILState_Ensure()) {}
    ~GilLock() { PyGILState_Release(mState); }

  private:
    PyGILState_STATE mState;
  };
}  // namespace

namespace webots_ros2_driver {
  PythonPlugin::PythonPlugin(PyObject *pyPlugin) : mPyPlugin(pyPlugin){};

  void PythonPlugin::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    GilLock gil;
    PyObject *pyParameters = PyDict_New();
    for (const std::pair<std::string, std::string> &parameter : parameters)
      PyDict_SetItem(pyParameters, PyUnicode_FromString(parameter.first.c_str()),
                     PyUnicode_FromString(parameter.second.c_str()));

    PyObject_CallMethod(mPyPlugin, "init", "OO", getPyWebotsNodeInstance(), pyParameters);
    PyErr_Print();
  }

  void PythonPlugin::step() {
    if (mNextStepTime > 0.0 && wb_robot_get_time() + 1e-9 < mNextStepTime)
      return;
    GilLock gil;
    PyObject *result = PyObject_CallMethod(mPyPlugin, "step", "");
    if (result && PyFloat_Check(result))
      mNextStepTime = PyFloat_AsDouble(result);
    else if (result && PyLong_Check(result))
      mNextStepTime = PyLong_AsDouble(result);
    else
      mNextStepTime = -1.0;
    if (result)
      Py_DECREF(result);
    else
      PyErr_Print();
  }

  void PythonPlugin::stop() {
    // Py_Finalize needs the GIL and the main thread state, which PyGILState_Ensure
    // restores on the main thread; the GIL is not given back afterwards.
    PyGILState_Ensure();
    PyObject_CallMethod(mPyPlugin, "stop", "");
    Py_Finalize();
  }

  PyObject *PythonPlugin::getPyWebotsNodeInstance() {
    if (gPyWebotsNode)
      return gPyWebotsNode;

    PyObject *pyWebotsExtraModuleSource = Py_CompileString(
      R"EOT(
import os
import sys

import controller
from controller import Supervisor

# As Driver need the controller library, we extend the path here
# to avoid to load another library named "controller" when loading vehicle library
sys.path.insert(1, os.path.dirname(controller.__file__))
from vehicle import Driver

class WebotsNode:
    def __init__(self):
        if Driver.isInitialisationPossible():
            self.robot = Driver()
        else:
            self.robot = Supervisor()
)EOT",
      "webots_extra", Py_file_input);

    if (!pyWebotsExtraModuleSource)
      throw std::runtime_error("Error: The Python module with the WebotsNode class cannot be compiled.");

    PyObject *pyWebotsExtraModule = PyImport_ExecCodeModule("webots_extra", pyWebotsExtraModuleSource);

    if (!pyWebotsExtraModule)
      throw std::runtime_error("Error: The Python module with the WebotsNode class cannot be executed.");

    PyObject *pyDict = PyModule_GetDict(pyWebotsExtraModule);
    PyObject *pyClass = PyDict_GetItemString(pyDict, "WebotsNode");
    gPyWebotsNode = PyObject_CallObject(pyClass, nullptr);
    return gPyWebotsNode;
  }

  std::shared_ptr<PythonPlugin> PythonPlugin::createFromType(const std::string &type) {
    const std::string moduleName = type.substr(0, type.find_last_of("."));
    const std::string className = type.substr(type.find_last_of(".") + 1);

    initializePython();
    GilLock gil;

    PyObject *pyName = PyUnicode_FromString(moduleName.c_str());
    PyObject *pyModule = PyImport_Import(pyName);
    PyErr_Print();

    // If the module cannot be found the error should be handled in the upper level (e.g. try loading C++ plugin)
    if (!pyModule)
      return NULL;

    PyObject *pyDict = PyModule_GetDict(pyModule);
    PyObject *pyClass = PyDict_GetItemString(pyDict, className.c_str());
    PyErr_Print();
    if (!pyClass)
      throw std::runtime_error("Error in plugin " + type + ": The class " + className + " cannot be found.");

    PyObject *pyPlugin = PyObject_CallObject(pyClass, nullptr);
    return std::shared_ptr<PythonPlugin>(new PythonPlugin(pyPlugin));
  }
}  // namespace webots_ros2_driver
