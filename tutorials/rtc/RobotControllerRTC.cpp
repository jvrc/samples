/**
   Sample Robot motion controller for the JVRC robot model.
   This program was ported from the "SR1WalkControllerRTC.cpp" sample of
   Choreonoid.
*/

#include "RobotControllerRTC.h"
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const char* samplepd_spec[] =
{
    "implementation_id", "RobotControllerRTC",
    "type_name",         "RobotControllerRTC",
    "description",       "Robot Controller component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};
}


RobotControllerRTC::RobotControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle)
{

}

RobotControllerRTC::~RobotControllerRTC()
{

}


RTC::ReturnCode_t RobotControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    for(size_t i=0; i < m_angle.data.length(); ++i){
            cout << "m_angle.data[" << i << "] is " << m_angle.data[i] << std::endl;
    }

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void RobotControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<RobotControllerRTC>,
                                 RTC::Delete<RobotControllerRTC>);
    }
};
