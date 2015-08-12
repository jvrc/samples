/**
   Sample Robot motion controller for the JVRC robot model.
   This program was ported from the "SR1WalkControllerRTC.cpp" sample of
   Choreonoid.
*/

#include "RobotSensorsControllerRTC.h"
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const char* samplepd_spec[] =
{
    "implementation_id", "RobotSensorsControllerRTC",
    "type_name",         "RobotSensorsControllerRTC",
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


RobotSensorsControllerRTC::RobotSensorsControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_gsensorIn("gsensor", m_gsensor),
      m_gyrometerIn("gyrometer", m_gyrometer),
      m_lfsensorIn("lfsensor", m_lfsensor),
      m_rfsensorIn("rfsensor", m_rfsensor),
      m_lcameraIn("lcamera", m_lcamera),
      m_rcameraIn("rcamera", m_rcamera),
      m_rangerIn("ranger", m_ranger)
{

}

RobotSensorsControllerRTC::~RobotSensorsControllerRTC()
{

}


RTC::ReturnCode_t RobotSensorsControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);
    addInPort("gsensor", m_gsensorIn);
    addInPort("gyrometer", m_gyrometerIn);
    addInPort("lfsensor", m_lfsensorIn);
    addInPort("rfsensor", m_rfsensorIn);
    addInPort("lcamera", m_lcameraIn);
    addInPort("rcamera", m_rcameraIn);
    addInPort("ranger", m_rangerIn);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotSensorsControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t RobotSensorsControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotSensorsControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
            m_angleIn.read();
    }

    for(size_t i=0; i < m_angle.data.length(); ++i){
            // cout << "m_angle.data[" << i << "] is " << m_angle.data[i] << std::endl;
    }

    if(m_gsensorIn.isNew()){
            m_gsensorIn.read();
    }

    for(size_t i=0; i < m_gsensor.data.length(); ++i){
            cout << "m_gsensor.data[" << i << "] is " << m_gsensor.data[i] << std::endl;
    }

    if(m_gyrometerIn.isNew()){
            m_gyrometerIn.read();
    }

    for(size_t i=0; i < m_gyrometer.data.length(); ++i){
            cout << "m_gyrometer.data[" << i << "] is " << m_gyrometer.data[i] << std::endl;
    }

    if(m_lfsensorIn.isNew()){
            m_lfsensorIn.read();
    }

    for(size_t i=0; i < m_lfsensor.data.length(); ++i){
            cout << "m_lfsensor.data[" << i << "] is " << m_lfsensor.data[i] << std::endl;
    }

    if(m_rfsensorIn.isNew()){
            m_rfsensorIn.read();
    }

    for(size_t i=0; i < m_rfsensor.data.length(); ++i){
            cout << "m_rfsensor.data[" << i << "] is " << m_rfsensor.data[i] << std::endl;
    }

    // if(m_lcameraIn.isNew()){
    //     m_lcameraIn.read();
    // }
    //
    // for(size_t i=0; i < m_lcamera.data.image.raw_data.length(); ++i){
    //         cout << "m_lcamera.data.image.raw_data[" << i <<
    //                 "] is " << m_lcamera.data.image.raw_data[i] << std::endl;
    // }
    //
    // if(m_rcameraIn.isNew()){
    //     m_rcameraIn.read();
    // }
    //
    // for(size_t i=0; i < m_rcamera.data.image.raw_data.length(); ++i){
    //         cout << "m_rcamera.data.image.raw_data[" << i <<
    //                 "] is " << m_rcamera.data.image.raw_data[i] << std::endl;
    // }

    if(m_rangerIn.isNew()){
            m_rangerIn.read();
    }

    for(size_t i=0; i < m_ranger.ranges.length(); ++i){
            cout << "m_ranger.ranges[" << i << "] is " << m_ranger.ranges[i] << std::endl;
    }
    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void RobotSensorsControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<RobotSensorsControllerRTC>,
                                 RTC::Delete<RobotSensorsControllerRTC>);
    }
};
