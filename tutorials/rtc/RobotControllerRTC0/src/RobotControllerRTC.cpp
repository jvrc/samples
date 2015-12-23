// -*- C++ -*-
/*!
 * @file  RobotControllerRTC.cpp
 * @brief Robot Controller component
 * @date $Date$
 *
 * $Id$
 */

#include "RobotControllerRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* robotcontrollerrtc_spec[] =
  {
    "implementation_id", "RobotControllerRTC",
    "type_name",         "RobotControllerRTC",
    "description",       "Robot Controller component",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RobotControllerRTC::RobotControllerRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleIn("q", m_angle)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RobotControllerRTC::~RobotControllerRTC()
{
}



RTC::ReturnCode_t RobotControllerRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_angleIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotControllerRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


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
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RobotControllerRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotControllerRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RobotControllerRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(robotcontrollerrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<RobotControllerRTC>,
                             RTC::Delete<RobotControllerRTC>);
  }
  
};


