// -*- C++ -*-
/*!
 * @file  RobotControllerRTC.cpp
 * @brief Robot Controller component
 * @date $Date$
 *
 * $Id$
 */

#include "RobotControllerRTC.h"
#include <iostream>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

namespace {

  static const double pgain[] = {
    50000.0, 30000.0, 30000.0, 30000.0, 30000.0,
    80000.0, 80000.0, 10000.0, 3000.0, 30000.0,
    30000.0, 80000.0, 3000.0, 30000.0, 10000.0,
    3000.0, 3000.0, 30000.0, 30000.0, 10000.0,
    3000.0, 30000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 10000.0, 3000.0, 3000.0,
    30000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0, 3000.0,
  };

  static const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0,
  };
};

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
    m_angleIn("q", m_angle),
    m_torqueOut("u", m_torque)

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
  addOutPort("u", m_torqueOut);
  
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
  if(!qseq){
    string filename = getNativePathString(
					  boost::filesystem::path(shareDirectory())
					  / "motion" / "RobotPattern.yaml");

    BodyMotion motion;

    if(!motion.loadStandardYAMLformat(filename)){
      cout << motion.seqMessage() << endl;
      return RTC::RTC_ERROR;
    }
    qseq = motion.jointPosSeq();
    if(qseq->numFrames() == 0){
      cout << "Empty motion data." << endl;
      return RTC::RTC_ERROR;
    }
    q0.resize(qseq->numParts());
    timeStep_ = qseq->getTimeStep();
  }

  m_torque.data.length(qseq->numParts());

  if(m_angleIn.isNew()){
    m_angleIn.read();
  }
  for(int i=0; i < qseq->numParts(); ++i){
    q0[i] = m_angle.data[i];
  }
  oldFrame = qseq->frame(0);
  currentFrame = 0;

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

  MultiValueSeq::Frame frame;
  
  if(currentFrame > qseq->numFrames()){
    frame = oldFrame;
  }else{
    frame = qseq->frame(currentFrame++);
  }
  
  for(int i=0; i < frame.size(); i++){
    double q_ref = frame[i];
    double q = m_angle.data[i];
    double dq_ref = (q_ref - oldFrame[i]) / timeStep_;
    double dq = (q - q0[i]) / timeStep_;
    m_torque.data[i] = (q_ref - q) * pgain[i]/100.0 + (dq_ref - dq) * dgain[i]/100.0;
    q0[i] = q;
#if 0
    cout << "i = " << i << " ";
    cout << "q_ref = " << frame[i] << " ";
    cout << "q = " << q << " ";
    cout << "dq_ref = " << dq_ref << " ";
    cout << "dq = " << dq << " ";
    cout << "torque = " << m_torque.data[i] << endl;
#endif
  }
  oldFrame = frame;

  m_torqueOut.write();

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


