/**
   Sample Robot motion controller for the JVRC robot model.
   This program was ported from the "SR1WalkControllerRTC.h" sample of Choreonoid.
*/

#ifndef RobotSensorsControllerRTC_H
#define RobotSensorsControllerRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/MultiValueSeq>
#include <cnoid/corba/CameraImage.hh>

class RobotSensorsControllerRTC : public RTC::DataFlowComponentBase
{
public:
    RobotSensorsControllerRTC(RTC::Manager* manager);
    ~RobotSensorsControllerRTC();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;
    RTC::TimedDoubleSeq m_gsensor;
    RTC::InPort<RTC::TimedDoubleSeq> m_gsensorIn;
    RTC::TimedDoubleSeq m_gyrometer;
    RTC::InPort<RTC::TimedDoubleSeq> m_gyrometerIn;
    RTC::TimedDoubleSeq m_lfsensor;
    RTC::InPort<RTC::TimedDoubleSeq> m_lfsensorIn;
    RTC::TimedDoubleSeq m_rfsensor;
    RTC::InPort<RTC::TimedDoubleSeq> m_rfsensorIn;
    Img::TimedCameraImage m_lcamera;
    RTC::InPort<Img::TimedCameraImage> m_lcameraIn;
    Img::TimedCameraImage m_rcamera;
    RTC::InPort<Img::TimedCameraImage> m_rcameraIn;
    RTC::RangeData m_ranger;
    RTC::InPort<RTC::RangeData> m_rangerIn;
};

extern "C"
{
    DLL_EXPORT void RobotSensorsControllerRTCInit(RTC::Manager* manager);
};

#endif
