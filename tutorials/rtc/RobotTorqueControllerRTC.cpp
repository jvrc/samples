/**
   Sample Robot motion controller for the JVRC robot model.
   This program was ported from the "SR1WalkControllerRTC.cpp" sample of
   Choreonoid.
*/

#include "RobotTorqueControllerRTC.h"
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

const char* samplepd_spec[] =
{
    "implementation_id", "RobotTorqueControllerRTC",
    "type_name",         "RobotTorqueControllerRTC",
    "description",       "Robot TorqueController component",
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


RobotTorqueControllerRTC::RobotTorqueControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_torqueOut("u", m_torque)
{
}

RobotTorqueControllerRTC::~RobotTorqueControllerRTC()
{

}


RTC::ReturnCode_t RobotTorqueControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);

    // Set OutPort buffer
    addOutPort("u", m_torqueOut);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotTorqueControllerRTC::onActivated(RTC::UniqueId ec_id)
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


RTC::ReturnCode_t RobotTorqueControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t RobotTorqueControllerRTC::onExecute(RTC::UniqueId ec_id)
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


extern "C"
{
    DLL_EXPORT void RobotTorqueControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<RobotTorqueControllerRTC>,
                                 RTC::Delete<RobotTorqueControllerRTC>);
    }
};
