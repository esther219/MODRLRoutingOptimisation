#include "MO-env.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <vector>
#include <numeric>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ns3::MOGymEnv");
NS_OBJECT_ENSURE_REGISTERED (MOGymEnv);

MOGymEnv::MOGymEnv ()
{
  NS_LOG_FUNCTION (this);
  SetOpenGymInterface(OpenGymInterface::Get());
}

MOGymEnv::~MOGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MOGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MOGymEnv")
    .SetParent<OpenGymEnv> ()
    .SetGroupName ("OpenGym")
  ;

  return tid;
}

void
MOGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

/*
Define action space
*/
Ptr<OpenGymSpace>
MOGymEnv::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (m_obs_link_num * 2 + 1);
    NS_LOG_UNCOND ("GetActionSpace: " << space);
    return space；
}

/*
Execute received actions
*/
bool
MOGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    uint32_t num = discrete->GetValue();
    uint32_t oplink = num % m_obs_link_num;
    if(num < m_obs_link_num){
        //num th link up
        Ptr <Node> n1 = NodeList::GetNode(m_link_interface_first[oplink].first);
        Ptr <Ipv4> ipNode1 = n1->GetObject<Ipv4> ();
        if(ipnode1->isUp(m_link_interface_first[oplink].second)){
            m_reward[1] = - m_obs_link_num; //invalid action, punishment
        }
        else{
	        Simulator::Schedule (Simulator::Now().GetSeconds(), &Ipv4::SetUp, ipNode1, m_link_interface_first[oplink].second);
        }

        Ptr <Node> n2 = NodeList::GetNode(m_link_interface_second[oplink].first);
        Ptr <Ipv4> ipNode2 = n2->GetObject<Ipv4> ();
        if(ipnode2->isUp(m_link_interface_second[oplink].second)){
            m_reward[1] = - m_obs_link_num;
        }
        else{
            Simulator::Schedule (Simulator::Now().GetSeconds(), &Ipv4::SetUp, ipNode2, m_link_interface_second[oplink].second);
        }

    }
    else if(num > m_obs_link_num){
        //num th link down
        Ptr <Node> n1 = NodeList::GetNode(m_link_interface_first[oplink].first);
        Ptr <Ipv4> ipNode1 = n1->GetObject<Ipv4> ();
        if(!ipnode1->isUp(m_link_interface_first[oplink].second)){
            m_reward[1] = - m_obs_link_num; //invalid action, punishment
        }
        else{
	        Simulator::Schedule (Simulator::Now().GetSeconds(), &Ipv4::SetDown, ipNode1, m_link_interface_first[oplink].second);
        }

        Ptr <Node> n2 = NodeList::GetNode(m_link_interface_second[oplink].first);
        Ptr <Ipv4> ipNode2 = n2->GetObject<Ipv4> ();
        if(!ipnode1->isUp(m_link_interface_second[oplink].second)){
            m_reward[1] = - m_obs_link_num; //invalid action, punishment
        }
        else{
	        Simulator::Schedule (Simulator::Now().GetSeconds(), &Ipv4::SetDown, ipNode2, m_link_interface_second[oplink].second);
        }
    }
    else{
         // num = 32 null action
    }

    NS_LOG_INFO ("MyExecuteActions: " << oplink;
    return true;
}

/*
Define reward function
*/
float
MOGymEnv::GetReward()
{
  NS_LOG_INFO("MyGetReward: " );
  return m_envReward;
}

/*
Define extra info. Optional
*/
std::string
MOGymEnv::GetExtraInfo()
{
  NS_LOG_INFO("MyGetExtraInfo: " << m_info);
  return m_info;
}

/*
Define game over condition
*/
bool
MOGymEnv::GetGameOver()
{
  m_isGameOver = false; // TODO
  return m_isGameOver;
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
TcpEventGymEnv::GetObservationSpace()
{
  // socket unique ID
  // tcp env type: event-based = 0 / time-based = 1
  // sim time in us
  // node ID
  // ssThresh
  // cWnd
  // segmentSize
  // segmentsAcked
  // bytesInFlight
  // rtt in us
  // min rtt in us
  // called func
  // congestion algorithm (CA) state
  // CA event
  // ECN state
  uint32_t parameterNum = m_obs_link_num;
  float low = -1000000000.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<double> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetObservationSpace: " << box);
  return box;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
MOGymEnv::GetObservation()
{
  uint32_t parameterNum = m_obs_link_num;
  std::vector<uint32_t> shape = {parameterNum,};

  Ptr<OpenGymBoxContainer<uint64_t> > box = CreateObject<OpenGymBoxContainer<uint64_t> >(shape);

  // TODO
  box->AddValue(m_socketUuid);
  box->AddValue(0);
  box->AddValue(Simulator::Now().GetMicroSeconds ());
  box->AddValue(m_nodeId);
  box->AddValue(m_tcb->m_ssThresh);
  box->AddValue(m_tcb->m_cWnd);
  box->AddValue(m_tcb->m_segmentSize);
  box->AddValue(m_segmentsAcked);
  box->AddValue(m_bytesInFlight);
  box->AddValue(m_rtt.GetMicroSeconds ());
  //box->AddValue(m_tcb->m_minRtt.GetMicroSeconds ());
  //box->AddValue(m_calledFunc);
  //box->AddValue(m_tcb->m_congState);
  //box->AddValue(m_event);
  //box->AddValue(m_tcb->m_ecnState);

  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);
  return box;
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
MOGymEnv::GetObservationSpace()
{
  // socket unique ID
  // tcp env type: event-based = 0 / time-based = 1
  // sim time in us
  // node ID
  // ssThresh
  // cWnd
  // segmentSize
  // bytesInFlightSum
  // bytesInFlightAvg
  // segmentsAckedSum
  // segmentsAckedAvg
  // avgRtt
  // minRtt
  // avgInterTx
  // avgInterRx
  // throughput
  uint32_t parameterNum = m_obs_link_num;
  float low = 0.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint64_t> ();

  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_INFO ("MyGetObservationSpace: " << box);
  return box;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
MOGymEnv::GetObservation()
{
  uint32_t parameterNum = m_obs_link_num;
  std::vector<uint32_t> shape = {parameterNum,};

  Ptr<OpenGymBoxContainer<uint64_t> > box = CreateObject<OpenGymBoxContainer<uint64_t> >(shape);

  box->AddValue(m_socketUuid);
  box->AddValue(1);
  box->AddValue(Simulator::Now().GetMicroSeconds ());
  box->AddValue(m_nodeId);
  box->AddValue(m_tcb->m_ssThresh);
  box->AddValue(m_tcb->m_cWnd);
  box->AddValue(m_tcb->m_segmentSize);

  //bytesInFlightSum
  uint64_t bytesInFlightSum = std::accumulate(m_bytesInFlight.begin(), m_bytesInFlight.end(), 0);
  box->AddValue(bytesInFlightSum);

  //bytesInFlightAvg
  uint64_t bytesInFlightAvg = 0;
  if (m_bytesInFlight.size()) {
    bytesInFlightAvg = bytesInFlightSum / m_bytesInFlight.size();
  }
  box->AddValue(bytesInFlightAvg);

  //segmentsAckedSum
  uint64_t segmentsAckedSum = std::accumulate(m_segmentsAcked.begin(), m_segmentsAcked.end(), 0);
  box->AddValue(segmentsAckedSum);

  //segmentsAckedAvg
  uint64_t segmentsAckedAvg = 0;
  if (m_segmentsAcked.size()) {
    segmentsAckedAvg = segmentsAckedSum / m_segmentsAcked.size();
  }
  box->AddValue(segmentsAckedAvg);

  //avgRtt
  Time avgRtt = Seconds(0.0);
  if(m_rttSampleNum) {
    avgRtt = m_rttSum / m_rttSampleNum;
  }
  box->AddValue(avgRtt.GetMicroSeconds ());

  //m_minRtt
  box->AddValue(m_tcb->m_minRtt.GetMicroSeconds ());

  //avgInterTx
  Time avgInterTx = Seconds(0.0);
  if (m_interTxTimeNum) {
    avgInterTx = m_interTxTimeSum / m_interTxTimeNum;
  }
  box->AddValue(avgInterTx.GetMicroSeconds ());

  //avgInterRx
  Time avgInterRx = Seconds(0.0);
  if (m_interRxTimeNum) {
    avgInterRx = m_interRxTimeSum / m_interRxTimeNum;
  }
  box->AddValue(avgInterRx.GetMicroSeconds ());

  //throughput  bytes/s
  float throughput = (segmentsAckedSum * m_tcb->m_segmentSize) / m_timeStep.GetSeconds();
  box->AddValue(throughput);

/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/

  // Update reward based on overall average of avgRtt over all steps so far
  // only when agent increases cWnd
  // TODO: this is not the right way of doing this.
  // place this somewhere else. see TcpEventGymEnv, how they've done it.

  if (m_new_cWnd > m_old_cWnd && m_totalAvgRttSum > 0 && avgRtt > 0)  {
    // when agent increases cWnd
    if ((m_totalAvgRttSum / m_totalAvgRttNum) >= avgRtt)  {
      // give reward for decreasing avgRtt
      m_envReward = m_reward;
    } else {
      // give penalty for increasing avgRtt
      m_envReward = m_penalty;
    }
  } else  {
    // agent has not increased cWnd
    m_envReward = 0;
  }

  // Update m_totalAvgRtSum and m_totalAvgRttNum
  m_totalAvgRttSum += avgRtt;
  m_totalAvgRttNum++;

  m_old_cWnd = m_new_cWnd;
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/

  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);

  m_bytesInFlight.clear();
  m_segmentsAcked.clear();

  m_rttSampleNum = 0;
  m_rttSum = MicroSeconds (0.0);

  m_interTxTimeNum = 0;
  m_interTxTimeSum = MicroSeconds (0.0);

  m_interRxTimeNum = 0;
  m_interRxTimeSum = MicroSeconds (0.0);

  return box;
}

} // namespace ns3