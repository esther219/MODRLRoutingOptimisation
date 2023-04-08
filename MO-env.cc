#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <vector>
#include <map>
#include <utility>
#include <set>
#include <sstream>
#include <algorithm>

#include "MO-env.h"
#include "ns3/object.h"
#include "ns3/node-list.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/core-module.h"
#include "ns3/opengym-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/csma-module.h"
#include "ns3/ipv4-nix-vector-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/constant-position-mobility-model.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ns3::MOGymEnv");
NS_OBJECT_ENSURE_REGISTERED (MOGymEnv);

MOGymEnv::MOGymEnv (Time stepTime)
{
  NS_LOG_FUNCTION (this);
  m_obs_link_num = 32;
  m_op_link_num = 20;
  m_node_num = 36;
  m_pod_num = 4;
  m_interval = stepTime;
  m_isGameOver = false;

  Simulator::Schedule(stepTime,&MOGymEnv::ScheduleNextStateRead,this);
  SetOpenGymInterface(OpenGymInterface::Get());
}

void
MOGymEnv::ScheduleNextStateRead()
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule(m_interval,&MOGymEnv::ScheduleNextStateRead,this);
  Notify();
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
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (m_op_link_num * 2 + 1);
    NS_LOG_UNCOND ("GetActionSpace: " << space);
    return space;
}

/*
Execute received actions
*/
bool
MOGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    uint32_t oplink = discrete->GetValue();
    if(oplink>0 && oplink<m_op_link_num){
        //num th link up
        Ptr <Node> n1 = NodeList::GetNode(m_op_link_interface_first[oplink].first);
        Ptr <Ipv4> ipnode1 = n1->GetObject<Ipv4> ();
        if(ipnode1->IsUp(m_op_link_interface_first[oplink].second)){
            m_reward1 = m_reward0=- m_obs_link_num; //invalid action, punishment
        }
        else{
	    Simulator::Schedule (Simulator::Now(), &Ipv4::SetUp, ipnode1, m_op_link_interface_first[oplink].second);
        }

        Ptr <Node> n2 = NodeList::GetNode(m_op_link_interface_second[oplink].first);
        Ptr <Ipv4> ipnode2 = n2->GetObject<Ipv4> ();
        if(ipnode2->IsUp(m_op_link_interface_second[oplink].second)){
            m_reward1 = m_reward0 = - m_obs_link_num;
        }
        else{
            Simulator::Schedule (Simulator::Now(), &Ipv4::SetUp, ipnode2, m_op_link_interface_second[oplink].second);
        }

    }
    else if(oplink > m_op_link_num){
        //num th link down
        Ptr <Node> n1 = NodeList::GetNode(m_op_link_interface_first[oplink].first);
        Ptr <Ipv4> ipnode1 = n1->GetObject<Ipv4> ();
        if(!ipnode1->IsUp(m_op_link_interface_first[oplink].second)){
            m_reward1=m_reward0 = - m_obs_link_num; //invalid action, punishment
        }
        else{
	        Simulator::Schedule (Simulator::Now(), &Ipv4::SetDown, ipnode1, m_op_link_interface_first[oplink].second);
        }

        Ptr <Node> n2 = NodeList::GetNode(m_op_link_interface_second[oplink].first);
        Ptr <Ipv4> ipnode2 = n2->GetObject<Ipv4> ();
        if(!ipnode1->IsUp(m_op_link_interface_second[oplink].second)){
            m_reward1=m_reward0 = - m_obs_link_num; //invalid action, punishment
        }
        else{
	        Simulator::Schedule (Simulator::Now(), &Ipv4::SetDown, ipnode2, m_op_link_interface_second[oplink].second);
        }
    }
    else{
         // num = 20 null action
    }

    NS_LOG_INFO ("MyExecuteActions: " << oplink);
    return true;
}

/*
Define reward function
*/
float
MOGymEnv::GetReward()
{
  NS_LOG_INFO("MyGetReward: " );
  static double last_value1 = 0.0;
  if(m_reward0==-m_obs_link_num && m_reward1==- m_obs_link_num*1.0){
    m_info = std::to_string(m_reward0);
    return m_reward1; //invalid action
  }

  // compute energy reward (m_reward0)
  double r1=0;
  for(NodeList::Iterator i=NodeList::Begin();i!=NodeList::End();++i){
     Ptr<Node> node = *i;
     Ptr <Ipv4> ipNode = node->GetObject<Ipv4> ();
     bool node_down=true;
     for(uint32_t j=1;j<=m_pod_num;j++){
        if(ipNode->IsUp(j)){
            node_down=false;
        }
        else{
            r1+=2;
        }
     }
     if(node_down){
        r1+=90;
     }
  }
  m_reward0=r1-last_value1;
  last_value1=r1;

  // compute network performance reward(m_reward[1])
  //final reward : fct
  if(m_fct!=0){
       m_reward1=-m_fct*m_obs_link_num;
       m_isGameOver = true;
  }
  //step reward : timestep
  else{
    int total_queue_size=0;
    for (NodeContainer::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
    {
      Ptr<Node> node = *i;
      for (uint32_t j = 0; j < node->GetNDevices (); ++j)
        {
          Ptr<PointToPointNetDevice> d = DynamicCast<PointToPointNetDevice>(node->GetDevice(j));
          total_queue_size+=d->GetQueue()->GetNPackets();
        }
    }
    m_reward1=total_queue_size;
  }
  m_info = std::to_string(m_reward0);
  return m_reward1; //invalid action
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
  return m_isGameOver;
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
MOGymEnv::GetObservationSpace()
{
  // link utilization
  uint32_t parameterNum = m_obs_link_num;
  double low = -1.0;
  double high = 1.0;
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

  Ptr<OpenGymBoxContainer<double> > box = CreateObject<OpenGymBoxContainer<double> >(shape);

  for(uint32_t i=0;i<m_obs_link_num;i++){
    int i0,j0,i1,j1;
    i0=m_obs_link_interface_first[i].first;
    j0=m_obs_link_interface_first[i].second;
    i1=m_obs_link_interface_second[i].first;
    i1=m_obs_link_interface_second[i].second;

    if(m_link_utilization[i]&&(std::max(m_end_time[i0][j0],m_end_time[i1][j1])-std::min(m_start_time[i0][j0],m_start_time[i1][j1]))){
        m_link_utilization[i]=m_total_bytes[i0][j0]>=m_total_bytes[i1][j1]?m_total_bytes[i0][j0]:m_total_bytes[i1][j1];
        m_link_utilization[i] = m_link_utilization[i] / ((std::max(m_end_time[i0][j0],m_end_time[i1][j1])-std::min(m_start_time[i0][j0],m_start_time[i1][j1])) * 10 * 1000000000);
    }
    Ptr <Node> n = NodeList::GetNode(m_obs_link_interface_first[i].first);
    Ptr <Ipv4> ipnode = n->GetObject<Ipv4> ();
    if(!ipnode->IsUp(m_obs_link_interface_first[i].second)){
        box->AddValue(-1);
    }
    else{
    box->AddValue(m_link_utilization[i]);
    }
  }
  
  // clear m_total_bytes
  /*
  std::for_each(m_total_bytes.begin(), m_total_bytes.end(),
              [](auto& sub) {
                  std::fill(sub.begin(), sub.end(), 0);
              });
  */

   for (auto& sub : m_total_bytes) {
    std::fill(sub.begin(), sub.end(), 0);
    }
  // Print data
  NS_LOG_INFO ("MyGetObservation: " << box);
  return box;

}


void MOGymEnv::TxTrace(Ptr<MOGymEnv> entity,int node_id,int dev_id, Ptr<Packet const> packet )
{

    if(m_start_time[node_id][dev_id]==0){
        m_start_time[node_id][dev_id]=Simulator::Now().GetSeconds();
    }
    m_end_time[node_id][dev_id]=Simulator::Now().GetSeconds();
    m_total_bytes[node_id][dev_id]+=packet->GetSize();
}


/*
void MOGymEnv::TxTrace(std::string context, Ptr<Packet const> packet )
{
    cout<<"TXTrace"<<endl;
    std::cout<<context<<endl;
    char* p = (char *)context.data();
    char buf1[255];
	char buf2[255];
	char buf3[255];
	char buf4[255];
    char buf5[255];
    sscanf(p, "/%[^/]/%[^/]/%[^/]/%[^/]/%s", buf1, buf2, buf3, buf4,buf5);
    cout<<buf2<<" "<<buf4<<endl;
    int dev = atoi(buf2);
    int iface = atoi(buf4);
    if(m_start_time[dev][iface]==0){
        m_start_time[dev][iface]=Simulator::Now().GetSeconds();
    }
    m_end_time[dev][iface]=Simulator::Now().GetSeconds();
    m_total_bytes[dev][iface]+=packet->GetSize();
}

void MOGymEnv::SetupTxTrace(int dev_id, int ith_id)
{
    std::ostringstream oss;
    cout<<"setuptxtrace"<<endl;
    oss << "/NodeList/"<<dev_id<<"/DeviceList/"<<ith_id<<"/$ns3::PointToPointNetDevice/TxQueue/Dequeue";
    Config::Connect (oss.str (), MakeCallback (&MOGymEnv::TxTrace,this));
}
*/

} // namespace ns3
