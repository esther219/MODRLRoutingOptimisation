#ifndef MO_ENV_H
#define MO_ENV_H

#include "ns3/opengym-module.h"
#include <vector>

namespace ns3 {

class Time;


class MOGymEnv : public OpenGymEnv
{
public:
  MOGymEnv ();
  virtual ~MOGymEnv ();
  static TypeId GetTypeId (void);
  virtual void DoDispose ();

  void SetReward(double value1, double value2);
  void SetupTxTrace(int dev_id, int ith_id);
  void TxTrace(std::string context, Ptr<Packet const> packet);


  //std::string GetTcpCongStateName(const TcpSocketState::TcpCongState_t state);
  //std::string GetTcpCAEventName(const TcpSocketState::TcpCAEvent_t event);

  // OpenGym interface
  virtual Ptr<OpenGymSpace> GetActionSpace();
  virtual bool GetGameOver();
  virtual float GetReward();
  virtual std::string GetExtraInfo();
  virtual bool ExecuteActions(Ptr<OpenGymDataContainer> action);

  virtual Ptr<OpenGymSpace> GetObservationSpace() = 0;
  virtual Ptr<OpenGymDataContainer> GetObservation() = 0;
  int m_fct = 0;

  // trace packets, e.g. for calculating inter tx/rx time
  //virtual void TxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>) = 0;
  //virtual void RxPktTrace(Ptr<const Packet>, const TcpHeader&, Ptr<const TcpSocketBase>) = 0;

  // TCP congestion control interface
 // virtual uint32_t GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) = 0;
  //virtual void IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) = 0;
  // optional functions used to collect obs
  //virtual void PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt) = 0;
  //virtual void CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState) = 0;
  //virtual void CwndEvent (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event) = 0;


protected:
  uint32_t m_obs_link_num;
  uint32_t m_op_link_num;
  uint32_t m_node_num;
  uint32_t m_pod_num;

  // state
  // obs has to be implemented in child class

  // game over
  bool m_isGameOver;


  // extra info
  std::string m_info;

  // actions
  uint32_t m_new_index;


private:
  // reward
  double m_reward[2];

  void ScheduleNextStateRead();

  Time m_duration;
  Time m_timeStep;
  Time m_interval = Seconds(0.005);

  vector < vector<uint64_t> > m_total_bytes (40,vector<uint64_t>(8,0));
  vector < vector<double> > m_start_time (40,vector<double>(8,0));
  vector < vector<double> > m_end_time (40, vector<double>(8,0));
  vector < vector<double> >m_link_utilization(40,0);
  vector <pair<int,double>> m_fct;

  // (A,B) : node A device(interface) B
  std::vector<pair<int,int>> m_op_link_interface_first = {(7,1),(7,2),(15,1),(15,2),
                                                       (23,1),(23,2),(31,1),(31,2),
                                                       (1,1),(1,2),(1,3),(1,4),
                                                       (2,1),(2,2),(2,3),(2,4),
                                                       (3,1),(3,2),(3,3),(3,4)};
  std::vector<pair<int,int>> m_op_link_interface_second = {(4,4),(5,4),(12,4),(13,4),
                                                       (20,4),(21,4),(28,4),(29,4),
                                                       (7,3),(15,3),(23,3),(31,3),
                                                       (6,4),(14,4),(22,4),(30,4),
                                                       (7,4),(15,4),(23,4),(31,4)};

  std::vector<pair<int,int>> m_obs_link_interface_first = {(6,1),(6,2),(7,1),(7,2),(14,1)(14,2),(15,1),(15,2),
                                                           (22,1),(22,2),(23,1),(23,2),(30,1),(30,2),(31,1),(31,2),
                                                           (0,1),(0,2),(0,3),(0,4),(1,1),(1,2),(1,3),(1,4),
                                                           (2,1),(2,2),(2,3),(2,4),(3,1),(3,2),(3,3),(3,4)};
  std::vector<pair<int,int>> m_obs_link_interface_second ={(4,3),(4,4),(5,3),(5,4),(12,3)(12,4),(13,3),(13,4),
                                                          (20,3),(20,4),(21,3),(21,4),(28,3),(28,4),(29,3),(29,4),
                                                          (6,3),(14,3),(22,3),(30,3),(7,3),(15,3),(23,3),(31,3),
                                                          (6,4),(14,4),(22,4),(30,4),(7,4),(15,4),(23,4),(31,4)};
};

} // namespace ns3

#endif /* MO_ENV_H */