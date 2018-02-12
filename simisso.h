
#ifndef SIMISSO_H
#define SIMISSO_H

#include <string>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"


#include "ns3/vanetmobility-helper.h"

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "ns3/trace-source-accessor.h"

using namespace ns3;

class VanetSim
{
public:
	VanetSim();
	~VanetSim();
	void Simulate(int argc, char *argv[]);
protected:
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void ConfigNode();
	void ConfigChannels();
	void ConfigDevices();
	void ConfigMobility();
	void ConfigApp();
	void ConfigTracing(Ptr<const Packet> txpacket);
	void Run();
	void ProcessOutputs();
	bool CheckActive(Node node);
	void Look_at_clock();
	
private:
	Ptr<Socket> source;

	std::string phyMode;
	std::string lossModle;

	std::string homepath;
	std::string folder;
	std::ofstream os;
	
	std::ofstream os1;

 
	std::string Tx_output;

	double freq;
	double txp;
	
	
	
	int mod;//0=aodv 1=olsr 2=dsdv 3=dsr

	uint32_t nodeNum;
	double duration;
	int m_sinks;
	int m_sources;

	NodeContainer m_nodes;//Cars + Source + Sink

	NetDeviceContainer m_TxDevices;

	Ipv4InterfaceContainer m_TxInterfaces;

	ApplicationContainer app;

	
	uint32_t Rx1_Data_Bytes, Tx1_Data_Bytes;
	uint32_t Rx1_Data_Pkts, Tx1_Data_Pkts;

	uint64_t control_packets;

	uint32_t m_port;

	//ApplicationContainer m_source1, m_sink1;

	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPacket1 (Ptr<Socket> socket);
	void SendDataPacket ();
	void TXTrace1 (Ptr<const Packet> newpacket);


	std::unordered_map<uint64_t, Time> delay;
	std::vector<int64_t> delay_vector;

	
	std::string m_todo;
  	std::string m_ds;//DataSet

	uint32_t packetSize;
	std::string rate;
  	
};




#endif 
