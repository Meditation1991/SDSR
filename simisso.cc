#include<stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*

#include "simisso.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("simisso");

VanetSim::VanetSim()
{
	phyMode = "OfdmRate9MbpsBW10MHz";
	lossModle = "ns3::FriisPropagationLossModel";

	freq = 5.9e9;  //for 802.11p
	txp = 20;  // dBm
	
	mod = 0;
	duration = 0;
	nodeNum = 0;//cars
	m_sinks=10;
	m_sources=10;

	Rx1_Data_Bytes = 0;
	Rx1_Data_Pkts = 0;
	
	Tx1_Data_Bytes = 0;
	Tx1_Data_Pkts = 0;
	
	control_packets = 0;


	m_port = 65419;

	homepath = ".";//getenv("HOME");
	folder="SimMap";

	packetSize = 64;
	rate = "0.256kbps";
}

VanetSim::~VanetSim()
{
	os.close();
}

void VanetSim::Simulate(int argc, char *argv[])
{
	SetDefault();
	ParseArguments(argc, argv);
	LoadTraffic();
	ConfigNode();
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	Run(); 
	ProcessOutputs();
	std::cout<<std::endl;
}

void VanetSim::SetDefault()
{
	//Handle By Constructor
}



void VanetSim::ParseArguments(int argc, char *argv[])
{
	CommandLine cmd;

	cmd.AddValue ("duration", "Duration of Simulation", duration);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp", "TX power", txp);
	cmd.AddValue ("mod", "0=aodv 1=olsr 2=dsdv 3=dsr", mod);

	//cmd.AddValue ("ds", "DataSet", m_ds);
	cmd.Parse (argc,argv);

	
	SeedManager::SetSeed (5);
	SeedManager::SetRun (1);

}

void VanetSim::LoadTraffic()
{
	switch (mod)
	{

	  case 0:
		std::cout<<"Mode: AODV"<<std::endl;
		m_todo = "AODV" ;
	  break;

	  case 1:
		std::cout<<"Mode: OLSR-N"<<std::endl;
		m_todo = "OLSR";
	  break;

	  case 2:
		std::cout<<"Mode: DSDV"<<std::endl;
		m_todo = "DSDV" ;
	  break;

	  case 3:
		std::cout<<"Mode: DSR"<<std::endl;
		m_todo = "DSR";
	  break;

	  default:
		std::cout<<"Mode:AODV"<<std::endl;
		m_todo ="AODV";
	  break;
	}


	DIR* dir = NULL;
	
	std::string temp(homepath+"/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");


	std::string sumo_net = temp + "/input.net.xml";
	std::string sumo_fcd = temp + "/input.fcd.xml";
	std::string sumo_route = temp + "/input.rou.xml";

	std::string output = temp + "/" + m_todo + "_" + m_ds + "_result_new.txt";

	os.open(output.data(),std::ios::out);

	ns3::vanetmobility::VANETmobilityHelper mobilityHelper;
	VMo=mobilityHelper.GetSumoMObility(sumo_net,sumo_route,sumo_fcd);

	nodeNum = VMo->GetNodeSize();
	std::cout<<"nodeNum"<<nodeNum<<std::endl;
	os<<"Mode:  "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
}



void VanetSim::ConfigNode()
{
	NS_LOG_INFO ("creating the nodes");
	m_nodes.Create(nodeNum);//Cars


}

void VanetSim::ConfigChannels()
{
	//===channel

	YansWifiChannelHelper Channel;
	Channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	Channel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq));
	

	// the channelg
	Ptr<YansWifiChannel> CH = Channel.Create();

	//===wifiphy
	YansWifiPhyHelper CHPhy =  YansWifiPhyHelper::Default ();
	CHPhy.SetChannel (CH);
	CHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

	// 802.11p mac
	NqosWaveMacHelper CH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CH80211p = Wifi80211pHelper::Default ();

	CH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue (phyMode), "ControlMode",StringValue (phyMode));

	// Set Tx Power
	CHPhy.Set ("TxPowerStart",DoubleValue (txp));
	CHPhy.Set ("TxPowerEnd", DoubleValue (txp));
	m_TxDevices = CH80211p.Install(CHPhy, CH80211pMac, m_nodes);


	std::cout<<"ConfigChannels Done"<<std::endl;

}

void VanetSim::ConfigDevices()
{
	//Done in ConfigChannels()s
}

void VanetSim::ConfigMobility()
{

	VMo->Install();
	duration = VMo->GetReadTotalTime();
	
	Time temp_now = Simulator::Now();


	std::cout<<"ConfigMobility Done"<<std::endl;
}




void VanetSim::ConfigApp()
{
	//===Routing protocol
	InternetStackHelper internet;
	OlsrHelper olsr;
	AodvHelper aodv;
	DsdvHelper dsdv;
	DsrHelper dsr;
	DsrMainHelper dsrMain;

	switch (mod)
	  {
      case 0:
        

	internet.SetRoutingHelper(aodv);
        internet.Install (m_nodes);
        std::cout<<"AODV"<<std::endl;
        os<<"AODV"<<std::endl;

        break;
      case 1:

        internet.SetRoutingHelper(olsr);
        internet.Install (m_nodes);
        std::cout<<"OLSR"<<std::endl;
        os<<"OLSR"<<std::endl;

        break;
      case 2:

        internet.SetRoutingHelper(dsdv);
        internet.Install (m_nodes);
        std::cout<<"DSDV"<<std::endl;
        os<<"DSDV"<<std::endl;
        break;

      case 3:
        internet.Install (m_nodes);
        dsrMain.Install (dsr, m_nodes);
        std::cout<<"DSR"<<std::endl;
        os<<"DSR"<<std::endl;
        break;
      default:

	internet.SetRoutingHelper(aodv);
        internet.Install (m_nodes);
        std::cout<<"AODV"<<std::endl;
        os<<"AODV"<<std::endl;
        break;

	  }

	std::cout<<"internet.Install Done"<<std::endl;

	NS_LOG_INFO ("assigning ip address");


	//===assign IP ADDRESS

	Ipv4AddressHelper ipv4;
	ipv4.SetBase ("10.1.0.0", "255.255.0.0");
	m_TxInterfaces = ipv4.Assign (m_TxDevices);
	std::cout<<"IPV4S Assigned"<<std::endl;
	


	//Setup routing transmission(same with Vanet-routing-compare.cc)
	OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address ());
	onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	onoff1.SetAttribute ("PacketSize", UintegerValue (packetSize));
	onoff1.SetAttribute ("DataRate", DataRateValue (DataRate (rate)));


	Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
	int64_t stream = 2;
	var->SetStream (stream);
	//mesh
	for (int i = 0; i < m_sources; i++)
 	  for (int j = 0; j < m_sinks; ++j)
		if (i != j)
		{
		  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		  Ipv4Address AddressJ = m_TxInterfaces.GetAddress (j);
		  Ptr<Node> NodeJ = m_nodes.Get (j);
		  Ptr<Socket> sink = Socket::CreateSocket (NodeJ, tid);
		  InetSocketAddress local = InetSocketAddress (AddressJ, m_port);
		  sink->Bind (local);
		  sink->SetRecvCallback (MakeCallback (&VanetSim::ReceiveDataPacket1, this));

		  AddressValue remoteAddress (InetSocketAddress (AddressJ, m_port));
		  onoff1.SetAttribute ("Remote", remoteAddress);

		  ApplicationContainer app = onoff1.Install (m_nodes.Get (i));
		  app.Start (Seconds (var->GetValue (1.0, 2.0)));
		  app.Stop (Seconds (duration)); //i send message
			
		 

	}

	 // Connect the tracers
	
	std::string temp1(homepath+"/"+folder);
	std::string Tx_output = temp1 + "/" +  "_TxPackets.txt";
	os1.open(Tx_output.data(),std::ios::out);
	std::string TxPath = "/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx"; 
	 Config::ConnectWithoutContext (TxPath,MakeCallback(&VanetSim::ConfigTracing, this));

	

	 std::string IpTxPath = "/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx"; 
	 Config::ConnectWithoutContext (IpTxPath,MakeCallback(&VanetSim::TXTrace1, this));


	std::cout<<"ConfigApp Done"<<std::endl;
	
}

void VanetSim::ReceiveDataPacket1(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	Address srcAddress;
	while ((packet = socket->RecvFrom (srcAddress)))
	{
		Rx1_Data_Bytes += packet->GetSize();
		Rx1_Data_Pkts++;
		//std::cout<<"."<<std::endl;

		Time now = Simulator::Now ();
		uint64_t uid = packet->GetUid ();
		int64_t temp = now.GetMilliSeconds() - delay[uid].GetMilliSeconds();
		delay_vector.push_back (temp);
	         
	}
}


void VanetSim::SendDataPacket()
{

	//done in ConfigApp
}

void VanetSim::ConfigTracing(Ptr<const Packet> txpacket)
{
	 

os1 <<"Now:  "<<Simulator::Now().GetSeconds()
  	<<"Tx_Data_Pkts:   "<<Tx1_Data_Pkts <<std::endl;


}

void VanetSim::ProcessOutputs()
{
	std::cout<<"send:"<<Tx1_Data_Pkts<<std::endl;
	std::cout<<"recv:"<<Rx1_Data_Pkts <<std::endl;

	os<<"send:   "<<Tx1_Data_Pkts<<std::endl;
	os<<"recv:  "<<Rx1_Data_Pkts <<std::endl;
	
        if (!delay_vector.empty ())
	{
              int64_t best = delay_vector[0],
              worst = delay_vector[0];
              double avg = 0;
              for (std::vector<int64_t>::const_iterator cit = delay_vector.begin ();
              cit != delay_vector.end ();++cit)
              {
                if (*cit<best)
                {
                  best = *cit;
                }

                if (*cit>worst)
                {
                  worst = *cit;
                }
                avg += *cit;
              }

              avg /= delay_vector.size();
              std::cout<<"Best delay:   "<<best<<"ms"<<std::endl;
              std::cout<<"Worst delay:   "<<worst<<"ms"<<std::endl;
              std::cout<<"Avg delay: "<<avg<<"ms"<<std::endl;
              os<<"Best delay:   "<<best<<"ms"<<std::endl;
              os<<"Worst delay:   "<<worst<<"ms"<<std::endl;
              os<<"Avg delay: "<<avg<<"ms"<<std::endl;
	}
}

void VanetSim::Run()
{

	NS_LOG_INFO ("Run Simulation.");

	Simulator::Schedule(Seconds(0.0), &VanetSim::Look_at_clock, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	os << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();

}

void VanetSim::Look_at_clock()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	std::cout<<"Mode:"<<m_todo<<", Dataset:"<<m_ds<<std::endl;
	std::cout<<"Tx_Data_Pkts:"<<Tx1_Data_Pkts<<std::endl;
	std::cout<<"Rx_Data_Pkts:"<<Rx1_Data_Pkts<<std::endl;

	os<<"Now:  "<<Simulator::Now().GetSeconds()
  	<<"Tx_Data_Pkts:   "<<Tx1_Data_Pkts
  	<<"Rx_Data_Pkts:   "<<Rx1_Data_Pkts
	<<"Control PKts:   "<<control_packets <<std::endl;



	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock, this);
}

void
VanetSim::TXTrace1 (Ptr<const Packet> newpacket)
{

 Ptr<Packet> packet = newpacket->CreateFragment(0, newpacket->GetSize());
  
  Ipv4Header ipheader;
  UdpHeader udpheader;
  
  packet->RemoveHeader(ipheader);
  packet->RemoveHeader(udpheader);
  
  if (udpheader.GetSourcePort() != 49192) {
    
    control_packets++;
    
  }


  Tx1_Data_Pkts++;
  Tx1_Data_Bytes += newpacket->GetSize ();



  Time now = Simulator::Now ();
  delay[newpacket->GetUid ()] = now;

}



int main (int argc, char *argv[])
{     
	VanetSim Simi;
	Simi.Simulate(argc, argv);
	return 0;
}



