//======================================================================
/*! \file IbeoSdkEcuLiveDemo.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Jun 1, 2012
 *
 * Demo project for connecting to an ECU and process the received
 * data blocks.
 *///-------------------------------------------------------------------

#include <ibeosdk/scala.hpp>

#include <ibeosdk/ecu.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <iostream>
#include <cstdlib>

//#include <hiredis.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>


#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub, pub_top, pub_bottom, pub_point_filtered;
std::ifstream g_cfg_ifs("default-convert.yaml");
PM::DataPointsFilters g_dpf(g_cfg_ifs);

//======================================================================

using namespace ibeo;

//======================================================================

const ibeo::Version::MajorVersion majorVersion(2);
const ibeo::Version::MinorVersion minorVersion(11);
const ibeo::Version::Revision revision = ibeo::Version::Revision(1);
const ibeo::Version::PatchLevel patchLevel;
const ibeo::Version::Build build;
const std::string info = "Demo";

ibeo::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);
IbeoSDK ibeoSDK;

//======================================================================

void live_demo(LogFileManager& logFileManager, std::string ip);

//======================================================================

TimeConversion tc;

//======================================================================

class AllEcuListener : public ibeo::DataListener<ibeo::ScanEcu>,
                       public ibeo::DataListener<ObjectListEcu>,
                       public ibeo::DataListener<ObjectListEcuEt>,
                       public ibeo::DataListener<Image>,
                       public ibeo::DataListener<PositionWgs84>,
                       public ibeo::DataListener<VehicleStateBasicEcu2806>,
                       public ibeo::DataListener<VehicleStateBasicEcu>,
                       public ibeo::DataListener<MeasurementList2821>,
                       public ibeo::DataListener<DeviceStatus>,
                       public ibeo::DataListener<DeviceStatus6303>,
                       public ibeo::DataListener<LogMessageError>,
                       public ibeo::DataListener<LogMessageDebug>,
                       public ibeo::DataListener<LogMessageNote>,
                       public ibeo::DataListener<LogMessageWarning>
{
public:
	virtual ~AllEcuListener() {}

public:
	//========================================
	virtual void onData(const ScanEcu* const scan)
	{
		logInfo << "Scan received: # " << scan->getScanNumber()
			<<"  time: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
			<< std::endl;

		static bool _first = true;
		static VPointCloud::Ptr point_cloud_total(new VPointCloud());
		//ros::Rate r(32);r.sleep();
		//ros::Rate r(15);r.sleep();
		//ros::Rate r(5);r.sleep();
		VPointCloud::Ptr outMsg(new VPointCloud());
		outMsg->header.stamp = scan->getStartTimestamp().getTime();//ros::Time::now();
		//outMsg->header.stamp = ros::Time::now();
		outMsg->header.frame_id = "ibeo";//"filtered_velodyne";
		outMsg->height = 1;
		if (_first){
		  //point_cloud_total =  new VPointCloud();
		  point_cloud_total->header.stamp = scan->getStartTimestamp().getTime();//ros::Time::now();
		  //point_cloud_total->header.stamp = ros::Time::now();
		  point_cloud_total->header.frame_id = "ibeo";//"filtered_velodyne";
		  point_cloud_total->height = 1;
		  point_cloud_total->points.clear();
		  _first = false;

		}

		bool top = false;
		for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) 
		{
		  if (scan->getScanPoints()[_ii].getFlags() & ScanPointEcu::ESPF_MaskInvalid) continue;
		  if (scan->getScanPoints()[_ii].getFlags() & ScanPointEcu::ESPF_Transparent) continue;
		  if (scan->getScanPoints()[_ii].getEcho() > 0 ) continue;
		  velodyne_pointcloud::PointXYZIR _point_new;
		  _point_new.ring = 0;
		  _point_new.intensity = scan->getScanPoints()[_ii].getLayer();//0;
		  _point_new.x    = scan->getScanPoints()[_ii].getPositionX();
		  _point_new.y    = scan->getScanPoints()[_ii].getPositionY();
		  //_point_new.z    = scan->getScanPoints()[_ii].getPositionZ();
		  _point_new.z    = 0 ;//scan->getScanPoints()[_ii].getPositionZ();
		  if (scan->getScanPoints()[_ii].getLayer() >3) 
		    //{}
		  {top = true;}//break;}
		  //else break;
		  outMsg->push_back(_point_new);
		}
		if (top)
		{
		  //point_cloud_total->points.insert(point_cloud_total->points.end(), outMsg->points.begin(), outMsg->points.end());
		  for (int _j = 0; _j < outMsg->size(); _j++)
		  {
		    velodyne_pointcloud::PointXYZIR _point_new;
		    _point_new.ring = 0;
		    _point_new.intensity = outMsg->points[_j].intensity;//0;
		    _point_new.x    = outMsg->points[_j].x;
		    _point_new.y    = outMsg->points[_j].y;
		    _point_new.z    = outMsg->points[_j].z;
		    point_cloud_total->push_back(_point_new);
		  }
		  pub_top.publish(outMsg);


		  point_cloud_total->header.stamp = scan->getStartTimestamp().getTime();//ros::Time::now();
		  //point_cloud_total->header.stamp = ros::Time::now();
		  point_cloud_total->header.frame_id = "ibeo";//"filtered_velodyne";
		  point_cloud_total->height = 1;
		  pub.publish(point_cloud_total);
		  sensor_msgs::PointCloud2 point_cloud_total_pcl;//(new PointCloud());
		  pcl::toROSMsg(*point_cloud_total, point_cloud_total_pcl);
		  DP mapPointCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(point_cloud_total_pcl));
		  g_dpf.apply(mapPointCloud);
		  pub_point_filtered.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, "ibeo", ros::Time::now()));
		  std::cout << mapPointCloud.getNbPoints() << std::endl;

		}
		else{
		  //point_cloud_total =  new VPointCloud();
		  point_cloud_total->header.stamp = scan->getStartTimestamp().getTime();//ros::Time::now();
		  point_cloud_total->header.frame_id = "ibeo";//"filtered_velodyne";
		  point_cloud_total->height = 1;
		  point_cloud_total->points.clear();
		  //point_cloud_total->points.insert(point_cloud_total->points.end(), outMsg->points.begin(), outMsg->points.end());
		  ////for (int _j = 0; _j < point_cloud_total.points.size(); _j++)
		  for (int _j = 0; _j < outMsg->size(); _j++)
		  {
		    velodyne_pointcloud::PointXYZIR _point_new;
		    _point_new.ring = 0;
		    //_point_new.intensity = point_cloud_total.points[_j].intensity;//0;
		    //_point_new.x    = point_cloud_total.points[_j].x;
		    //_point_new.y    = point_cloud_total.points[_j].y;
		    //_point_new.z    = point_cloud_total.points[_j].z;
		    //outMsg->push_back(_point_new);
		    _point_new.intensity = outMsg->points[_j].intensity;//0;
		    _point_new.x    = outMsg->points[_j].x;
		    _point_new.y    = outMsg->points[_j].y;
		    _point_new.z    = outMsg->points[_j].z;
		    point_cloud_total->push_back(_point_new);
		  }

		  pub_bottom.publish(outMsg);
		  //pub.publish(outMsg);

		}

		ros::spinOnce();


	}

	//========================================
	virtual void onData(const ObjectListEcu* const objectList)
	{
		logInfo << "Objects received: # " << objectList->getNumberOfObjects() << std::endl;
	}

	//========================================
	virtual void onData(const ObjectListEcuEt* const objectList)
	{
		logInfo << "ET Objects received: # " << objectList->getNbOfObjects() << std::endl;
	}

	//========================================
	virtual void onData(const Image* const image)
	{
		logInfo << std::setw(5) << image->getSerializedSize() << " Bytes  " << "Image received: time: " << tc.toString(image->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const PositionWgs84* const wgs84)
	{
		logInfo << std::setw(5) << wgs84->getSerializedSize() << " Bytes  " << "PositionWGS84 received: time: " << tc.toString(wgs84->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	virtual void onData(const VehicleStateBasicEcu2806* const vsb)
	{
		logInfo << "VSB (0x2806) " << tc.toString(vsb->getTimestamp().toPtime(), 3) << std::endl;
	}

	//========================================
	virtual void onData(const VehicleStateBasicEcu* const vsb)
	{
		logInfo << "VSB " << tc.toString(vsb->getTimestamp().toPtime(), 3) << std::endl;
	}

	//========================================
	void onData(const MeasurementList2821* const ml)
	{
		logInfo << std::setw(5) << ml->getSerializedSize() << " Bytes  "
				<< "MeasurementList received: time: " << tc.toString(ml->getTimestamp().toPtime())
				<< " LN: '" << ml->getListName() << "' GN: '" << ml->getGroupName() << "'"
				<< std::endl;
	}

	//========================================
	virtual void onData(const DeviceStatus* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  " << "DevStat received" << std::endl;
	}

	//========================================
	virtual void onData(const DeviceStatus6303* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  " << "DevStat 0x6303 received" << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageError* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageWarning* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageNote* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageDebug* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}
}; // AllEcuListener

//======================================================================
//======================================================================
//======================================================================

int checkArguments(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
		std::cerr << "\tIP is the ip address of the Ibeo Ecu, e.g. 192.168.0.1." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

int main(const int argc, const char** argv)
{
	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;


	int _argc;
	char ** _argv;
	ros::init(_argc, _argv, "publish_ibeo_from_ecu");
	ros::NodeHandle nh;//("~");
	pub = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points", 1);
	pub_top = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_top", 1);
	pub_bottom = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_bottom", 1);
	pub_point_filtered = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_filtered", 1);

	bool hasLogFile;
	const int checkResult = checkArguments(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string ip = argv[currArg++];

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeo::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeo::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeo::LogLevel ll = ibeo::logLevelFromString("Debug");
	ibeo::LogFile::setLogLevel(ll);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersion.toString()
		        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	}

	live_demo(logFileManager, ip);

	exit(0);
}

//======================================================================

void live_demo(LogFileManager& logFileManager, std::string ip)
{
	AllEcuListener allEcuListener;

	const uint16_t port = getPort(ip, 12004);//12002
	IbeoEcu ecu(ip, port);
	ecu.setLogFileManager(&logFileManager);

	ecu.registerListener(dynamic_cast<DataListener<ScanEcu>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ObjectListEcu>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ObjectListEcuEt>*>(&allEcuListener));
	
	//ecu.registerListener(dynamic_cast<DataListener<FrameEndSeparator>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<Scan2208>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ObjectListScala>*>(&allEcuListener));
	//ecu.registerListener(dynamic_cast<DataListener<ObjectListScala2271>*>(&allEcuListener));
	
	/*
	ecu.registerListener(dynamic_cast<DataListener<Image>*>(&allEcuListener));
	
	ecu.registerListener(dynamic_cast<DataListener<PositionWgs84>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu2806>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<MeasurementList2821>*>(&allEcuListener));
	/*
	ecu.registerListener(dynamic_cast<DataListener<DeviceStatus>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<LogMessageError>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<LogMessageDebug>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<LogMessageNote>*>(&allEcuListener));
	ecu.registerListener(dynamic_cast<DataListener<LogMessageWarning>*>(&allEcuListener));
	*/ 
	ecu.getConnected();

	// Just to keep the program alive
	while (true) {
		if (!ecu.isConnected())
			return;
#		ifdef _WIN32
			::Sleep(1);
#		else // _WIN32
			sleep(1);
#		endif // _WIN32
	}
}

//======================================================================
