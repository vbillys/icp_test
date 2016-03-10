//======================================================================
/*! \file IbeoSdkFileDemo.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Jun 1, 2012
 *
 * Demo project for reading IDC files and process the data blocks.
 *///-------------------------------------------------------------------

#include <ibeosdk/lux.hpp>
#include <ibeosdk/ecu.hpp>
#include <ibeosdk/minilux.hpp>
#include <ibeosdk/scala.hpp>

#include <ibeosdk/devices/IdcFile.hpp>

#include <iostream>
#include <cstdlib>

#include <CkCsv.h>


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
const std::string info = "IbeoSdkFileDemo";

ibeo::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

IbeoSDK ibeoSDK;

//CkCsv csv_save;
int gRowIndex_csv_save = 0;
int gDataScanCounter =0;

//======================================================================

void file_demo(const std::string& filename);

//======================================================================

TimeConversion tc;

//======================================================================

//VPointCloud::Ptr point_cloud_total=NULL;
//VPointCloud::Ptr point_cloud_total;

class AllListener : public ibeo::DataListener<FrameEndSeparator>,
                    public ibeo::DataListener<ScanLux>,
                    public ibeo::DataListener<ScanEcu>,
                    public ibeo::DataListener<Scan2208>,
                    public ibeo::DataListener<ObjectListLux>,
                    public ibeo::DataListener<ObjectListEcu>,
                    public ibeo::DataListener<ObjectListScala>,
                    public ibeo::DataListener<ObjectListScala2271>,
                    public ibeo::DataListener<ObjectListEcuEt>,
                    public ibeo::DataListener<Image>,
                    public ibeo::DataListener<PositionWgs84>,
                    public ibeo::DataListener<MeasurementList2821>,
                    public ibeo::DataListener<VehicleStateBasicLux>,
                    public ibeo::DataListener<VehicleStateBasicEcu2806>,
                    public ibeo::DataListener<VehicleStateBasicEcu>,
                    public ibeo::DataListener<DeviceStatus>,
                    public ibeo::DataListener<DeviceStatus6303>,
                    public ibeo::DataListener<LogMessageError>,
                    public ibeo::DataListener<LogMessageWarning>,
                    public ibeo::DataListener<LogMessageNote>,
                    public ibeo::DataListener<LogMessageDebug>
{
public:
	//========================================
	void onData(const FrameEndSeparator* const fes)
	{
		logInfo << std::setw(5) << fes->getSerializedSize() << " Bytes  "
				<< "Frame received: # " << fes->getFrameId()
				<< "  Frame time: " << tc.toString(fes->getHeaderNtpTime().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const ScanLux* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "ScanLux received: # " << scan->getScanNumber()
				<< "  ScanStart: " << tc.toString(scan->getStartTimestamp().toPtime())
				<< std::endl;
	}
	//static VPointCloud::Ptr point_cloud_total=NULL;
	//========================================
	void onData(const ScanEcu* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "ScanEcu received: # " << scan->getScanNumber()
				<< "  #Pts: " << scan->getNumberOfScanPoints()
				<< "  ScanStart: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
				<< std::endl;
	    //usleep(35000); //std::cout << scan->getEndTimeOffset()<< std::endl;
	    //if (scan->getScanNumber() % 2) 
	    //{}
	      ////return; // top layers only
	    //else return; // bottom layers only
	    //if (!(scan->getScanNumber() % 2)) return;
	    static bool _first = true;
	    static VPointCloud::Ptr point_cloud_total(new VPointCloud());
	    ros::Rate r(32);r.sleep();
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
		//printf(" No. of ScannerInfos: %d No. of ScanPoints: %d\n" , scan->getNumberOfScannerInfos() , scan->getNumberOfScanPoints());
		//printf(" Device IDs: ");
		//for (int _ii = 0; _ii < scan->getNumberOfScannerInfos() ; _ii++) printf(" %d %d", scan->getScannerInfos()[_ii].getDeviceId(), scan->getScannerInfos()[_ii].getFlags());
		//printf("\n)");
		//printf(" Device Offsets: ");
		//for (int _ii = 0; _ii < scan->getNumberOfScannerInfos() ; _ii++) printf(" %.2f %.2f %.2f", scan->getScannerInfos()[_ii].getOffsetX()
																								 //, scan->getScannerInfos()[_ii].getOffsetY()
																								 //, scan->getScannerInfos()[_ii].getOffsetZ()
																			   //);
	    //for (int _ii = 0; _ii < scan->getNumberOfScannerInfos() ; _ii++) printf(" %.2f %.2f %.2f", scan->getScannerInfos()[_ii].getYawAngle()
																								 //, scan->getScannerInfos()[_ii].getPitchAngle()
																								 //, scan->getScannerInfos()[_ii].getRollAngle()
																			   //);
        //for (int _ii = 0; _ii < scan->getNumberOfScannerInfos() ; _ii++) printf(" %.2f %.2f %.3f %.3f"     , scan->getScannerInfos()[_ii].getStartAngle()
																								 //, scan->getScannerInfos()[_ii].getEndAngle()
																								 //, scan->getScannerInfos()[_ii].getBeamTilt()
																								 //, scan->getScannerInfos()[_ii].getFrequency()
																			   //);
																			   


        //for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %d %.3f", scan->getScanPoints()[_ii].getLayer(), scan->getScanPoints()[_ii].getPositionZ());
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

        //if (scan->getScanPoints()[_ii].getLayer() == 0)
          //printf(" %d %.3f %.3f %.3f", scan->getScanPoints()[_ii].getLayer()
          //, scan->getScanPoints()[_ii].getPositionX()
          //, scan->getScanPoints()[_ii].getPositionY()
          //, scan->getScanPoints()[_ii].getPositionZ()
          //);

		/*float deviceOffsets[6][3];
		for (int _ii = 0; _ii < scan->getNumberOfScannerInfos() ; _ii++){
			deviceOffsets[scan->getScannerInfos()[_ii].getDeviceId()-1][0] =  scan->getScannerInfos()[_ii].getOffsetX();
			deviceOffsets[scan->getScannerInfos()[_ii].getDeviceId()-1][1] =  scan->getScannerInfos()[_ii].getOffsetY();
			deviceOffsets[scan->getScannerInfos()[_ii].getDeviceId()-1][2] =  scan->getScannerInfos()[_ii].getOffsetZ();
		}

		printf("\n)");
		//printf(" Point's device IDs: ");
		//printf(" Point's Z coord's: ");
		printf(" Ground Point's Z coord's: ");
		
		//printf(" Point's echo widths: ");
		//printf(" Point's echo numbers: ");
		//printf(" Point's flags: ");
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %d", scan->getScanPoints()[_ii].getDeviceId());
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %.3f", scan->getScanPoints()[_ii].getPositionZ());
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) if (scan->getScanPoints()[_ii].getFlags() == 1)printf(" %.3f", scan->getScanPoints()[_ii].getPositionZ());
		for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) if (scan->getScanPoints()[_ii].getFlags() == 1)printf(" %d", scan->getScanPoints()[_ii].getEcho());
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %d", scan->getScanPoints()[_ii].getEcho());
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %.3f", scan->getScanPoints()[_ii].getEchoPulseWidth());
		//for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++) printf(" %d", scan->getScanPoints()[_ii].getFlags());
		printf("\n)");

		char _tmpStr[84] = "";

		if (gDataScanCounter == 1){

		for (int _ii = 0; _ii < scan->getNumberOfScanPoints() ; _ii++){
			
			//if (scan->getScanPoints()[_ii].getFlags() != 1) continue;

			sprintf(_tmpStr,"%.3f",deviceOffsets[scan->getScanPoints()[_ii].getDeviceId()-1][0]);
			std::string _offset_X = std::string(_tmpStr);
			sprintf(_tmpStr,"%.3f",deviceOffsets[scan->getScanPoints()[_ii].getDeviceId()-1][1]);
			std::string _offset_Y = std::string(_tmpStr);
			sprintf(_tmpStr,"%.3f",deviceOffsets[scan->getScanPoints()[_ii].getDeviceId()-1][2]);
			std::string _offset_Z = std::string(_tmpStr);
			
			sprintf(_tmpStr,"%.3f",scan->getScanPoints()[_ii].getPositionX());
			std::string _data_X = std::string(_tmpStr);
			sprintf(_tmpStr,"%.3f",scan->getScanPoints()[_ii].getPositionY());
			std::string _data_Y = std::string(_tmpStr);
			sprintf(_tmpStr,"%.3f",scan->getScanPoints()[_ii].getPositionZ());
			std::string _data_Z = std::string(_tmpStr);

			sprintf(_tmpStr,"%d",scan->getScanPoints()[_ii].getLayer());
			std::string _data_layer = std::string(_tmpStr);
			sprintf(_tmpStr,"%d",scan->getScanPoints()[_ii].getFlags());
			std::string _data_flag = std::string(_tmpStr);

			csv_save.SetCell(gRowIndex_csv_save,0,_offset_X.c_str());
            csv_save.SetCell(gRowIndex_csv_save,1,_offset_Y.c_str());
            csv_save.SetCell(gRowIndex_csv_save,2,_offset_Z.c_str());
            csv_save.SetCell(gRowIndex_csv_save,3,_data_X.c_str());
            csv_save.SetCell(gRowIndex_csv_save,4,_data_Y.c_str());
            csv_save.SetCell(gRowIndex_csv_save,5,_data_Z.c_str());
			csv_save.SetCell(gRowIndex_csv_save,6,_data_layer.c_str());
			csv_save.SetCell(gRowIndex_csv_save,7,_data_flag.c_str());

			gRowIndex_csv_save++;
		}
		}*/
		gDataScanCounter++;
	}

	//========================================
	void onData(const Scan2208* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "Scan2208 received: # " << scan->getScanNumber()
				<< "  #Pts: " << scan->getSubScans().at(0).getNbOfPoints()
				<< "  ScanStart: " << tc.toString(scan->getSubScans().at(0).getStartScanTimestamp().toPtime(), 3)
				<< std::endl;
	}

	//========================================
	void onData(const ObjectListLux* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListLux received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListEcu* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListEcu received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListScala* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListScala received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListScala2271* const objs)
	{
		logInfo << std::setw(5) << objs->getSerializedSize() << " Bytes  "
				<< "ObjectList 2271 received. Scan: " << objs->getScanNumber()
				<< "  ObjLstId: " << int(objs->getObjectListId())
				<< "  #Obj:" << objs->getNumberOfObjects()
				<< std::endl;
	}

	//========================================
	void onData(const ObjectListEcuEt* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListEcUEts received: # " << objList->getNbOfObjects() << std::endl;
	}

	//========================================
	void onData(const Image* const image)
	{
		logInfo << std::setw(5) << image->getSerializedSize() << " Bytes  " << "Image received: time: " << tc.toString(image->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const PositionWgs84* const wgs84)
	{
		logInfo << std::setw(5) << wgs84->getSerializedSize() << " Bytes  " << "PositionWGS84 received: time: " << tc.toString(wgs84->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicLux* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  " << "VSB (LUX) received: time: " << tc.toString(vsb->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicEcu2806* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				<< "VSB (ECU;old) received: time: " << tc.toString(vsb->getTimestamp().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicEcu* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				<< "VSB (ECU) received: time: " << tc.toString(vsb->getTimestamp().toPtime())
				<< std::endl;
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
	void onData(const DeviceStatus* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  "
				<< "DevStat received"
				<< std::endl;
	}

	//========================================
	void onData(const DeviceStatus6303* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  "
				<< "DevStat 0x6303 received"
				<< std::endl;
	}

	//========================================
	void onData(const LogMessageError* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageWarning* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageNote* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageDebug* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================

}; // AllListener

//======================================================================
//======================================================================
//======================================================================

class CustomLogStreamCallbackExample : public CustomLogStreamCallback {
public:
	virtual ~CustomLogStreamCallbackExample() {}
public:
	virtual void onLineEnd(const char* const s, const int)
	{
		std::cerr << s << std::endl;
	}
}; // CustomLogStreamCallback


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
		std::cerr << argv[0] << " " << " INPUTFILENAME [LOGFILE]" << std::endl;
		std::cerr << "\tINPUTFILENAME Name of the file to use as input instead of a sensor." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

int main(const int argc, const char** argv)
{

	int _argc;
	char ** _argv;
	ros::init(_argc, _argv, "publish_ibeo_from_file");
	ros::NodeHandle nh;//("~");
	pub = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points", 1);
	pub_top = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_top", 1);
	pub_bottom = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_bottom", 1);
	pub_point_filtered = nh.advertise<sensor_msgs::PointCloud2>( "ibeo_points_filtered", 1);

	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	bool hasLogFile;
	const int checkResult = checkArguments(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string filename = argv[currArg++];

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeo::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeo::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeo::LogLevel ll = ibeo::logLevelFromString("Debug");
	ibeo::LogFile::setLogLevel(ll);

	static CustomLogStreamCallbackExample clsce;

	LogFile::setCustomLogStreamCallback(&clsce);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersion.toString()
		        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	}

	file_demo(filename);

	exit(0);
}

//======================================================================

void file_demo(const std::string& filename)
{

	std::string _filename = std::string("ibeo_all.csv");

	IdcFile file;
	file.open(filename);
	if (file.isOpen()) {
		AllListener allListener;
		//file.registerListener(dynamic_cast<DataListener<Scan2208>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<DeviceStatus>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<ObjectListScala2271>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<ObjectListScala>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ScanEcu>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu2806>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu>*>(&allListener));
		//file.registerListener(dynamic_cast<DataListener<ScanEcu>*>(&allListener));
		/*
		file.registerListener(dynamic_cast<DataListener<FrameEndSeparator>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ScanEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<Scan2208>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListScala>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListScala2271>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListEcuEt>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<Image>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<PositionWgs84>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu2806>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<MeasurementList2821>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<DeviceStatus>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageError>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageWarning>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageNote>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageDebug>*>(&allListener));
		//*/
		const DataBlock* db = NULL;
		unsigned short nbMessages = 0; // # of messages we parsed

		

		while (file.isGood()&& ros::ok()){// && gDataScanCounter <8) {
		//if (file.isGood()) {
			//sleep(1.0);
			db = file.getNextDataBlock();
			if (db == NULL) {
				ros::spinOnce();
				continue; // might be eof or unknown file type
			}
			else
			{
			ros::spinOnce();
			file.notifyListeners(db);
			ros::spinOnce();
			++nbMessages;
			}
		}

		logDebug << "EOF reached. OR interrupted... (by ROS)" << nbMessages << " known blocks found." << std::endl;
		logDebug << "No of Scan Data : " << gDataScanCounter << " known blocks found." << std::endl;
		//bool _success = csv_save.SaveFile(_filename.c_str()); std::cout << "create new "<<_success<<std::endl;
	}
	else {
		logError << "File not readable." << std::endl;
	}
}

//======================================================================
