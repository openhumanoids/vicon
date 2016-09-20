/*
 *
 * LCM vicon publisher
 *
 * origina author Brian J. Julian
 */

// includes
#include <lcm/lcm-cpp.hpp>
#include <sys/time.h>
#include <unistd.h>
#include <lcmtypes/vicon.hpp>
#include <iostream>

#include "data_stream_client.hpp"

// defaults
#define DEFAULT_MOCAP_HOST_ADDR "192.168.20.99"
//#define DEFAULT_MOCAP_HOST_ADDR "192.168.1.104"
#define DEFAULT_MOCAP_HOST_PORT "801"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

static int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// namespaces
using namespace ViconDataStreamSDK::CPP;

namespace {
std::string Adapt(const bool i_Value)
{
  return i_Value ? "True" : "False";
}

std::string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction) {
  case Direction::Forward:
    return "Forward";
  case Direction::Backward:
    return "Backward";
  case Direction::Left:
    return "Left";
  case Direction::Right:
    return "Right";
  case Direction::Up:
    return "Up";
  case Direction::Down:
    return "Down";
  default:
    return "Unknown";
  }
}

std::string Adapt(const DeviceType::Enum i_DeviceType)
{
  switch (i_DeviceType) {
  case DeviceType::ForcePlate:
    return "ForcePlate";
  case DeviceType::Unknown:
  default:
    return "Unknown";
  }
}

std::string Adapt(const Unit::Enum i_Unit)
{
  switch (i_Unit) {
  case Unit::Meter:
    return "Meter";
  case Unit::Volt:
    return "Volt";
  case Unit::NewtonMeter:
    return "NewtonMeter";
  case Unit::Newton:
    return "Newton";
  case Unit::Unknown:
  default:
    return "Unknown";
  }
}
}

vicon::model_t& findModel(std::vector<vicon::model_t>& models, std::string modelname)
{
    for(int i = 0; i < models.size(); i++)
    {
        if (models[i].name == modelname)
        {
            return models[i];
        }
    }
    vicon::model_t model;
    model.name = modelname;
    model.nummarkers = 0;
    model.numsegments = 0;
    models.push_back(model);
    return models.back();
}

vicon::marker_t& findMarker(std::vector<vicon::marker_t>& markers, std::string markername)
{
    for(int i = 0; i < markers.size(); i++)
    {
        if (markers[i].name == markername)
        {
            return markers[i];
        }
    }
    vicon::marker_t marker;
    marker.name = markername;
    markers.push_back(marker);
    return markers.back();
}

vicon::segment_t& findSegment(std::vector<vicon::segment_t>& segments, std::string segmentname)
{
    for(int i = 0; i < segments.size(); i++)
    {
        if (segments[i].name == segmentname)
        {
            return segments[i];
        }
    }
    vicon::segment_t segment;
    segment.name = segmentname;
    segments.push_back(segment);
    return segments.back();
}

vicon::xyz_t getXYZ(double vals[3])
{
    vicon::xyz_t xyz;
    xyz.x = vals[0];
    xyz.y = vals[1];
    xyz.z = vals[2];
    return xyz;
}

// class
class DataStreamClient {
public:

  // constructor
  DataStreamClient(lcm::LCM &lcm, std::string vicon_hostname);

  // destructor
  ~DataStreamClient();

  void run();
  void stop();

private:
  //lcm
  lcm::LCM &_lcm;

  // data stream client
  ViconDataStreamSDK::CPP::Client _vicon_client;

  // mocap tcp client
  std::string mocap_host_addr_;
  std::string mocap_host_port_;

};

// class constructor
DataStreamClient::DataStreamClient(lcm::LCM &lcm, std::string vicon_hostname) :
  _lcm(lcm)
{
  // local stack
  Output_GetAxisMapping axis_mapping;
  Output_GetVersion version_number;

  _vicon_client.Connect(vicon_hostname);
  if (!_vicon_client.IsConnected().Connected) {
    fprintf(stderr, "Error connecting to %s\n", vicon_hostname.c_str());
    exit(1);
  }

  // enable segment data
  _vicon_client.EnableSegmentData();
  if (!_vicon_client.IsSegmentDataEnabled().Enabled) {
    fprintf(stderr, "Error enabling segment data\n");
    _vicon_client.Disconnect();
    return;
  }

  // set streaming mode
  _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);
  // _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch);
  // _vicon_client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

  // set global axes
  _vicon_client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
  axis_mapping = _vicon_client.GetAxisMapping();
  dbg("Axis Mapping: X-%s Y-%s Z-%s\n", Adapt(axis_mapping.XAxis).c_str(), Adapt(axis_mapping.YAxis).c_str(), Adapt(axis_mapping.ZAxis).c_str());

  // Enable some different data types
  _vicon_client.EnableMarkerData();
  _vicon_client.EnableUnlabeledMarkerData();
  _vicon_client.EnableDeviceData();
}

// class destructor
DataStreamClient::~DataStreamClient(void)
{
  //TODO: cleanup
}

// mocap position thread
void DataStreamClient::run(void)
{

  while (true) //TODO: exit cleanly?
  {

    // get frame
    while (_vicon_client.GetFrame().Result != Result::Success) {
      dbg("Couldn't GetFrame()\n");
      usleep(1000);
    }

    // get timecode and populate subject message
    //    Output_GetTimecode time_code = _vicon_client.GetTimecode();
    vicon::body_t msg;
    msg.utime = _timestamp_now();

    vicon::vicon_t system;

    // get subject count
    system.nummodels = _vicon_client.GetSubjectCount().SubjectCount;

    // Uh oh, models disappeared - realloc the system
    if (system.nummodels < system.models.size())
    {
        system.models.clear();
        system.models.reserve(system.nummodels);
    }

    if (system.nummodels == 0)
    {
        std::cout << "no models detected" << std::endl;
        continue;
    }

    // loop through subjects
    for (uint subject_index = 0; subject_index < system.nummodels; subject_index++) {
      ////////////////////////////////////////////////////////////////////////
      /// Marker
      vicon::model_t& model = findModel(system.models, _vicon_client.GetSubjectName(subject_index).SubjectName);

      model.nummarkers = _vicon_client.GetMarkerCount(model.name).MarkerCount;
      if (model.nummarkers < model.markers.size())
      {
        model.markers.clear();
        model.markers.reserve(model.nummarkers);
      }

      for(int32_t j = 0; j < model.nummarkers; j++)
      {
        vicon::marker_t& marker = findMarker(model.markers, _vicon_client.GetMarkerName(model.name, j).MarkerName);
        Output_GetMarkerGlobalTranslation translation = _vicon_client.GetMarkerGlobalTranslation(model.name, marker.name);
        marker.o = translation.Occluded;
        memcpy(marker.xyz, translation.Translation, 3*sizeof(double));
      }

      ////////////////////////////////////////////////////////////////////////
      /// Segments

      // get number of segments
      model.numsegments = _vicon_client.GetSegmentCount(model.name).SegmentCount;
      if (model.numsegments < model.segments.size())
      {
          model.segments.clear();
          model.segments.reserve(model.numsegments);
      }

      // get subject name
      std::string subject_name = _vicon_client.GetSubjectName(subject_index).SubjectName;

      // get root segment name
      std::string root_segment_name = _vicon_client.GetSubjectRootSegmentName(subject_name).SegmentName;

      // loop through segments
      for (uint segment_index = 0; segment_index < model.numsegments; segment_index++) {
        // get segment name
        std::string segment_name = _vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

        vicon::segment_t& segment = findSegment(model.segments, segment_name);
        Output_GetSegmentGlobalRotationEulerXYZ A = _vicon_client.GetSegmentGlobalRotationEulerXYZ(model.name, segment.name);
        Output_GetSegmentGlobalTranslation T = _vicon_client.GetSegmentGlobalTranslation(model.name, segment.name);
        Output_GetSegmentLocalRotationEulerXYZ ba = _vicon_client.GetSegmentLocalRotationEulerXYZ(model.name, segment.name);
        Output_GetSegmentLocalTranslation bt = _vicon_client.GetSegmentLocalTranslation(model.name, segment.name);
        memcpy(segment.A, A.Rotation, 3*sizeof(double));
        memcpy(segment.T, T.Translation, 3*sizeof(double));
        memcpy(segment.ba, ba.Rotation, 3*sizeof(double));
        memcpy(segment.bt, bt.Translation, 3*sizeof(double));

        // check if root segment
        if (segment_name == root_segment_name) { //TODO: handle articulated bodies
          // get segment translation
          // Output_GetSegmentStaticTranslation segment_translation = _vicon_client.GetSegmentStaticTranslation(subject_name, segment_name);
          Output_GetSegmentGlobalTranslation segment_translation = _vicon_client.GetSegmentGlobalTranslation(
              subject_name, segment_name);

          // get segment rotation
          //Output_GetSegmentStaticRotationQuaternion segment_rotation = _vicon_client.GetSegmentStaticRotationQuaternion(subject_name, segment_name);
          Output_GetSegmentGlobalRotationQuaternion segment_rotation =
              _vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
          //           Output_GetSegmentGlobalRotationEulerXYZ   segment_rotation = _vicon_client.GetSegmentGlobalRotationEulerXYZ(subject_name, segment_name);

          // populate message with position
          for (int i = 0; i < 3; i++)
            msg.trans[i] = segment_translation.Translation[i]/1000.0; //vicon data is in mm
	  msg.quat[0] = segment_rotation.Rotation[3]; //vicon is x,y,z,w
	  msg.quat[1] = segment_rotation.Rotation[0];
	  msg.quat[2] = segment_rotation.Rotation[1];
	  msg.quat[3] = segment_rotation.Rotation[2];


          std::string channel = "VICON_" + subject_name;
          _lcm.publish(channel.c_str(), &msg);

          // break from segment for loop
          break;
        } // root segment
      } // for each segment (model.numsegments)
    } // for each subject (system.nummodels)

    _lcm.publish("VICON_MARKERS", &system);

  }

  // disconnect client
  dbg("Subject position thread terminated properly, disconnecting client\n");
  _vicon_client.Disconnect();
}

// main function
int main(int argc, char **argv)
{
  lcm::LCM lcm;

  //TODO: get host and port from command line
  std::string mocap_host_addr = DEFAULT_MOCAP_HOST_ADDR;
  std::string mocap_host_port = DEFAULT_MOCAP_HOST_PORT;

  if(argc > 1) {
      mocap_host_addr = argv[1];
  }

  std::string vicon_host = mocap_host_addr + ":" + mocap_host_port;
  std::cout << "Vicon adddress: " << vicon_host << std::endl;

  // create class instance/ connect to server
  DataStreamClient *data_stream_client = new DataStreamClient(lcm, vicon_host);

  data_stream_client->run();

  // return success
  return (0);
}
