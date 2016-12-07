#ifndef NEW_SLS_H_
#define NEW_SLS_H_
#include "SimulatedDevice.h"
#include <ros/ros.h>
using namespace uwsim;

/* You will need to add your code HERE */
#include <osgDB/ReadFile>

#include "VirtualCamera.h"


class New_SLS_Factory : public SimulatedDeviceFactory
{
public:
    //std::vector<VirtualCamera> camview;
  //std::list<slProjector> sls_projectors;
  //this is the only place the device/interface type is set
  New_SLS_Factory(std::string type_ = "New_SLS") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                                                            std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

#include "ConfigXMLParser.h"
#include <osg/Node>
#include "SimulatedIAUV.h"
#include <osg/PositionAttitudeTransform>


/*struct slProjector
{
  string name;
  string linkName;
  string image_name;
  double position[3], orientation[3];
  double fov;
  int laser;
  int link;
  void init()
  {
    name = "";
    linkName = "";
    image_name = "";
    position[0] = 0;
    position[1] = 0;
    position[2] = 0;
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    fov = 0;
    laser = 1;
  }
};
  void processSLProjector(const xmlpp::Node* node, slProjector &slp);*/

class New_SLS_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string relativeTo;
  string linkName;
  string image_name;
  double position[3], orientation[3];
  double fov;
  int laser;
  int link;

  //std::string relativeTo;
  //double position[3], orientation[3];
  //constructor
  New_SLS_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

class New_SLS : public SimulatedDevice
{
public:
  osg::Node *parent; 
  
  std::string relativeTo;               /*!< Link from which we position the sensor */
  double position[3], orientation[3]; /*!< Variables which define the Pose of the sensor */
  //acrescentei//////
  /// \brief name
  //std::vector<VirtualCamera> camview;
  std::string name;
  std::string image_name;
  osg::ref_ptr<osg::Node> node;
  osg::ref_ptr<osg::Node> root;
  double range; ///< Max projection range
  double fov; ///< Field of view
  unsigned int textureUnit;
  osg::Texture2D* dbgDepthTexture;
  VirtualCamera camera;

  New_SLS(New_SLS_Config * cfg, osg::Node *node);

  void New_SLS_Sensing(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name, double fov,
                      bool laser);
  //New_SLS();

  /*virtual void init(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name, double range,
                    double fov, bool laser);*/
  //////////////////
  
  //New_SLS(New_SLS_Config * cfg,osg::Node *trackNode);
};

#include "ROSInterface.h"

class New_SLS_ROSPublisher : public ROSPublisherInterface
{
  New_SLS * dev;
public:
  New_SLS_ROSPublisher(New_SLS *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~New_SLS_ROSPublisher()
  {
  }
};

#endif
