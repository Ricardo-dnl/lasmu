#include <pluginlib/class_list_macros.h>
#include <uwsim/New_SLS.h>



class UpdateLMVPM : public osg::Uniform::Callback
{
public:
  UpdateLMVPM(osg::Camera* camera) :
      mCamera(camera)
  {
  }
  virtual void operator ()(osg::Uniform* u, osg::NodeVisitor*)
  {
    osg::Matrixd lmvpm = mCamera->getViewMatrix() * mCamera->getProjectionMatrix() * osg::Matrix::translate(1, 1, 1)
        * osg::Matrix::scale(0.5, 0.5, 0.5);

    u->set(lmvpm);
  }

protected:
  osg::Camera* mCamera;
};


void New_SLS::New_SLS_Sensing(std::string name,std::string parentName, osg::Node *root, osg::Node *node, std::string image_name,
                                         double fov, bool laser)
{
  double range = 0;

  this->name = name;
  this->fov = fov;
  this->range = range;
  this->node = node;
  this->image_name = image_name;
  this->textureUnit = 3; // It shouldn't be fixed

  //Create projected texture
  osg::Texture2D* texture = new osg::Texture2D();
  osg::Image* texture_to_project = osgDB::readImageFile(this->image_name);
  assert(texture_to_project);
  texture->setImage(texture_to_project);
  texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER); // It makes texture not to repeat 
  texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER); // F.M.I: http://lucera-project.blogspot.com.es/2010/06/opengl-wrap.html
  texture->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_BORDER);
  texture->setBorderColor(osg::Vec4d(0.0, 0.0, 0.0, 0.0));
  root->getOrCreateStateSet()->setTextureAttributeAndModes(4, texture, osg::StateAttribute::ON);

  //Shadow camera
  camera = VirtualCamera(root->asGroup(), name,parentName, node, texture_to_project->s(), texture_to_project->t(), fov,
                         texture_to_project->s() / (float)texture_to_project->t());


  printf("\n\n INIT Camera SLS \n \n");

  //Create depth texture for shadow mapping test
  dbgDepthTexture = new osg::Texture2D;
  dbgDepthTexture->setTextureSize(texture_to_project->s(), texture_to_project->t()); //CHECK: Maybe we can use a smaller texture?
  dbgDepthTexture->setInternalFormat(GL_DEPTH_COMPONENT);
  dbgDepthTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
  dbgDepthTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
  root->getOrCreateStateSet()->setTextureAttributeAndModes(3, dbgDepthTexture, osg::StateAttribute::ON);
  camera.textureCamera->attach(osg::Camera::DEPTH_BUFFER, dbgDepthTexture);

  //Uniform to update texture
  osg::Matrixd lmvpm = camera.textureCamera->getViewMatrix() * camera.textureCamera->getProjectionMatrix()
      * osg::Matrix::translate(1, 1, 1) * osg::Matrix::scale(0.5, 0.5, 0.5);
  osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT_MAT4,"LightModelViewProjectionMatrix");
  u->setUpdateCallback(new UpdateLMVPM(camera.textureCamera));
  root->getOrCreateStateSet()->addUniform(u);

  // add Laser uniform to change from laser to light behaviours
  osg::Uniform* laserUniform = new osg::Uniform("isLaser", laser);
  root->getOrCreateStateSet()->addUniform(laserUniform);

}

New_SLS::New_SLS(New_SLS_Config * cfg, osg::Node *trackNode) : SimulatedDevice(cfg)
{
    // Set position of the sensor according to declaration in
    // xml with relativeTo variable
    cout<<"\ntrackNode----"<<trackNode<<"----\n";

    this->parent = trackNode;
    this->relativeTo = cfg->relativeTo;
    this->position[0] = cfg->position[0];
    this->position[1] = cfg->position[1];
    this->position[2] = cfg->position[2];
    this->orientation[0] = cfg->orientation[0];
    this->orientation[1] = cfg->orientation[1];
    this->orientation[2] = cfg->orientation[2];
}


SimulatedDeviceConfig::Ptr New_SLS_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{   
  New_SLS_Config * cfg = new New_SLS_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();

  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);

    if (child->get_name() == "relativeTo"){
        printf("\n\n aqui1\n \n");
      config->extractStringChar(child, cfg->relativeTo);
      config->extractStringChar(child,cfg->linkName);
    }
    if (child->get_name() == "linkName"){
      config->extractStringChar(child,cfg->linkName);
      printf("\n\n aqui2\n \n");
    }
    else if (child->get_name() == "position")
      config->extractPositionOrColor(child, cfg->position);
    else if (child->get_name() == "orientation")
      config->extractOrientation(child, cfg->orientation);
    else if (child->get_name() == "name")
      config->extractStringChar(child, cfg->name);
    else if (child->get_name() == "fov")
      config->extractFloatChar(child, cfg->fov);
    else if (child->get_name() == "laser")
      config->extractIntChar(child, cfg->laser);
    else if (child->get_name() == "image_name")
      config->extractStringChar(child, cfg->image_name);
  }
 
  return SimulatedDeviceConfig::Ptr(cfg);
}



bool New_SLS_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
{
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      New_SLS_Config * cfg = dynamic_cast<New_SLS_Config *>(vehicleChars.simulated_devices[i].get());
      osg::ref_ptr<osg::Node> target;
      if (cfg)
      {
        int target=-1;
        for(int j=0;j<auv->urdf->link.size();j++)
        {
          if(auv->urdf->link[j]->getName()==cfg->relativeTo)
          {
            target=j;
          }
        }
        if(target==-1)
		{
          OSG_FATAL << "New_SLS device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has an unknown relativeTo, discarding..." << std::endl;
        }
        else
        {
			//OSG_INFO << "Adding a New_SLS..." << std::endl;
            printf("\n\n adding Camera SLS \n \n");
          osg::ref_ptr < osg::Transform > vMd = (osg::Transform*) new osg::PositionAttitudeTransform;
          
		  vMd->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
          vMd->asPositionAttitudeTransform()->setAttitude(
              osg::Quat(cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1], osg::Vec3d(0, 1, 0),
              cfg->orientation[2], osg::Vec3d(0, 0, 1)));
          auv->urdf->link[target]->getParent(0)->getParent(0)->asGroup()->addChild(vMd);
 
          //auv->devices->all.push_back(New_SLS::Ptr(new New_SLS(cfg,vMd)));
        // Create an object New_SLS
          New_SLS* SLS;
          SLS = new New_SLS(cfg, vMd);

 		//sls_projectors.push_back(VirtualSLSProjector(slp.name, slp.linkName, oscene->root, //maybe oscene->scene->localizedWorld ?
        //                                         vMp, slp.image_name, slp.fov, (slp.laser) ? true : false));  
          //printf("\n\n ---%s \n \n",cfg->linkName);
          //cout<<cfg->linkName;

        SLS->New_SLS_Sensing(cfg->name, cfg->linkName, sceneBuilder->root, //maybe oscene->scene->localizedWorld ?
                                                 vMd, cfg->image_name, cfg->fov, (cfg->laser) ? true : false);


        auv->devices->all.push_back(New_SLS::Ptr(SLS));
        //camview.push_back(auv->SimulatedIAUV.);

        auv->camview.push_back(SLS->camera);

        cout<<"camara name"<<SLS->camera.name;

		//OSG_INFO << "Done adding a structured New_SLS..." << std::endl;
        }
      }
      else
        OSG_FATAL << "New_SLS device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
            << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
    }
  return true;
}


  // Adding Structured light projector
  /*while (vehicleChars.sls_projectors.size() > 0)
  {
    OSG_INFO << "Adding a structured light projector..." << std::endl;
    slProjector slp;
    slp = vehicleChars.sls_projectors.front();
    vehicleChars.sls_projectors.pop_front();
    
	osg::ref_ptr < osg::Transform > vMp = (osg::Transform*)new osg::PositionAttitudeTransform;
    vMp->asPositionAttitudeTransform()->setPosition(osg::Vec3d(slp.position[0], slp.position[1], slp.position[2]));
    vMp->asPositionAttitudeTransform()->setAttitude(
        osg::Quat(slp.orientation[0], osg::Vec3d(1, 0, 0), slp.orientation[1], osg::Vec3d(0, 1, 0), slp.orientation[2],
                  osg::Vec3d(0, 0, 1)));
    urdf->link[slp.link]->getParent(0)->getParent(0)->asGroup()->addChild(vMp);


    //camview.push_back(VirtualCamera(oscene->root, "slp_camera", vMp, 512, 512,slp.fov,102.4));
    sls_projectors.push_back(VirtualSLSProjector(slp.name, slp.linkName, oscene->root, //maybe oscene->scene->localizedWorld ?
                                                 vMp, slp.image_name, slp.fov, (slp.laser) ? true : false));
    camview.push_back(sls_projectors.back().camera);


    OSG_INFO << "Done adding a structured light projector..." << std::endl;
  }*/



std::vector<boost::shared_ptr<ROSInterface> > New_SLS_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector < boost::shared_ptr<ROSInterface> > ifaces;
  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
      if (iauvFile[i]->devices->all[d]->getType() == this->getType()
          && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
      {
        ifaces.push_back(
            boost::shared_ptr < ROSInterface
                > (new New_SLS_ROSPublisher(dynamic_cast<New_SLS*>(iauvFile[i]->devices->all[d].get()),
                                                rosInterface.topic, rosInterface.rate)));
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
  return ifaces;
}



void New_SLS_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("New_SLS_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise < geometry_msgs::Pose > (topic, 1);
}

void New_SLS_ROSPublisher::publish()
{
  geometry_msgs::Pose msg;
  boost::shared_ptr<osg::Matrix> mat = getWorldCoords(dev->parent);
  msg.position.x= mat->getTrans().x();
  msg.position.y= mat->getTrans().y();
  msg.position.z= mat->getTrans().z();
  msg.orientation.x=mat->getRotate().x();	
  msg.orientation.y=mat->getRotate().y();	
  msg.orientation.z=mat->getRotate().z();	
  msg.orientation.w=mat->getRotate().w();	
  pub_.publish(msg);
}


#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(New_SLS_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(New_SLS_Factory, New_SLS_Factory, uwsim::SimulatedDeviceFactory)
#endif

