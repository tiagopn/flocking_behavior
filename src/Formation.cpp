#include <Formation.h>
#include <pluginlib/class_list_macros.h>

namespace formation
{

/* onInit() //{ */

void Formation::onInit() {
  /* set flags to false */
  is_initialized_        = false;
  hover_mode_            = false;
  swarming_mode_         = false;
  state_machine_running_ = false;
  has_virtual_heading_   = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "Formation");

  /* load parameters */
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("frame", _frame_);
  param_loader.loadParam("land_at_the_end", _land_end_);
  // param_loader.loadParam("use_3D", _use_3D_);
  param_loader.loadParam("minimum_height", _minimum_height_);

  param_loader.loadParam("flocking/auto_start", _auto_start_);
  param_loader.loadParam("flocking/swarming_after_hover", _timeout_state_change_);
  param_loader.loadParam("flocking/duration", _timeout_flocking_);

  /* load proximal control parameters */
  param_loader.loadParam("flocking/proximal/desired_distance", _desired_distance_);
  param_loader.loadParam("flocking/proximal/strength_potential", _strength_potential_);
  param_loader.loadParam("flocking/proximal/steepness_potential", _steepness_potential_);
  param_loader.loadParam("flocking/proximal/range_multiplier", _range_multipler_);

  /* load motion control parameters */
  param_loader.loadParam("flocking/motion/K1", _K1_);
  param_loader.loadParam("flocking/motion/K2", _K2_);
  param_loader.loadParam("flocking/motion/K3", _K3_);
  param_loader.loadParam("flocking/motion/move_forward", _move_forward_);
  param_loader.loadParam("flocking/motion/interpolate_coeff", _interpolate_coeff_);
  param_loader.loadParam("flocking/motion/fixed_heading", _fixed_heading_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Formation]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | -------------------- batch visualzier -------------------- |

  bw_ = mrs_lib::BatchVisualizer(nh, "markers", "common_origin");

  bw_.clearBuffers();
  bw_.clearVisuals();

  /* set remaining parameters using loaded ones */
  noise_     = _desired_distance_ / pow(2, 1 / _steepness_potential_);
  max_range_ = _range_multipler_ * _desired_distance_;

  /* get current heading */
  /* nav_msgs::Odometry::ConstPtr       odom   = ros::topic::waitForMessage<nav_msgs::Odometry>("/" + _uav_name_ + "/control_manager/cmd_odom",
   * ros::Duration(15)); */
  /* mrs_msgs::Float64Stamped::ConstPtr height = ros::topic::waitForMessage<mrs_msgs::Float64Stamped>("/" + _uav_name_ + "/odometry/height", ros::Duration(15));
   */

  /* set smooth or virtual heading */
  /* smooth_heading_ = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading(); */

  pub_virtual_heading_ = nh.advertise<mrs_msgs::Float64Stamped>("/" + _uav_name_ + "/flocking/virtual_heading", 1);

  /* subscribers */
  sub_this_uav_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _uav_name_ + "/control_manager/cmd_odom", 1, &Formation::callbackThisUAVOdom, this);

  /* publishers */
  pub_mode_changed_ = nh.advertise<flocking::ModeStamped>("/" + _uav_name_ + "/flocking/mode_changed", 1);

  /* message filters */
  sub_odom_.subscribe(nh, "/" + _uav_name_ + "/control_manager/cmd_odom", 1);
  sub_neighbors_info_.subscribe(nh, "/" + _uav_name_ + "/sensor_neighbor/neighbors", 1);
  sub_height_.subscribe(nh, "/" + _uav_name_ + "/odometry/height", 1);

  /* service client */
  srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _uav_name_ + "/control_manager/reference");
  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/" + _uav_name_ + "/uav_manager/land");

  /* service servers */
  srv_server_state_machine_ = nh.advertiseService("start_state_machine_in", &Formation::callbackStartStateMachine, this);
  srv_server_hover_mode_    = nh.advertiseService("start_hover_mode", &Formation::callbackStartHoverMode, this);
  srv_server_swarming_mode_ = nh.advertiseService("start_swarming_mode", &Formation::callbackStartSwarmingMode, this);
  srv_server_close_node_    = nh.advertiseService("close_node", &Formation::callbackCloseNode, this);

  /* timers */
  timer_state_machine_ = nh.createTimer(ros::Duration(_timeout_state_change_), &Formation::callbackTimerStateMachine, this, true, false);
  timer_flocking_end_  = nh.createTimer(ros::Duration(_timeout_flocking_), &Formation::callbackTimerAbortFlocking, this, false, false);

  sync_.reset(new Sync(FormationPolicy(10), sub_neighbors_info_, sub_odom_, sub_height_));
  sync_->registerCallback(boost::bind(&Formation::callbackUAVNeighbors, this, _1, _2, _3));

  // | --------------------- finish the init -------------------- |

  ROS_INFO_ONCE("[Formation]: initialized");
  is_initialized_ = true;

  ros::spin();
}

//}

/* callbackThisUAVOdom() //{ */
void Formation::callbackThisUAVOdom(const nav_msgs::Odometry::ConstPtr& odom) {

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_this_uav_odom_);

    this_uav_odom_ = *odom;
  }

  got_this_uav_odom_ = true;

  {
    std::scoped_lock lock(mutex_virtual_heading_);

    /* create float64 stamped msg */
    mrs_msgs::Float64Stamped msg_float_stamped;

    /* fill in */
    msg_float_stamped.header.stamp    = ros::Time::now();
    msg_float_stamped.header.frame_id = _uav_name_ + "/" + _frame_;
    msg_float_stamped.value           = virtual_heading_;

    /* publish virtual heading */
    try {
      pub_virtual_heading_.publish(msg_float_stamped);
    } catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_virtual_heading_.getTopic().c_str());
    }
  }
}

//}

// | ----------------------- message filters callbacks ----------------------- |

/* callbackUAVNeighbors() //{ */

void Formation::callbackUAVNeighbors(const flocking::Neighbors::ConstPtr& neighbors, const nav_msgs::Odometry::ConstPtr& odom,
                                     const mrs_msgs::Float64Stamped::ConstPtr& height) {
  /* return if is not initialized or the code is not on swarming mode */
  if (!is_initialized_ || !swarming_mode_) {
    return;
  }

  if (_fixed_heading_ && !has_virtual_heading_) {
    return;
  }

  /* calculate proximal control vector P */
  double prox_vector_x = 0.0;
  double prox_vector_y = 0.0;
  double prox_vector_z = 0.0;
  double prox_magnitude;

  if (neighbors->num_neighbors > 0) {
    for (unsigned int i = 0; i < neighbors->num_neighbors; i++) {
      if (neighbors->range[i] <= max_range_) {
        prox_magnitude = Formation::getProximalMagnitude(neighbors->range[i]);

        prox_vector_x += prox_magnitude * sin(neighbors->inclination[i]) * cos(neighbors->bearing[i]);
        prox_vector_y += prox_magnitude * sin(neighbors->inclination[i]) * sin(neighbors->bearing[i]);
        prox_vector_z += prox_magnitude * cos(neighbors->inclination[i]);
      }
    }
  }

  /* convert flocking control f (f = p) vector to angular and linear movement */
  double flock_vector_x = prox_vector_x;
  double flock_vector_y = prox_vector_y;
  double flock_vector_z = prox_vector_z;

  double u = flock_vector_x * _K1_ + _move_forward_;
  double w = flock_vector_y * _K2_;
  double v = flock_vector_z * _K3_;

  /* create reference stamped service msg */
  mrs_msgs::ReferenceStampedSrv srv_reference_stamped_msg;

  /* fill in header */
  srv_reference_stamped_msg.request.header.stamp    = ros::Time::now();
  srv_reference_stamped_msg.request.header.frame_id = _uav_name_ + "/" + _frame_;

  /* get current heading */
  double heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();

  if (_fixed_heading_) {
    {
      std::scoped_lock lock(mutex_virtual_heading_);
      virtual_heading_ = mrs_lib::geometry::radians::interp(virtual_heading_, heading, _interpolate_coeff_);

      /* fill in reference */
      srv_reference_stamped_msg.request.reference.position.x = odom->pose.pose.position.x + u * cos(virtual_heading_);
      srv_reference_stamped_msg.request.reference.position.y = odom->pose.pose.position.y + u * sin(virtual_heading_);
      srv_reference_stamped_msg.request.reference.heading    = initial_heading_;

      /* update virtual heading */
      virtual_heading_ = virtual_heading_ + w;
    }
  } else {

    ROS_INFO("smooth heading before update: %.2f, heading %.2f", smooth_heading_, heading);

    smooth_heading_ = mrs_lib::geometry::radians::interp(smooth_heading_, heading, _interpolate_coeff_);

    ROS_INFO("smooth heading after update: %.2f", smooth_heading_);

    /* fill in reference */
    srv_reference_stamped_msg.request.reference.position.x = odom->pose.pose.position.x + u * cos(heading);
    srv_reference_stamped_msg.request.reference.position.y = odom->pose.pose.position.y + u * sin(heading);
    srv_reference_stamped_msg.request.reference.heading    = smooth_heading_ + w;
  }

  double height_push;
  double desired_altitude;

  height_push = math_utils::getMaxValue(0, _minimum_height_ - height->value);

  if (v + height_push + height->value <= _minimum_height_) {
    desired_altitude = odom->pose.pose.position.z + height_push;
  } else {
    desired_altitude = odom->pose.pose.position.z + v + height_push;
  }

  /* set height */
  // if (_use_3D_) {
  // srv_reference_stamped_msg.request.reference.position.z = math_utils::getMaxValue(odom->pose.pose.position.z + v, _minimum_height_);

  srv_reference_stamped_msg.request.reference.position.z = desired_altitude;

  //} else {

  // srv_reference_stamped_msg.request.reference.position.z = math_utils::getMaxValue(odom->pose.pose.position.z + v, odom->pose.pose.position.z +
  // neighbors->max_height_diff);

  // desired_altitude = desired_altitude + neighbors->max_height_diff;

  //  srv_reference_stamped_msg.request.reference.position.z = desired_altitude;
  //}

  bw_.clearBuffers();
  bw_.clearVisuals();

  bw_.setParentFrame(srv_reference_stamped_msg.request.header.frame_id);

  // example of plotting a red square
  bw_.addPoint(vec3_t(srv_reference_stamped_msg.request.reference.position.x, srv_reference_stamped_msg.request.reference.position.y,
                      srv_reference_stamped_msg.request.reference.position.z),
               1.0, 0.0, 0.0, 1.0);
  bw_.setPointsScale(0.3);

  // this should plot a vector, but does not work now
  Eigen::Vector3d        uav_pose(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  Eigen::Vector3d        vector_tip(srv_reference_stamped_msg.request.reference.position.x, srv_reference_stamped_msg.request.reference.position.y,
                             srv_reference_stamped_msg.request.reference.position.z);
  mrs_lib::geometry::Ray ray = mrs_lib::geometry::Ray::twopointCast(uav_pose, vector_tip);
  bw_.addRay(ray, 0, 0, 1, 1);

  /* Eigen::Vector3d center(srv_reference_stamped_msg.request.reference.position.x, srv_reference_stamped_msg.request.reference.position.y,
   * srv_reference_stamped_msg.request.reference.position.z); */
  /* Eigen::Vector3d scale(0.3, 0.3, 0.3); */
  /* mrs_lib::geometry::Cuboid cub(center, scale, mrs_lib::AttitudeConverter(0, 0, 0)); */
  /* bw_.addCuboid(cub, 0, 0, 1, 1, true); */

  bw_.publish();

  /* request service */
  if (srv_client_goto_.call(srv_reference_stamped_msg)) {

  } else {
    ROS_ERROR("[Formation]: Failed to call service.\n");
  }
}

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerStateMachine() //{ */

void Formation::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_ || !state_machine_running_) {
    return;
  }

  ros::Time now = ros::Time::now();
  if (hover_mode_) {
    /* turn on swarming mode */
    state_change_time_ = now;
    swarming_mode_     = true;

    /* create mode stamped msg */
    flocking::ModeStamped ms;

    /* fill in msg */
    ms.header.frame_id = _uav_name_;
    ms.header.stamp    = ros::Time::now();
    ms.mode            = "Swarming";

    /* publish mode changed */
    pub_mode_changed_.publish(ms);

    /* start flocking countdown */
    timer_flocking_end_.start();
  }

  return;
}

//}

/* callbackTimerAbortFlocking() //{ */

void Formation::callbackTimerAbortFlocking([[maybe_unused]] const ros::TimerEvent& event) {
  /* turn off swarming mode */
  swarming_mode_ = false;

  /* create mode stamped msg */
  flocking::ModeStamped ms;

  /* fill in msg */
  ms.header.frame_id = _uav_name_;
  ms.header.stamp    = ros::Time::now();
  ms.mode            = "No mode";

  /* publish mode changed */
  pub_mode_changed_.publish(ms);

  /* request land service */
  if (_land_end_) {
    ROS_INFO("[Formation]: Calling land service");
    std_srvs::Trigger srv_land_call;
    srv_client_land_.call(srv_land_call);
  }

  ROS_INFO_ONCE("[Formation]: The time is over. Shutting down");

  /* reset message filter */
  sync_.reset();

  /* shutdown node */
  ros::shutdown();

  return;
}

//}

// | ----------------------- service server callbacks ----------------------- |

/* callbackStartStateMachine() //{ */

bool Formation::callbackStartStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!_auto_start_) {
    ROS_WARN_ONCE("[Formation]: The automatic start is not on. The hover and swarming mode should start manually.");
    res.message = "[Formation]: The automatic start is not on. The hover and swarming mode should start manually.";
    res.success = false;
    return false;
  } else if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot start state machine, nodelet is not initialized.");
    res.message = "Cannot change to hover mode, nodelet is not initialized.";
    res.success = false;
    return false;
  } else if (hover_mode_) {
    ROS_WARN("[Formation]: Cannot start state machine, already in hover mode.");
    res.message = "Cannot start state machine, already in hover mode.";
    res.success = false;
    return false;
  } else if (swarming_mode_) {
    ROS_WARN("[Formation]: Cannot start state machine, already in swarming mode.");
    res.message = "Cannot start state machine, already in swarming mode.";
    res.success = false;
    return false;
  }

  /* change code to hover mode */
  hover_mode_            = true;
  state_change_time_     = ros::Time::now();
  state_machine_running_ = true;

  /* create mode stamped msg */
  flocking::ModeStamped ms;

  /* fill in msg */
  ms.header.frame_id = _uav_name_;
  ms.header.stamp    = ros::Time::now();
  ms.mode            = "Hover";

  /* publish mode changed */
  pub_mode_changed_.publish(ms);

  timer_state_machine_.start();

  ROS_INFO("[Formation]: Changed to hover mode. Swarming mode will be activated in %0.2f seconds.", _timeout_state_change_);
  res.message = "Changed to hover mode. Swarming mode will be activated in specified timeout.";
  res.success = true;

  return true;
}

//}

/* callbackStartHoverMode() //{ */

bool Formation::callbackStartHoverMode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot change to hover mode, nodelet is not initialized");
    res.message = "Cannot change to hover mode, nodelet is not initialized";
    res.success = false;
    return true;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot change to hover mode. The states are handled by a state machine.");
    res.message = "Cannot change to hover mode. The states are handled by a state machine.";
    res.success = false;
    return true;
  }

  if (!got_this_uav_odom_) {
    ROS_WARN("[Formation]: Cannot change to hover mode. Missing this uav odom.");
    res.message = "Cannot change to hover mode. Missing this uav odom.";
    res.success = false;
    return true;
  }

  nav_msgs::Odometry this_uav_odom;

  {
    std::scoped_lock lock(mutex_this_uav_odom_);

    this_uav_odom = this_uav_odom_;
  }

  if (!has_virtual_heading_) {

    double current_heading = 0;

    try {
      current_heading = mrs_lib::AttitudeConverter(this_uav_odom.pose.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Formation]: exception caught while getting the heading in the hover callback");
      res.message = "Cannot hover, exception while calculating current heading";
      res.success = false;
      return true;
    }

    if (!std::isfinite(current_heading)) {
      ROS_ERROR("[Formation]: exception caught while getting the heading in the hover callback (NaN)");
      res.message = "Cannot hover, exception while calculating current heading";
      res.success = false;
      return true;
    }

    if (_fixed_heading_) {
      {
        std::scoped_lock lock(mutex_virtual_heading_);
        initial_heading_ = current_heading;
        virtual_heading_ = current_heading;
      }

    } else {

      smooth_heading_ = current_heading;
    }

    has_virtual_heading_ = true;
  }

  ROS_INFO("[Formation]: initial heading is %.2f", initial_heading_);
  ROS_INFO("[Formation]: virtual heading is %.2f", virtual_heading_);
  ROS_INFO("[Formation]: smooth heading is %.2f", smooth_heading_);

  /* change code to hover mode */
  hover_mode_ = true;

  /* create mode stamped msg */
  flocking::ModeStamped ms;

  /* fill in msg */
  ms.header.frame_id = _uav_name_;
  ms.header.stamp    = ros::Time::now();
  ms.mode            = "Hover";

  /* publish mode changed */
  pub_mode_changed_.publish(ms);

  ROS_INFO("[Formation]: Changed to hover mode. It is safe now to start the swarming mode");
  res.message = "Changed to hover mode. It is safe now to start the swarming mode";
  res.success = true;

  return true;
}

//}

/* callbackStartSwarmingMode() //{ */

bool Formation::callbackStartSwarmingMode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    ROS_WARN("[Formation]: cannot change to swarming mode, nodelet is not initialized");
    res.message = "Cannot change to swarming mode, nodelet is not initialized";
    res.success = false;
    return false;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot change to swarming mode. The states are handled by a state machine.");
    res.message = "Cannot change to swarming mode. The states are handled by a state machine.";
    res.success = false;
    return false;
  } else if (!hover_mode_) {
    ROS_WARN("[Formation]: Cannot change to swarming mode, hover mode has not been started");
    res.message = "Cannot change to swarming mode, hover mode has not been started";
    res.success = false;
    return false;
  }

  ROS_INFO("[Formation]: Starting the swarming mode");
  res.message = "Starting the swarming mode";
  res.success = true;

  /* change code to swarming mode */
  swarming_mode_ = true;

  /* create mode stamped msg */
  flocking::ModeStamped ms;

  /* fill in msg */
  ms.header.frame_id = _uav_name_;
  ms.header.stamp    = ros::Time::now();
  ms.mode            = "Swarming";

  /* publish mode changed */
  pub_mode_changed_.publish(ms);

  /* start flocking countdown */
  timer_flocking_end_.start();

  ROS_INFO("[Formation]: The flocking behavior has started and will last %0.2f seconds", _timeout_flocking_);

  return true;
}

//}

/* callbackCloseNode() //{ */

bool Formation::callbackCloseNode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot stop formation, nodelet is not initialized");
    res.message = "Cannot stop formation, nodelet is not initialized";
    res.success = false;
    return false;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot close node. The states are handled by a state machine.");
    res.message = "Cannot close node. The states are handled by a state machine.";
    res.success = false;
    return false;
  }

  /* turn off swarming mode */
  swarming_mode_ = false;

  /* change code to swarming mode */
  swarming_mode_ = true;

  /* create mode stamped msg */
  flocking::ModeStamped ms;

  /* fill in msg */
  ms.header.frame_id = _uav_name_;
  ms.header.stamp    = ros::Time::now();
  ms.mode            = "No mode";

  /* publish mode changed */
  pub_mode_changed_.publish(ms);

  /* request land service */
  if (_land_end_) {
    ROS_INFO("[Formation]: Calling land service");
    std_srvs::Trigger srv_land_call;
    srv_client_land_.call(srv_land_call);
  }

  ROS_INFO("[Formation]: Closing node before the time. Shutting down");
  res.message = "Closing node before the time. Shutting down";
  res.success = true;

  /* reset message filter */
  sync_.reset();

  /* shutdown node */
  ros::shutdown();

  return true;
}

//}

// | -------------------------- support functions -------------------------- |

/* getProximalMagnitude() //{ */

double Formation::getProximalMagnitude(double range) {
  return -4 * _steepness_potential_ * _strength_potential_ / range *
         (2 * pow(noise_ / range, 2 * _steepness_potential_) - pow(noise_ / range, _steepness_potential_));
}

//}

}  // namespace formation

PLUGINLIB_EXPORT_CLASS(formation::Formation, nodelet::Nodelet);
