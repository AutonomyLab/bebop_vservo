// c++
#include <cmath>

// boost
#include <boost/make_shared.hpp>

// ros
#include <angles/angles.h>
#include <visp_bridge/camera.h>

// visp
#include <visp/vpFeatureBuilder.h>

#include "bebop_vservo/bebop_vservo_ctrl.h"

namespace bebop_vservo
{

BebopVServoCtrl::BebopVServoCtrl(ros::NodeHandle &nh)
  : enabled_(false),
    servo_inited_(false),
    caminfo_recv_(false),
    force_reinit_(false),
    nh_(nh),
    nh_priv_("~"),
    sub_caminfo_(nh.subscribe("bebop/camera_info", 1, &BebopVServoCtrl::CameraCallback, this)),
    sub_cam_orientation_(nh_.subscribe("bebop/states/ardrone3/CameraState/Orientation", 10
                                      , &BebopVServoCtrl::CameraOrientationCallback, this)),
    sub_bebop_att_(nh_.subscribe("bebop/states/ardrone3/PilotingState/AttitudeChanged", 10,
                                 &BebopVServoCtrl::BebopAttitudeCallback, this)),
    sub_roi_(nh_.subscribe("target", 1, &BebopVServoCtrl::TargetCallback, this)),
    sub_enable_(nh_.subscribe("enable", 1, &BebopVServoCtrl::EnableCallback, this)),
    pub_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
    pub_debug_(nh_.advertise<bebop_vservo::Debug>("debug", 30)),
    pub_land_(nh_.advertise<std_msgs::Empty>("land", 10)),
    target_recv_time_(ros::Time(0)),
    bebop_recv_time_(ros::Time(0)),
    fov_x_(0.0),
    fov_y_(0.0),
    cam_tilt_rad_(0.0),
    bebop_yaw_rad_(0.0),
    vp_task_ptr_()  // set to null
{
  UpdateParams();
  util::ResetCmdVel(msg_cmd_vel_);
}

void BebopVServoCtrl::UpdateParams()
{
  util::GetParam(nh_priv_, "start_enabled", enabled_, false);
  util::GetParam(nh_priv_, "servo_gain", vp_gain_, 0.4);
  util::GetParam(nh_priv_, "update_freq", param_update_freq_, 30.0);
  util::GetParam(nh_priv_, "cam_tilt", cam_tilt_rad_, 0.0);
  util::GetParam(nh_priv_, "land_on_small_error", param_land_on_small_error_, false);
  util::GetParam(nh_priv_, "error_v_threshold", param_error_v_threshold_, 0.1);
  util::GetParam(nh_priv_, "error_angle_threshold", param_error_angle_threshold_, 10.0);
  param_error_angle_threshold_ = angles::from_degrees(param_error_angle_threshold_);


  cam_tilt_rad_ = angles::from_degrees(cam_tilt_rad_);
}

void BebopVServoCtrl::EnableCallback(const std_msgs::BoolConstPtr &enable_msg_ptr)
{
  enabled_ = enable_msg_ptr->data;
  ROS_INFO_STREAM("[VSER] request: " << (enabled_ ? "enable" : "disable"));
}

void BebopVServoCtrl::TargetCallback(const TargetConstPtr target_msg_ptr)
{
  target_cptr_ = target_msg_ptr;
  force_reinit_ = static_cast<bool>(target_msg_ptr->reinit);
  target_recv_time_ = ros::Time::now();
}

void BebopVServoCtrl::CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  const double& new_tilt_angle_rad = -angles::from_degrees(cam_ori_ptr->tilt);
  if (fabs(cam_tilt_rad_ - new_tilt_angle_rad) > 0.01)
  {
    // TEMP
    ROS_DEBUG_STREAM("[VSER] Bebop camera's new tilt: " << -cam_ori_ptr->tilt);
  }
  cam_tilt_rad_ = new_tilt_angle_rad;
}

void BebopVServoCtrl::BebopAttitudeCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr &att_ptr)
{
  bebop_yaw_rad_ = -att_ptr->yaw;
  bebop_recv_time_ = ros::Time::now();
}

void BebopVServoCtrl::CameraCallback(const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  if (!caminfo_recv_)
  {
    cam_model_.fromCameraInfo(cinfo_msg_ptr);
    fov_x_ = 2.0 * atan2(cam_model_.fullResolution().width / 2.0, cam_model_.fx());
    fov_y_ = 2.0 * atan2(cam_model_.fullResolution().height/ 2.0, cam_model_.fy());
    ROS_INFO_STREAM_ONCE("[VSER] FOV " << angles::to_degrees(fov_x_) << " " << angles::to_degrees(fov_y_));

    vp_cam_ = visp_bridge::toVispCameraParameters(*cinfo_msg_ptr);
    vp_cam_.printParameters();

    caminfo_recv_ = true;
    // Might be an overkill
    ROS_WARN("[VSER] Camera info received, shutting down the subscriber.");
    sub_caminfo_.shutdown();
  }

}

void BebopVServoCtrl::Reset()
{
  util::ResetCmdVel(msg_cmd_vel_);
  pub_cmd_vel_.publish(msg_cmd_vel_);
}

void BebopVServoCtrl::Spin()
{
  ROS_INFO("[VSER] Control loop started ...");

  ros::Rate loop_rate(param_update_freq_);
  while (ros::ok())
  {
    try
    {
      ros::spinOnce();
      if (!loop_rate.sleep())
      {
        throw std::runtime_error("[VSER] Missed target frequency");
      }

      if (!enabled_)
      {
        ROS_WARN_THROTTLE(10, "[VSER] Not enabled ...");
        continue;
      }
      if (!Update()) Reset();
    }
    catch (const vpException& e)
    {
      ROS_ERROR_STREAM("[VSER] Visp error " << e.what());
      Reset();
    }
    catch (const ros::Exception& e)
    {
      ROS_ERROR_STREAM("[VSER] ROS error " << e.what());
      Reset();
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[VSER] Runtime error " << e.what());
      Reset();
    }
  }
}

bool BebopVServoCtrl::Update()
{
  if (!caminfo_recv_)
  {
    ROS_WARN("[VSER] The camera info has not been received yet!");
    return false;
  }

  //bool is_valid_roi = (ros::Time::now() - target_recv_time_).toSec() < (5.0 / param_update_freq_);
  bool is_valid_roi = (ros::Time::now() - target_recv_time_).toSec() < 2.0;

  sensor_msgs::RegionOfInterest roi;
  if (is_valid_roi && target_cptr_) roi = target_cptr_->roi;
  if (roi.width < 5 || roi.height < 5 ||
      (roi.x_offset + roi.width >= cam_model_.fullResolution().width) ||
      (roi.y_offset + roi.height >= cam_model_.fullResolution().height))
  {
    ROS_WARN_STREAM_THROTTLE(1, "[VSER] roi " << roi.x_offset << " " << roi.y_offset << " " << roi.width << " " << roi.height);
    is_valid_roi = false;
  }

  // Experimental
  // Our base pattern is a square, rectangles may make the system unstable
  if (roi.width != roi.height)
  {
    int32_t roi_wh = std::max(roi.width, roi.height);
    roi.x_offset -= (roi_wh - roi.width) / 2;
    roi.y_offset -= (roi_wh - roi.height) / 2;
    roi.width = roi_wh;
    roi.height = roi_wh;

    roi.x_offset = util::clamp<int32_t>(roi.x_offset, 0, cam_model_.fullResolution().width - 1);
    roi.y_offset = util::clamp<int32_t>(roi.y_offset, 0, cam_model_.fullResolution().height - 1);
    const int32_t x2 = util::clamp<int32_t>(roi.x_offset + roi.width, 0, cam_model_.fullResolution().width - 1);
    const int32_t y2 = util::clamp<int32_t>(roi.y_offset + roi.height, 0, cam_model_.fullResolution().height - 1);
    roi.width = x2 - roi.x_offset;
    roi.height = y2 - roi.y_offset;
  }

  if (!is_valid_roi)
  {
    ROS_WARN_THROTTLE(1, "[VSER] ROI is too old or not valid. Disabling visual servo!");
    ROS_WARN_STREAM_THROTTLE(1, "[VSER] Latency " << (ros::Time::now() - target_recv_time_).toSec());

    // Experimental
    enabled_ = false;
    servo_inited_ = false;
    return false;
  }

  if (is_valid_roi && (!servo_inited_ || force_reinit_))
  {
    // This will re-set all internal params of the task
    vp_task_ptr_ = boost::make_shared<vpServo>();
    fp_.clear();
    fpd_.clear();
    fp_.resize(4);
    fpd_.resize(4);


    servo_inited_ = false;

    servo_desired_depth_ = target_cptr_->desired_depth;
    servo_desired_yaw_rad_ = target_cptr_->desired_yaw_rad;
    servo_target_height_ = target_cptr_->target_height_m;
    servo_target_width_ = target_cptr_->target_width_m;
    servo_target_dist_ground_ = target_cptr_->target_distance_ground;

    ROS_WARN_STREAM("[VSER] (re-)init " <<
                    " is_valid_roi: " << (is_valid_roi ? "yes" : "no") <<
                    " re-init request: " << (force_reinit_ ? "yes" : "no") <<
                    " Desired depth: " << servo_desired_depth_ <<
                    " Desired yaw: " << angles::to_degrees(servo_desired_yaw_rad_) <<
                    " Target height: " << servo_target_height_ <<
                    " Target width: " << servo_target_width_ <<
                    " Target d2 ground: " << servo_target_dist_ground_
                    );

    ROS_WARN_STREAM("[VSER] Target ROI [x,y,w,h]: " << roi.x_offset << " " << roi.y_offset << " " << roi.width << " " << roi.height);

    //lambda_adapt.initStandard(4.0, 0.4, 40.0);

    ROS_ASSERT(fp_.size() == 4);
    vpFeatureBuilder::create(fp_[0], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset));
    vpFeatureBuilder::create(fp_[1], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
    vpFeatureBuilder::create(fp_[2], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
    vpFeatureBuilder::create(fp_[3], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

    vpPoint point[4];
    const double h2 = servo_target_height_ / 2.0;
    const double w2 = servo_target_width_ / 2.0;
    point[0].setWorldCoordinates(-w2, -h2, 0);
    point[1].setWorldCoordinates( w2, -h2, 0);
    point[2].setWorldCoordinates( w2,  h2, 0);
    point[3].setWorldCoordinates(-w2,  h2, 0);

//    point[0].setWorldCoordinates(-h2, -w2, 0);
//    point[1].setWorldCoordinates( h2, -w2, 0);
//    point[2].setWorldCoordinates( h2,  w2, 0);
//    point[3].setWorldCoordinates(-h2,  w2, 0);

    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, servo_desired_depth_);
    vpRxyzVector cro(vpMath::rad(0.0), vpMath::rad(0.0), vpMath::rad(0.0));
    vpRotationMatrix cRo(cro);
    cMo.buildFrom(cto, cRo);

    for (uint32_t i = 0; i < 4; i++)
    {
      vpColVector cP, p_img;
      point[i].changeFrame(cMo, cP);
      point[i].projection(cP, p_img);

      fpd_[i].set_xyZ(p_img[0], p_img[1], cP[2]);
    }

    ROS_ASSERT(vp_task_ptr_);
    ROS_ASSERT((fp_.size() == 4) && (fpd_.size() == 4));

    for (uint32_t i = 0; i < 4; i++)
    {
      vp_task_ptr_->addFeature(fp_[i], fpd_[i]);
    }

    vp_task_ptr_->setServo(vpServo::EYEINHAND_CAMERA);
    vp_task_ptr_->setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    vp_task_ptr_->setLambda(vp_gain_);
    vp_task_ptr_->print();
    servo_inited_ = true;
    force_reinit_ = false;
  }

  if (!servo_inited_ || !vp_task_ptr_)
  {
    ROS_WARN_THROTTLE(1, "[VSER] Servo task has not been initialized yet.");
    return false;
  }

  const double im_width_px = static_cast<double>(cam_model_.fullResolution().width);
  const double im_height_px = static_cast<double>(cam_model_.fullResolution().height);
  double sigma_1 = static_cast<double>(roi.y_offset) / im_height_px * fov_y_;
  double sigma_2 = (im_height_px - static_cast<double>(roi.y_offset + roi.height)) / im_height_px * fov_y_;
  double roi_height_px = roi.height;
  double beta_rad = (roi_height_px / im_height_px) * fov_y_;
  double d1_m = servo_target_height_ * sin(M_PI_2 - fov_y_/2.0 + sigma_2 - cam_tilt_rad_) / sin(beta_rad);
  double z1_m = d1_m * sin(M_PI_2 + fov_y_/2.0  - cam_tilt_rad_ - sigma_1);

  ROS_ASSERT(fp_.size() == 4);

  vpFeatureBuilder::create(fp_[0], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset));
  vpFeatureBuilder::create(fp_[1], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
  vpFeatureBuilder::create(fp_[2], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
  vpFeatureBuilder::create(fp_[3], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

  for (uint32_t i = 0; i < 4; i++)
  {
    fp_[i].set_Z(z1_m);
  }
  vp_v_ = vp_task_ptr_->computeControlLaw();

  //vp_task_.print();
  ROS_DEBUG_STREAM("V_SERVO: " << vp_v_.transpose());

  /* EXPERIMENTAL */
//  ROS_ERROR_STREAM("[VSER] delay: " << (ros::Time::now() - bebop_recv_time_).toSec());
//  ROS_ERROR_STREAM("[VSER] current yaw err (deg): " << angles::to_degrees(angles::normalize_angle(servo_desired_yaw_rad_ - bebop_yaw_rad_)));

  const double yaw_image_err_raw =
      ((cam_model_.fullResolution().width/2.0 - (roi.x_offset + roi.width / 2.0)) / (double) cam_model_.fullResolution().width) * fov_x_;

  vp_v_[0] = 0.0;
  servo_desired_yaw_rad_ = vp_gain_ * yaw_image_err_raw;
  /* END */


  // Here we should convert from Camera (Visp) coordinates to Bebop Coordinates
  // TODO: Consider camera tilt
  // Servo:           z: forward, x: right, y: down
  // Maps to bebop:   z: x_b    , x: -y_b , y: -z_b

  util::ResetCmdVel(msg_cmd_vel_);

  // No clamp/filter here
  // Map to Bebop's frame from camera's frame

  const double& vx_cam = vp_v_[2];
  const double& vz_cam = -vp_v_[1];

  const bool do_land = param_land_on_small_error_ &&
      (fabs(vx_cam) < param_error_v_threshold_) &&
      (fabs(vz_cam) < param_error_v_threshold_) &&
      (fabs(yaw_image_err_raw) < param_error_angle_threshold_);

  // TODO: Check the freshness of tilt!
  msg_cmd_vel_.linear.x =  vx_cam * cos(cam_tilt_rad_) + vz_cam * sin(cam_tilt_rad_);
  msg_cmd_vel_.linear.z = -vx_cam * sin(cam_tilt_rad_) + vz_cam * cos(cam_tilt_rad_);
  msg_cmd_vel_.linear.y = -vp_v_[0];

//  msg_cmd_vel_.linear.x = vp_v_[2];
//  msg_cmd_vel_.linear.y = -vp_v_[0];
//  msg_cmd_vel_.linear.z = -vp_v_[1];

  // We can't control all four DOF of the vehicle
  msg_cmd_vel_.angular.z = servo_desired_yaw_rad_;

  if (do_land)
  {
    ROS_WARN("[VSER] land on small error has been activated");
    std_msgs::Empty empty_msg;
    util::ResetCmdVel(msg_cmd_vel_);
    pub_land_.publish(empty_msg);
  }

  pub_cmd_vel_.publish(msg_cmd_vel_);

  msg_debug_.header.stamp = ros::Time::now();
  msg_debug_.header.frame_id = "";
  msg_debug_.bb_height = roi_height_px;
  //msg_debug_.d_groundtruth = d_truth;
  msg_debug_.cam_tilt = angles::to_degrees(cam_tilt_rad_);
  msg_debug_.beta = beta_rad;
  msg_debug_.sigma_1 = sigma_1;
  msg_debug_.sigma_2 = sigma_2;
  msg_debug_.d_raw_bb = z1_m;
  msg_debug_.target_height = servo_target_height_;
  msg_debug_.target_grounddist = servo_target_dist_ground_;
  for (uint32_t i = 0; i < 6; ++i)
    msg_debug_.v_img[i] = vp_v_.data[i];

  msg_debug_.desired_depth = servo_desired_depth_;
  msg_debug_.desired_yaw_rad = servo_desired_yaw_rad_;
  msg_debug_.target_roi = roi;

  pub_debug_.publish(msg_debug_);
  return true;
}

}  // namespace bebop_servo
