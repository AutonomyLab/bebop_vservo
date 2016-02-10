// c++
#include <cmath>

// boost
#include <boost/thread/lock_guard.hpp>

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
    nh_(nh),
    nh_priv_("~"),
    sub_caminfo_(nh.subscribe("bebop/camera_info", 1, &BebopVServoCtrl::CameraCallback, this)),
    sub_cam_orientation_(nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10
                                      , &BebopVServoCtrl::CameraOrientationCallback, this)),
    sub_roi_(nh_.subscribe("target", 1, &BebopVServoCtrl::TargetCallback, this)),
    sub_enable_(nh_.subscribe("enable", 1, &BebopVServoCtrl::EnableCallback, this)),
    pub_cmd_vel_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
    pub_debug_(nh_.advertise<bebop_vservo::Debug>("debug", 30)),
    target_recv_time_(ros::Time(0)),
    fov_x_(0.0),
    fov_y_(0.0),
    cam_tilt_rad_(0.0)
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
  target_recv_time_ = ros::Time::now();
}

void BebopVServoCtrl::CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  cam_tilt_rad_ = -angles::from_degrees(cam_ori_ptr->tilt);
  ROS_DEBUG_STREAM("[VSER] Bebop camera's new tilt: " << -cam_ori_ptr->tilt);
}

void BebopVServoCtrl::CameraCallback(const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  cam_model_.fromCameraInfo(cinfo_msg_ptr);
  fov_x_ = 2.0 * atan2(cam_model_.fullResolution().width / 2.0, cam_model_.fx());
  fov_y_ = 2.0 * atan2(cam_model_.fullResolution().height/ 2.0, cam_model_.fy());
  ROS_INFO_STREAM_ONCE("[VSER] FOV " << angles::to_degrees(fov_x_) << " " << angles::to_degrees(fov_y_));

  bool is_valid_roi = (ros::Time::now() - target_recv_time_).toSec() < (5.0 / param_update_freq_);
  sensor_msgs::RegionOfInterest roi;
  if (is_valid_roi && target_cptr_) roi = target_cptr_->roi;
  if (roi.width < 5 || roi.height < 5 ||
      (roi.x_offset + roi.width >= cam_model_.fullResolution().width) ||
      (roi.y_offset + roi.width >= cam_model_.fullResolution().height))
  {
    is_valid_roi = false;
  }

  if (is_valid_roi && (!servo_inited_ || target_cptr_->reinit))
  {
    servo_desired_depth_ = target_cptr_->desired_depth;
    servo_desired_yaw_rad_ = target_cptr_->desired_yaw_rad;
    servo_target_height_ = target_cptr_->target_height;
    servo_target_dist_ground_ = target_cptr_->target_distance_ground;

    ROS_WARN_STREAM("[VSER] (re-)init " <<
                    "Desired depth: " << servo_desired_depth_ <<
                    "Desired yaw:" << angles::to_degrees(servo_desired_yaw_rad_) <<
                    "Target height: " << servo_target_height_ <<
                    "Target d2 ground: " << servo_target_dist_ground_
                    );

    vp_cam_ = visp_bridge::toVispCameraParameters(*cinfo_msg_ptr);
    vp_cam_.printParameters();
    //lambda_adapt.initStandard(4.0, 0.4, 40.0);

    vpFeatureBuilder::create(fp_[0], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset));
    vpFeatureBuilder::create(fp_[1], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
    vpFeatureBuilder::create(fp_[2], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
    vpFeatureBuilder::create(fp_[3], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

    vpPoint point[4];
    const double tw = servo_target_height_ / 2.0;
    point[0].setWorldCoordinates(-tw, -tw, 0);
    point[1].setWorldCoordinates( tw, -tw, 0);
    point[2].setWorldCoordinates( tw,  tw, 0);
    point[3].setWorldCoordinates(-tw,  tw, 0);

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

    for (uint32_t i = 0; i < 4; i++)
    {
      vp_task_.addFeature(fp_[i], fpd_[i]);
    }

    vp_task_.setServo(vpServo::EYEINHAND_CAMERA);
    vp_task_.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    vp_task_.setLambda(vp_gain_);
    vp_task_.print();
    servo_inited_ = true;
  }

  if (!servo_inited_ && enabled_)
  {
    ROS_WARN_THROTTLE(1, "[VSER] Wating for the first roi ...");
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
  if (!servo_inited_)
  {
    ROS_WARN_THROTTLE(1, "[VSER] Servo task has not been initialized yet.");
    return false;
  }

  bool is_valid_roi = (ros::Time::now() - target_recv_time_).toSec() < (5.0 / param_update_freq_);
  sensor_msgs::RegionOfInterest roi;
  if (is_valid_roi && target_cptr_) roi = target_cptr_->roi;
  if (roi.width < 5 || roi.height < 5 ||
      (roi.x_offset + roi.width >= cam_model_.fullResolution().width) ||
      (roi.y_offset + roi.width >= cam_model_.fullResolution().height))
  {
    is_valid_roi = false;
  }

  if (!is_valid_roi)
  {
    ROS_WARN_THROTTLE(1, "[VSER] ROI is too old or not valid. Disabling visual servo!");
    // Experimental
    enabled_ = false;
    servo_inited_ = false;
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
  vpFeatureBuilder::create(fp_[0], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset));
  vpFeatureBuilder::create(fp_[1], vp_cam_, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
  vpFeatureBuilder::create(fp_[2], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
  vpFeatureBuilder::create(fp_[3], vp_cam_, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

  for (uint32_t i = 0; i < 4; i++)
  {
    fp_[i].set_Z(z1_m);
  }
  vp_v_ = vp_task_.computeControlLaw();
  //vp_task_.print();
  ROS_WARN_STREAM("V_SERVO: " << vp_v_.transpose());

  // Here we should convert from Camera (Visp) coordinates to Bebop Coordinates
  // TODO: Consider camera tilt
  // Servo:           z: forward, x: right, y: down
  // Maps to bebop:   z: x_b    , x: -y_b , y: -z_b

  util::ResetCmdVel(msg_cmd_vel_);

  // No clamp/filter here
  msg_cmd_vel_.linear.x = vp_v_[2];
  msg_cmd_vel_.linear.y = -vp_v_[0];
  msg_cmd_vel_.linear.z = -vp_v_[1];

  // We can't control all four DOF of the vehicle
  msg_cmd_vel_.angular.z = servo_desired_yaw_rad_;

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
  pub_debug_.publish(msg_debug_);
}

}  // namespace bebop_servo
