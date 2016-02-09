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
    servo_inited(false),
    nh_(nh),
    nh_priv_("~"),
    caminfo_sub(nh.subscribe("bebop/camera_info", 1, &BebopVServoCtrl::CameraCallback, this)),
    cam_orientation_sub(nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10
                                      , &BebopVServoCtrl::CameraOrientationCallback, this)),
    roi_sub(nh_.subscribe("track_roi", 1, &BebopVServoCtrl::RoiCallback, this)),
    enable_sub(nh_.subscribe("enable", 1, &BebopVServoCtrl::EnableCallback, this)),
    cmd_vel_pub(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
    roi_recv_time(ros::Time(0)),
    fov_x(0.0),
    fov_y(0.0),
    cam_tilt_rad(0.0),
    t_start_loop(0)

{
  UpdateParams();
  util::ResetCmdVel(cmd_vel);
}

void BebopVServoCtrl::UpdateParams()
{
  util::GetParam(nh_priv_, "desired_depth", depth, 2.5);
  util::GetParam(nh_priv_, "start_enabled", enabled_, false);
  util::GetParam(nh_priv_, "servo_gain", lambda, 0.4);
  util::GetParam(nh_priv_, "update_freq", update_freq, 30.0);
  util::GetParam(nh_priv_, "height_target", height_target_m, 0.5);
  util::GetParam(nh_priv_, "distground_target", distground_target_m, 0.75);
  util::GetParam(nh_priv_, "cam_tilt", cam_tilt_rad, 0.0);
  cam_tilt_rad = angles::from_degrees(cam_tilt_rad);
}

void BebopVServoCtrl::EnableCallback(const std_msgs::BoolConstPtr &enable_msg_ptr)
{
  enabled_ptr_ = enable_msg_ptr;
  ROS_INFO_STREAM("[VSER] Enable request: " << enable_msg_ptr->data);
}

void BebopVServoCtrl::RoiCallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr)
{
  roi_cptr_ = roi_msg_ptr;
  roi_recv_time = ros::Time::now();
}

void BebopVServoCtrl::CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  cam_tilt_rad = -angles::from_degrees(cam_ori_ptr->tilt);
  ROS_INFO_STREAM("[VSER] Bebop camera's new tilt: " << -cam_ori_ptr->tilt);
}

void BebopVServoCtrl::CameraCallback(const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  cam_model.fromCameraInfo(cinfo_msg_ptr);
  fov_x = 2.0 * atan2(cam_model.fullResolution().width / 2.0, cam_model.fx());
  fov_y = 2.0 * atan2(cam_model.fullResolution().height/ 2.0, cam_model.fy());
  ROS_INFO_STREAM_ONCE("[VSER] FOV " << angles::to_degrees(fov_x) << " " << angles::to_degrees(fov_y));

  bool is_valid_roi = (ros::Time::now() - roi_recv_time).toSec() < (5.0 / update_freq);
  sensor_msgs::RegionOfInterest roi;
  if (is_valid_roi && roi_cptr_) roi = *roi_cptr_;
  if (roi.width < 5 || roi.height < 5 ||
      (roi.x_offset + roi.width >= cam_model.fullResolution().width) ||
      (roi.y_offset + roi.width >= cam_model.fullResolution().height))
  {
    is_valid_roi = false;
  }

  if (!servo_inited && is_valid_roi)
  {
    cam = visp_bridge::toVispCameraParameters(*cinfo_msg_ptr);
    cam.printParameters();
    //lambda_adapt.initStandard(4.0, 0.4, 40.0);

    vpFeatureBuilder::create(p[0], cam, vpImagePoint(roi.y_offset, roi.x_offset));
    vpFeatureBuilder::create(p[1], cam, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
    vpFeatureBuilder::create(p[2], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
    vpFeatureBuilder::create(p[3], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

    vpPoint point[4];
    const double tw = height_target_m / 2.0;
    point[0].setWorldCoordinates(-tw, -tw, 0);
    point[1].setWorldCoordinates( tw, -tw, 0);
    point[2].setWorldCoordinates( tw,  tw, 0);
    point[3].setWorldCoordinates(-tw,  tw, 0);

    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, depth);
    vpRxyzVector cro(vpMath::rad(0.0), vpMath::rad(0.0), vpMath::rad(0.0));
    vpRotationMatrix cRo(cro);
    cMo.buildFrom(cto, cRo);

    for (uint32_t i = 0; i < 4; i++)
    {
      vpColVector cP, p_img;
      point[i].changeFrame(cMo, cP);
      point[i].projection(cP, p_img);

      pd[i].set_xyZ(p_img[0], p_img[1], cP[2]);
    }

    for (uint32_t i = 0; i < 4; i++)
    {
      task.addFeature(p[i], pd[i]);
    }

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    task.setLambda(lambda);
    task.print();
    servo_inited = true;
  }

  if (!servo_inited)
  {
    ROS_WARN_THROTTLE(1, "[VSER] Wating for the first roi ...");
  }
}

void BebopVServoCtrl::Reset()
{
  util::ResetCmdVel(cmd_vel);
  cmd_vel_pub.publish(cmd_vel);
}

void BebopVServoCtrl::Spin()
{
  ROS_INFO("[VSER] Control loop started ...");

  ros::Rate loop_rate(update_freq);
  while (ros::ok())
  {
    try
    {
      if (!loop_rate.sleep())
      {
        throw std::runtime_error("[VSER] Missed target frequency");
      }

      if (enabled_ptr_)
      {
        enabled_ = enabled_ptr_->data;
      }

      if (!enabled_)
      {
        ROS_WARN_THROTTLE(1, "[VSER] Not enabled ...");
        continue;
      }
      Update();
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

void BebopVServoCtrl::Update()
{
  bool is_valid_roi = (ros::Time::now() - roi_recv_time).toSec() < (5.0 / update_freq);
  sensor_msgs::RegionOfInterest roi;
  if (is_valid_roi && roi_cptr_) roi = *roi_cptr_;
  if (roi.width < 5 || roi.height < 5 ||
      (roi.x_offset + roi.width >= cam_model.fullResolution().width) ||
      (roi.y_offset + roi.width >= cam_model.fullResolution().height))
  {
    is_valid_roi = false;
  }

  if (!is_valid_roi)
  {
    ROS_WARN_THROTTLE(1, "[VSER] ROI is too old or not valid");
    Reset();
    return;
  }
  const double im_width_px = static_cast<double>(cam_model.fullResolution().width);
  const double im_height_px = static_cast<double>(cam_model.fullResolution().height);
  double sigma_1 = static_cast<double>(roi.y_offset) / im_height_px * fov_y;
  double sigma_2 = (im_height_px - static_cast<double>(roi.y_offset + roi.height)) / im_height_px * fov_y;
  double roi_height_px = roi.height;
  double beta_rad = (roi_height_px / im_height_px) * fov_y;
  double d1_m = height_target_m * sin(M_PI_2 - fov_y/2.0 + sigma_2 - cam_tilt_rad) / sin(beta_rad);
  double z1_m = d1_m * sin(M_PI_2 + fov_y/2.0  - cam_tilt_rad - sigma_1);
  vpFeatureBuilder::create(p[0], cam, vpImagePoint(roi.y_offset, roi.x_offset));
  vpFeatureBuilder::create(p[1], cam, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
  vpFeatureBuilder::create(p[2], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
  vpFeatureBuilder::create(p[3], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

  for (uint32_t i = 0; i < 4; i++)
  {
    p[i].set_Z(z1_m);
  }
  v = task.computeControlLaw();
  task.print();
  ROS_WARN_STREAM("V_SERVO\n" << v);

  // Here we should convert from Camera (Visp) coordinates to Bebop Coordinates
  // TODO: Consider camera tilt
  // Servo:           z: forward, x: right, y: down
  // Maps to bebop:   z: x_b    , x: -y_b , y: -z_b

  util::ResetCmdVel(cmd_vel);

  // No clamp/filter here
  cmd_vel.linear.x = v[2];
  cmd_vel.linear.y = -v[0];
  cmd_vel.linear.z = -v[1];
  cmd_vel.angular.z = -v[4];

  cmd_vel_pub.publish(cmd_vel);
}

}  // namespace bebop_servo
