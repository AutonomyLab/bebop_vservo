// c++
#include <cmath>

// ros
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <visp_bridge/camera.h>

// visp
#include <visp/vpFeatureBuilder.h>

#include "bebop_vservo/bebop_vservo_nodelet.h"

namespace bebop_vservo
{

BebopVServoNodelet::BebopVServoNodelet()
  : enabled_(false),
    servo_inited(false),
    cinfo_recv(false),
    roi_recv_time(ros::Time(0)),
    fov_x(0.0),
    fov_y(0.0),
    cam_tilt_rad(0.0),
    t_start_loop(0)

{
}

void BebopVServoNodelet::UpdateParams()
{
  util::GetParam(private_nh_, "desired_depth", depth, 2.5);
  util::GetParam(private_nh_, "start_enabled", enabled_, false);
  util::GetParam(private_nh_, "servo_gain", lambda, 0.4);
  util::GetParam(private_nh_, "safety_timeout", safety_timeout_, 0.5);
  util::GetParam(private_nh_, "height_target", height_target_m, 0.5);
  util::GetParam(private_nh_, "distground_target", distground_target_m, 0.75);
  util::GetParam(private_nh_, "cam_tilt", cam_tilt_rad, 0.0);
  cam_tilt_rad = angles::from_degrees(cam_tilt_rad);
}

void BebopVServoNodelet::RoiCallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr)
{
  roi = *roi_msg_ptr;
  roi_recv_time = ros::Time::now();
}

void BebopVServoNodelet::CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  cam_tilt_rad = -angles::from_degrees(cam_ori_ptr->tilt);
  ROS_INFO_STREAM("[VSER] Bebop camera's new tilt: " << -cam_ori_ptr->tilt);
}

void BebopVServoNodelet::CameraCallback(const sensor_msgs::ImageConstPtr& img_ptr,
                                        const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  if (!enabled_)
  {
    return;
  }

  cam_model.fromCameraInfo(cinfo_msg_ptr);
  fov_x = 2.0 * atan2(cam_model.fullResolution().width / 2.0, cam_model.fx());
  fov_y = 2.0 * atan2(cam_model.fullResolution().height/ 2.0, cam_model.fy());

  ROS_INFO_STREAM_ONCE("[VSER] FOV " << angles::to_degrees(fov_x) << " " << angles::to_degrees(fov_y));
  cinfo_recv = true;

  if ((!servo_inited) && ((ros::Time::now() - roi_recv_time).toSec() < safety_timeout_))
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
    ROS_WARN("[VSER] Wating for the first roi ...");
  }

}
void BebopVServoNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  UpdateParams();
  util::ResetCmdVel(cmd_vel);

  it_ptr_ = boost::make_shared<image_transport::ImageTransport>(nh_);

  // In Nodelet version, the default transport hint does not use Nodelet's local and
  // private Nodehandles
  image_transport::TransportHints it_th("raw", ros::TransportHints(), private_nh_);

  camera_sub = it_ptr_->subscribeCamera("bebop/image_raw", 1, &BebopVServoNodelet::CameraCallback, this);

  NODELET_WARN_STREAM("[CFTLD] Image transport in use: " << camera_sub.getTransport());

  cam_orientation_sub = nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10
                                       , &BebopVServoNodelet::CameraOrientationCallback, this);

  roi_sub = nh_.subscribe("track_roi", 1, &BebopVServoNodelet::RoiCallback, this);
  enable_sub = nh_.subscribe("enable", 1, &BebopVServoNodelet::EnableCallback, this);

  cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

}  // namespace bebop_servo

PLUGINLIB_DECLARE_CLASS(bebop_vservo, BebopVServoNodelet, bebop_vservo::BebopVServoNodelet, nodelet::Nodelet);
