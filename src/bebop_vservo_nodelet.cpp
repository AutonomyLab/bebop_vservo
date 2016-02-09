// c++
#include <cmath>

// boost
#include <boost/thread/lock_guard.hpp>

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
    roi_recv_time(ros::Time(0)),
    fov_x(0.0),
    fov_y(0.0),
    cam_tilt_rad(0.0),
    t_start_loop(0)

{
}

BebopVServoNodelet::~BebopVServoNodelet()
{
  // nodelet log macros do not work here!

  if (ctrlloop_thread_ptr_)
  {
    ctrlloop_thread_ptr_->interrupt();
    ctrlloop_thread_ptr_->join();
  }
  std::cout << "[CVER] Graceful shutdown" << std::endl;
}

void BebopVServoNodelet::UpdateParams()
{
  util::GetParam(private_nh_, "desired_depth", depth, 2.5);
  util::GetParam(private_nh_, "start_enabled", enabled_, false);
  util::GetParam(private_nh_, "servo_gain", lambda, 0.4);
  util::GetParam(private_nh_, "update_freq", update_freq, 30.0);
  util::GetParam(private_nh_, "height_target", height_target_m, 0.5);
  util::GetParam(private_nh_, "distground_target", distground_target_m, 0.75);
  util::GetParam(private_nh_, "cam_tilt", cam_tilt_rad, 0.0);
  cam_tilt_rad = angles::from_degrees(cam_tilt_rad);
}

void BebopVServoNodelet::EnableCallback(const std_msgs::BoolConstPtr &enable_msg_ptr)
{
  boost::lock_guard<boost::mutex> lock(mutex_enable_);
  enabled_ptr_ = enable_msg_ptr;
  ROS_INFO_STREAM("[VSER] Enable request: " << enable_msg_ptr->data);
}

void BebopVServoNodelet::RoiCallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr)
{
  boost::lock_guard<boost::mutex> lock(mutex_roi_);
  roi_cptr_ = roi_msg_ptr;
  roi_recv_time = ros::Time::now();
}

void BebopVServoNodelet::CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  boost::lock_guard<boost::mutex> lock(mutex_servo);
  cam_tilt_rad = -angles::from_degrees(cam_ori_ptr->tilt);
  NODELET_INFO_STREAM("[VSER] Bebop camera's new tilt: " << -cam_ori_ptr->tilt);
}

void BebopVServoNodelet::CameraCallback(const sensor_msgs::ImageConstPtr& img_ptr,
                                        const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  {
    boost::lock_guard<boost::mutex> lock(mutex_servo);
    cam_model.fromCameraInfo(cinfo_msg_ptr);
    fov_x = 2.0 * atan2(cam_model.fullResolution().width / 2.0, cam_model.fx());
    fov_y = 2.0 * atan2(cam_model.fullResolution().height/ 2.0, cam_model.fy());
    NODELET_INFO_STREAM_ONCE("[VSER] FOV " << angles::to_degrees(fov_x) << " " << angles::to_degrees(fov_y));
  }

  bool is_valid_roi = false;
  sensor_msgs::RegionOfInterest roi_cached;
  {
    boost::lock_guard<boost::mutex> lock(mutex_roi_);
    is_valid_roi = (ros::Time::now() - roi_recv_time).toSec() < (5.0 / update_freq);
    if (is_valid_roi && roi_cptr_) roi_cached = *roi_cptr_;
  }

  {
    boost::lock_guard<boost::mutex> lock(mutex_servo);
    if (!servo_inited && is_valid_roi)
    {
      cam = visp_bridge::toVispCameraParameters(*cinfo_msg_ptr);
      cam.printParameters();
      //lambda_adapt.initStandard(4.0, 0.4, 40.0);

      vpFeatureBuilder::create(p[0], cam, vpImagePoint(roi_cached.y_offset, roi_cached.x_offset));
      vpFeatureBuilder::create(p[1], cam, vpImagePoint(roi_cached.y_offset, roi_cached.x_offset + roi_cached.width));
      vpFeatureBuilder::create(p[2], cam, vpImagePoint(roi_cached.y_offset + roi_cached.height, roi_cached.x_offset + roi_cached.width));
      vpFeatureBuilder::create(p[3], cam, vpImagePoint(roi_cached.y_offset + roi_cached.height, roi_cached.x_offset));

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
      NODELET_WARN_THROTTLE(1, "[VSER] Wating for the first roi ...");
    }
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

  NODELET_INFO_STREAM("[VSER] Image transport in use: " << camera_sub.getTransport());

  cam_orientation_sub = nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10
                                       , &BebopVServoNodelet::CameraOrientationCallback, this);

  roi_sub = nh_.subscribe("track_roi", 1, &BebopVServoNodelet::RoiCallback, this);
  enable_sub = nh_.subscribe("enable", 1, &BebopVServoNodelet::EnableCallback, this);

  cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ctrlloop_thread_ptr_ = boost::make_shared<boost::thread>(
        boost::bind(&BebopVServoNodelet::ControlLoopThread, this));
}

void BebopVServoNodelet::Reset()
{
  util::ResetCmdVel(cmd_vel);
  cmd_vel_pub.publish(cmd_vel);
}

void BebopVServoNodelet::ControlLoopThread()
{
  NODELET_INFO("[VSER] Control loop thread started ...");
  ros::Rate loop_rate(update_freq);
  while (!boost::this_thread::interruption_requested())
  {
    try
    {
      if (!loop_rate.sleep())
      {
        throw std::runtime_error("[VSER] Missed target frequency");
      }

      {
        boost::lock_guard<boost::mutex> lock(mutex_enable_);
        if (enabled_ptr_)
        {
          enabled_ = enabled_ptr_->data;
        }
      }

      if (!enabled_)
      {
        ROS_WARN_THROTTLE(1, "[VSER] Not enabled ...");
        continue;
      }

      bool is_valid_roi = false;
      sensor_msgs::RegionOfInterest roi;
      {
        boost::lock_guard<boost::mutex> lock(mutex_roi_);
        is_valid_roi = (ros::Time::now() - roi_recv_time).toSec() < (5.0 / update_freq);
        if (is_valid_roi && roi_cptr_) roi = *roi_cptr_;
        if (roi.width < 5 || roi.height < 5 ||
            (roi.x_offset + roi.width >= cam_model.fullResolution().width) ||
            (roi.y_offset + roi.width >= cam_model.fullResolution().height))
        {
          is_valid_roi = false;
        }
      }
      if (!is_valid_roi)
      {
        ROS_WARN_THROTTLE(1, "[VSER] ROI is too old or not valid");
        Reset();
        continue;
      }

      {
        boost::lock_guard<boost::mutex> lock(mutex_servo);
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
      }
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[VSER] Runtime error " << e.what());
      Reset();
    }
  }

  NODELET_INFO("[VSER] Control loop thread stopped ...");
}
}  // namespace bebop_servo

PLUGINLIB_DECLARE_CLASS(bebop_vservo, BebopVServoNodelet, bebop_vservo::BebopVServoNodelet, nodelet::Nodelet);
