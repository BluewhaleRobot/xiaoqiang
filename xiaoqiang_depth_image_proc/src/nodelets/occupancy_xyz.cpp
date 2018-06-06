/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <xiaoqiang_depth_image_proc/depth_traits.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ThirdParty/ANN/include/ANN/ANN.h"

//#include <cstdint>  //int8_t
#include <math.h>   //fmod
#include<cstring>

//line function class,uesed to calculate a point position according to the line,
class LineFunc
{
public:
  LineFunc(double p1[],double p2[])
  {
    if(std::fabs(p1[0]-p2[0])<0.000001)
    {
      A_=0;
      B_=1;
      C_=-p1[0];
    }
    else
    {
      A_=1;
      B_=(p2[1]-p1[1])/(p2[0]-p1[0]);
      C_=p1[1]-B_*p1[0];
    }
  }
  //return value>0 meaning point is locating at left or down side of line ,
  // return value==0 meaning point is in the  line
  //return value<0 meaning point is locating at right or top side of line
  double getPointPos(double p[])
  {
    return A_*p[1]-B_*p[0]-C_;
  }
private:
  double A_;
  double B_;
  double C_;
};

namespace xiaoqiang_depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class OccupancyXyzNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  int queue_size_;

  // Publications
  boost::mutex connect_mutex_;
  typedef sensor_msgs::PointCloud2 PointCloud;


  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  void convert2(const sensor_msgs::ImageConstPtr& depth_msg,PointCloud::Ptr& cloud_msg,const image_geometry::PinholeCameraModel& model,double range_max = 0.0);

private:
  double resolution_;  // The map grid resolution [m/cell] ,default 0.1
  std::vector<double>  lowerLeftPos_;// the position of the lower left border  [x,y] ,default [0.4,1.5]
  double gridHeight_; // the  height of map grid ,default 2.0,the width of grid map=2*LowerLeftPos_.y
  double usingWidth_;// the Width length of  lower left border that kinect can seen,default 0.3
  double usingHeight_down_;//the start height length of kinect must seen area
  double usingHeight_up_;//the end height length of kinect must seen area
  int8_t* mapGrid_;// the output occupancy map grid
  int8_t* unkown_mapGrid_;// the init output occupancy map grid
  int8_t* filter_mapGrid_;// the output occupancy map grid filter
  std::vector<int> seenGrid_indexs_;
  std::vector<int> mustseenGrid_indexs_;
  ANNpointArray gridCenters_;
  ANNkd_tree* kdTree_;
  int       ANN_num_   ;       //number of ANNpoint
  int				ANN_k_				;			// number of nearest neighbors
  int				ANN_dim_			;			// dimension
  double			ANN_eps_	 ;			// error bound
  bool      ANN_ready_   ;
  std::vector<double> R_;
  std::vector<double> T_;
  double kinectHeight_;
  double barHeight_max_;
  double barHeight_min_;
  ros::Publisher pub_occupancy_grid_;
  ros::Publisher pub_barpoint_cloud_;
  ros::Publisher pub_clearpoint_cloud_;
  ros::Publisher pub_point_cloud_;
  int point_mode_; //0 完整点云、1 除去地板后的所有可视点云
  std::string frame_id_;
  nav_msgs::OccupancyGrid  occupancygrid_msg_;
  double height_temp_;
  double width_temp_;
  unsigned int num_frame_;
  double bar_threshold_;
  double single_score_;
};
void OccupancyXyzNodelet::convert2(const sensor_msgs::ImageConstPtr& depth_msg,PointCloud::Ptr& cloud_msg,const image_geometry::PinholeCameraModel& model,double range_max)
{

  //static last_mapGrid =new  int8_t[2*height_temp_*width_temp_];
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  float unit_scaling = DepthTraits<uint16_t>::toMeters( uint16_t(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(uint16_t);
  float x,y,z;
  ANNpoint p;
  ANNpoint			queryPt=annAllocPt(ANN_dim_);					// allocate query point;
  ANNidxArray			nnIdx;					// near neighbor indices
  ANNdistArray		dists;					// near neighbor distances
  nnIdx = new ANNidx[ANN_k_];						// allocate near neigh indices
  dists = new ANNdist[ANN_k_];						// allocate near neighbor dists
  float resolution_square=resolution_*resolution_;
  int index_temp1,index_temp2;
  //boost::lock_guard<boost::mutex> lock(connect_mutex_);
  // for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; ++it)
  // {
  //
  //   index_temp1=*it;
  //   mapGrid_[index_temp1]=-1;
  // }
  int x_num,y_num;
  float float_temp;
  ANNpoint PP;
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; u+=2, ++iter_x, ++iter_y, ++iter_z)
    {
      uint16_t depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<uint16_t>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<uint16_t>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      x = (u - center_x) * depth * constant_x;
      y = (v - center_y) * depth * constant_y;
      z = DepthTraits<uint16_t>::toMeters(depth);

      *iter_x = R_[0]*x+R_[1]*y+R_[2]*z;
      *iter_y = R_[3]*x+R_[4]*y+R_[5]*z;
      *iter_z = R_[6]*x+R_[7]*y+R_[8]*z+kinectHeight_;

      x=*iter_x;
      y=*iter_y;
      z=*iter_z;

      //地板扣除 障碍物识别
     if(z< barHeight_max_)
     {
       if(z< barHeight_min_)
       {
         //地板
         if(point_mode_==1)
         {
           //ROS_INFO("get the floor\n");
           *iter_x = *iter_y = *iter_z = bad_point;
         }
         //除去不感兴趣区域
         if(!((x<(gridHeight_+lowerLeftPos_[0]) &&x>lowerLeftPos_[0])&&(y<lowerLeftPos_[1] && y> -lowerLeftPos_[1]))) continue;
         float_temp=(x-lowerLeftPos_[0])/resolution_;
         x_num=static_cast<int>(float_temp);
         float_temp=y/resolution_+width_temp_;
         y_num=static_cast<int>(float_temp);
         index_temp1=x_num+y_num*height_temp_;
         if(mapGrid_[index_temp1]<0) mapGrid_[index_temp1]=0;
       }
       else
       {
         //障碍物
         //除去不感兴趣区域
          if(!((x<(gridHeight_+lowerLeftPos_[0]) &&x>lowerLeftPos_[0])&&(y<lowerLeftPos_[1] && y> -lowerLeftPos_[1]))) continue;
          float_temp=(x-lowerLeftPos_[0])/resolution_;
          x_num=static_cast<int>(float_temp);
          float_temp=y/resolution_+width_temp_;
          y_num=static_cast<int>(float_temp);
          index_temp1=x_num+y_num*height_temp_;
          mapGrid_[index_temp1]+=single_score_;
          if(mapGrid_[index_temp1]>100) mapGrid_[index_temp1]=100;
        //  queryPt[0]=*iter_x;
        //  queryPt[1]=*iter_y;
        //  kdTree_->annkSearch(						// search
        //     queryPt,						// query point
        //     ANN_k_,								// number of near neighbors
        //     nnIdx,							// nearest neighbors (returned)
        //     dists,							// distance (returned)
        //     0.0);							// error bound
        //  if(dists[0]<=(resolution_square))
        //  {
        //    index_temp1=seenGrid_indexs_[nnIdx[0]];
        //    mapGrid_[index_temp1]+=10;
        //    if(mapGrid_[index_temp1]>100) mapGrid_[index_temp1]=100;
        //  }


       }
     }
    }
  }
  //更新必须看见区域

  for(std::vector<int>::iterator it = mustseenGrid_indexs_.begin() ; it !=mustseenGrid_indexs_.end() ; it++)
  {
    index_temp1=*it;
    if(mapGrid_[index_temp1]<0) //mapGrid_[index_temp1]=100;
    {
      filter_mapGrid_[index_temp1]-=1;
      if(filter_mapGrid_[index_temp1]<-10) filter_mapGrid_[index_temp1]=-10;
    }
    else
    {
      filter_mapGrid_[index_temp1]+=1;
      if(filter_mapGrid_[index_temp1]>0) filter_mapGrid_[index_temp1]=0;
    }
    if(filter_mapGrid_[index_temp1]<-9)
    {
      mapGrid_[index_temp1]=100;
    }
  }
  //将可见区域分成可移动区和雷区
  std::vector<int> clearArea,barArea;
  index_temp2=0;
  for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; it++)
  {
    index_temp1=*it;
    if(mapGrid_[index_temp1]<bar_threshold_&&mapGrid_[index_temp1]>=0)//被遮挡区域不发布
    {
      clearArea.push_back(index_temp2);
    }
    else if(mapGrid_[index_temp1]>=bar_threshold_)
    {
      barArea.push_back(index_temp2);
    }
    index_temp2++;
  }

  if(barArea.size()>1||(barArea.size()==1&&num_frame_%4==0))
  {
    //发布雷区
    PointCloud::Ptr barcloud_msg(new PointCloud);
    barcloud_msg->header = depth_msg->header;
    barcloud_msg->height = 1;
    barcloud_msg->width  = barArea.size();
    barcloud_msg->is_dense = true;
    barcloud_msg->is_bigendian = false;
    barcloud_msg->header.frame_id=frame_id_;
    sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
    pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
    sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
    for(std::vector<int>::iterator it = barArea.begin() ; it !=barArea.end() ; ++bariter_x, ++bariter_y,++bariter_z,it++)
    {
      index_temp1=*it;
      PP=gridCenters_[index_temp1];
      *bariter_x=PP[0];
      *bariter_y=PP[1];
      *bariter_z=0.15;
    }
    pub_barpoint_cloud_.publish(barcloud_msg);
  }
  // else
  // {
  //   //发布空雷区
  //   PointCloud::Ptr barcloud_msg(new PointCloud);
  //   barcloud_msg->header = depth_msg->header;
  //   barcloud_msg->height = 1;
  //   barcloud_msg->width  = 1;
  //   barcloud_msg->is_dense = true;
  //   barcloud_msg->is_bigendian = false;
  //   barcloud_msg->header.frame_id=frame_id_;
  //   sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
  //   pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
  //   sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
  //   *bariter_x=-1.0;
  //   *bariter_y=-1.0;
  //   *bariter_z=-1.0;
  //   pub_barpoint_cloud_.publish(barcloud_msg);
  // }
  if(clearArea.size()>0)
  {
    //发布可移动区域
    PointCloud::Ptr clearcloud_msg(new PointCloud);
    clearcloud_msg->header = depth_msg->header;
    clearcloud_msg->height = 1;
    clearcloud_msg->width  = clearArea.size();
    clearcloud_msg->is_dense = true;
    clearcloud_msg->is_bigendian = false;
    clearcloud_msg->header.frame_id=frame_id_;
    sensor_msgs::PointCloud2Modifier pcd_modifier2(*clearcloud_msg);
    pcd_modifier2.setPointCloud2FieldsByString(1, "xyz");
    sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
    for(std::vector<int>::iterator it = clearArea.begin() ; it !=clearArea.end() ; ++cleariter_x, ++cleariter_y,it++)
    {
      index_temp1=*it;
      PP=gridCenters_[index_temp1];
      *cleariter_x=PP[0];
      *cleariter_y=PP[1];
      *cleariter_z=mapGrid_[seenGrid_indexs_[index_temp1]];
    }
    pub_clearpoint_cloud_.publish(clearcloud_msg);
  }

}

void OccupancyXyzNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));
  num_frame_=1;
  // Read parameters
  private_nh.getParam("lowerLeftPos", lowerLeftPos_);
  private_nh.getParam("R", R_);
  private_nh.getParam("T", T_);
  private_nh.param("resolution", resolution_, 0.1);
  private_nh.param("gridHeight", gridHeight_, 2.0);
  private_nh.param("usingWidth", usingWidth_, 0.3);
  nh.param("usingHeight_down",usingHeight_down_, 0.10);
  nh.param("usingHeight_up",usingHeight_up_, 0.25);
  private_nh.param("frame_id",frame_id_,std::string("kinect_link_new"));
  private_nh.param("kinectHeight", kinectHeight_, 0.3769);
  private_nh.param("barHeight_max", barHeight_max_, 0.50);
  private_nh.param("barHeight_min", barHeight_min_, 0.05);
  private_nh.param("point_mode", point_mode_, 0);
  private_nh.param("bar_threshold", bar_threshold_, 10.0);
  private_nh.param("single_score", single_score_, 2.0);
  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&OccupancyXyzNodelet::connectCb, this);

  //init map grid
  ANN_k_ =1				;			// number of nearest neighbors
  ANN_dim_	=2		;			// dimension
  ANN_eps_ =0.0	 ;			// error bound
  ANN_ready_  =false ;

  //计算网格大小
  int width_temp,height_temp;

  if(fmod(lowerLeftPos_[1],resolution_)>0.001)
  {
    width_temp=lowerLeftPos_[1]/resolution_+1;
  }
  else
  {
    width_temp=lowerLeftPos_[1]/resolution_;
  }
  if(fmod(gridHeight_,resolution_)>0.001)
  {
    height_temp=gridHeight_/resolution_+1;
  }
  else
  {
    height_temp=gridHeight_/resolution_;
  }
  width_temp =std::max(width_temp,1);
  height_temp =std::max(height_temp,1);
  mapGrid_=new  int8_t[2*height_temp*width_temp];
  unkown_mapGrid_=new  int8_t[2*height_temp*width_temp];
  filter_mapGrid_=new  int8_t[2*height_temp*width_temp];
  int x_num,y_num;
  float x,y; //格子中心点坐标
  double p1[2]={lowerLeftPos_[0],usingWidth_},p2[2]={lowerLeftPos_[0]+gridHeight_,lowerLeftPos_[1]};
  double p3[2]={lowerLeftPos_[0],-usingWidth_},p4[2]={lowerLeftPos_[0]+gridHeight_,-lowerLeftPos_[1]};
  //设置视野边界线
  LineFunc leftborder= LineFunc(p1,p2);
  LineFunc rightborder= LineFunc(p3,p4);
  //初始化网格和视野内网格
  double p[2]={0.0,0.0};
  for(int i=0;i<(2*height_temp*width_temp);i++)
  {
    mapGrid_[i]=-1;//init as Unknown area
    unkown_mapGrid_[i]=-1;
    filter_mapGrid_[i]=-1;
    y_num=i/(height_temp);
    x_num=i%(height_temp);
    x=(x_num+0.5)*resolution_+lowerLeftPos_[0];
    y=(-width_temp+y_num+0.5)*resolution_;
    p[0]=x;
    p[1]=y;
    if((leftborder.getPointPos(p)*rightborder.getPointPos(p))>0.000001)
    {
      continue;
    }
    else
    {
      //NODELET_INFO("height_temp: %d  width_temp: %d p1:%f %f p2:%f %f p:%f %f i:%d x_num:%d y_num:%d\n",height_temp,width_temp,p1[0],p1[1],p2[0],p2[1],p[0],p[1],i,x_num,y_num);
      seenGrid_indexs_.push_back(i);
      if(x<(usingHeight_up_+lowerLeftPos_[0]) && x>(usingHeight_down_+lowerLeftPos_[0]) ) mustseenGrid_indexs_.push_back(i);
    }
  }
  //将视野内的网格中心点存入ANN,加速后续算法
  ANN_num_=seenGrid_indexs_.size();
  gridCenters_= annAllocPts(ANN_num_, ANN_dim_);			// allocate data points
  ANNpoint pp;
  int num=0;
  for(std::vector<int>::iterator it = seenGrid_indexs_.begin() ; it !=seenGrid_indexs_.end() ; it++)
  {
    int i=*it;
    pp=gridCenters_[num];
    y_num=i/(height_temp);
    x_num=i%(height_temp);
    pp[0]=(x_num+0.5)*resolution_+lowerLeftPos_[0];
    pp[1]=(-width_temp+y_num+0.5)*resolution_;
    //NODELET_INFO("num: %d index: %d x_num: %d y_num: %d\n",num,i,x_num,y_num);
    num++;
  }
  kdTree_ = new ANNkd_tree(					// build search structure
          gridCenters_,					// the data points
          ANN_num_,						// number of points
          ANN_dim_);						// dimension of space
  ANN_ready_=true;

  occupancygrid_msg_.info.resolution=resolution_;
  occupancygrid_msg_.info.height=2*width_temp;
  occupancygrid_msg_.info.width=height_temp;
  occupancygrid_msg_.info.origin.position.x=lowerLeftPos_[0];
  occupancygrid_msg_.info.origin.position.y=-width_temp*resolution_;
  occupancygrid_msg_.info.origin.position.z=0;


  height_temp_=height_temp;
  width_temp_=width_temp;
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  pub_occupancy_grid_ = nh.advertise<nav_msgs::OccupancyGrid>("occupancygrid", 1,connect_cb, connect_cb);
  pub_barpoint_cloud_ = nh.advertise<PointCloud>("barpoints", 1, connect_cb, connect_cb);
  pub_clearpoint_cloud_ = nh.advertise<PointCloud>("clearpoints", 1, connect_cb, connect_cb);


}

// Handles (un)subscribing when clients (un)subscribe
void OccupancyXyzNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if ((pub_point_cloud_.getNumSubscribers() == 0 )&& (pub_barpoint_cloud_.getNumSubscribers() ==0) && (pub_clearpoint_cloud_.getNumSubscribers() ==0) && (pub_occupancy_grid_.getNumSubscribers()==0))
  {
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect", queue_size_, &OccupancyXyzNodelet::depthCb, this, hints);
  }
}

void OccupancyXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{

  if (!ANN_ready_) return;
  PointCloud::Ptr cloud_msg(new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  cloud_msg->header.frame_id=frame_id_;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  std::vector<int8_t> a(height_temp_*2*width_temp_,-1);
  occupancygrid_msg_.data = a;
  memcpy(mapGrid_,unkown_mapGrid_,sizeof(int8_t)*(occupancygrid_msg_.info.height*occupancygrid_msg_.info.width));
  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    this->convert2(depth_msg, cloud_msg, model_);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }
  // Time
  ros::Time current_time = ros::Time::now();
  if((num_frame_%2==0))
  {
    num_frame_++;
    return;//10hz
  }
  num_frame_++;
  occupancygrid_msg_.header = depth_msg->header;
  occupancygrid_msg_.header.frame_id=frame_id_;
  occupancygrid_msg_.info.map_load_time=current_time;
  memcpy(&occupancygrid_msg_.data[0],mapGrid_,sizeof(int8_t)*(occupancygrid_msg_.info.height*occupancygrid_msg_.info.width));
  pub_point_cloud_.publish (cloud_msg);
  pub_occupancy_grid_.publish(occupancygrid_msg_);
}

} // namespace xiaoqiang_depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(xiaoqiang_depth_image_proc::OccupancyXyzNodelet,nodelet::Nodelet);
