#include <ros/ros.h>
#include <kanu_msgs/WheelMsg.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include <vector>
#include <cmath>

const std::string ODOM_NODE_NAME("/camera/aligned_depth_to_color/image_raw");
const std::string GPS_NODE_NAME("/camera/aligned_depth_to_color/image_raw");
const std::string THIS_NODE_NAME("localization_node");
const std::string POS_XY_PUB_NODE_NAME("posXYPub");
const std::string POS_XY_GPS_PUB_NODE_NAME("posXYGPSPub");
const uint8_t ODOM_MSG_BUFFER_SZ = 1;
const uint8_t GPS_MSG_BUFFER_SZ = 1;
const uint8_t POS_XY_MSG_BUFFER_SZ = 1;
const uint8_t POS_XY_GPS_MSG_BUFFER_SZ = 1;
const double standardAxisAngle = -M_PI*2/3; //from north
const double standardOriginLat = 37.296995568506205;
const double standardOriginLon = 126.83533095292523;
const double LAT_2_METER = 69.1*1609.344;
const double LON_2_METER = LAT_2_METER*cos(standardOriginLat/57.3);
const double dtOdom = 0.1;  //s
const double gpsVarX = pow(1, 2); //m ^2
const double gpsVarY = pow(1, 2);
const double initPosX = 0;  //m
const double initPosY = 0;  //m
const double initVelX = 0;  //m/s
const double initVelY = 0;  //m/s
const double initValPosX = pow(5, 2); //m ^2
const double initValPosY = pow(5, 2); //m ^2
const double initValVelX = pow(1, 2); //m/s ^2
const double initValVelY = pow(1, 2); //m/s ^2
const double unitValOdomPosX = pow(0.03, 2);  //(m ^2)/s
const double unitValOdomPosY = pow(0.03, 2);  //(m ^2)/s
const double valOdomVelX = pow(0.01, 2);  //m/s ^2
const double valOdomVelY = pow(0.01, 2);  //m/s ^2
const uint8_t kalmanN = 4;  //num of x
const uint8_t kalmanM = 2;  //num of z
const uint8_t kalmanxPosXIdx = 0;
const uint8_t kalmanxPosYIdx = 1;
const uint8_t kalmanxVelXIdx = 2;
const uint8_t kalmanxVelYIdx = 3;
const uint8_t MAX_OLD_ELEMENT_SZ = 4;
const double OLD_ELEMENT_IDX_WEIGHT = 1/((MAX_OLD_ELEMENT_SZ+1)*MAX_OLD_ELEMENT_SZ/2);
const double LPF_COEFFICENT = 0.5;
const double robotW = 0.5;  //m
tf::Vector3 zVec = tf::Vector3(0.0, 0.0, 1.0);



struct XY;
struct GPSData;

struct XY
{
  double x;
  double y;
  XY(double x, double y);
  XY(void);
  GPSData Cvt2GPSData(void);
};

struct GPSData
{
  double lat;
  double lon;
  GPSData(double lat, double lon);
  GPSData(void);
  XY Cvt2XY(void);
};

class LocalizationNode
{
private:
  XY posXY;
  double theta;
  double oldTheta;
  std::vector<XY> oldPosXYVec;
  GPSData posXYGPS;
  XY velXY;
  ros::Time recentGPSTime;
  ros::Publisher posXYPub;
  ros::Publisher posXYGPSPub;
  double yaw;
  std::vector<double> oldYawVec;
  Eigen::MatrixXd kalmanA;
  Eigen::MatrixXd kalmanQ;
  Eigen::MatrixXd kalmanH;
  Eigen::MatrixXd kalmanR;
  Eigen::MatrixXd kalmanx;
  Eigen::MatrixXd kalmanP;
  bool isReceivedOdomData;
  bool isReceivedGPSData;
  bool isReceivedYaw;

  void CallbackOdom(const kanu_msgs::WheelMsg::ConstPtr& wheelMsg);
  void CallbackGPS(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg);
  void UpadtePosXYByKalmanFilter(const Eigen::MatrixXd& kalmanz);
  void EstimateXY(const kanu_msgs::WheelMsg::ConstPtr& wheelMsg);
  void SendMsg(void);
  void EstimateTheta(void);
  double CalcTheta(const int idx);
  void LowPassFilt(double& newValue, const double oldValue);


public:
  LocalizationNode(int argc, char **argv);
};

XY::XY(const double x, const double y)
{
  this->x = x;
  this->y = y;
}

XY::XY(void){}

GPSData::GPSData(void){}

GPSData::GPSData(const double lat, const double lon)
{
  this->lat = lat;
  this->lon = lon;
}

XY GPSData::Cvt2XY(void)
{
  double latDist = LAT_2_METER*(this->lat-standardOriginLat);
  double lonDist = LON_2_METER*(this->lon-standardOriginLon);
  auto gpsXYVec = tf::Vector3(lonDist, latDist, 0.0).rotate(zVec, standardAxisAngle);

  return XY(-gpsXYVec.getX(), -gpsXYVec.getY());
}

GPSData XY::Cvt2GPSData(void)
{
  auto gpsXYVec = tf::Vector3(-this->x, -this->y, 0.0).rotate(zVec, -standardAxisAngle);
  double lat = gpsXYVec.getY()/LAT_2_METER+standardOriginLat;
  double lon = gpsXYVec.getX()/LON_2_METER+standardOriginLon;

  return GPSData(lat, lon);
}


void LocalizationNode::LowPassFilt(double& newValue, const double oldValue)
{
  newValue = LPF_COEFFICENT*oldValue+(1-LPF_COEFFICENT)*newValue;
}

void LocalizationNode::EstimateXY(const kanu_msgs::WheelMsg::ConstPtr& wheelMsg)
{
  bool isPositiveTurn;
  double omega, dx, dy, dTheta;

  if(wheelMsg->velR == 0 && wheelMsg->velL == 0)
  {
    dx = dy = dTheta = 0;
  }
  else if(wheelMsg->velR*wheelMsg->velL > 0)
  {
    isPositiveTurn = wheelMsg->velR > wheelMsg->velL ? true : false;
    double diffVel = abs(wheelMsg->velR-wheelMsg->velL);
    double linearVel = isPositiveTurn ? wheelMsg->velR-diffVel : wheelMsg->velL-diffVel;
    omega = diffVel/robotW;
    dTheta = omega*dtOdom;
    dx = linearVel*dtOdom+robotW/2*sin(dTheta);
    dy = robotW/2*(1-cos(dTheta));
    dy = isPositiveTurn ? dy : -dy;
    dTheta = isPositiveTurn ? dTheta : -dTheta;
  }
  else if(wheelMsg->velR*wheelMsg->velL < 0)
  {
    isPositiveTurn = wheelMsg->velR > wheelMsg->velL ? true : false;
    bool isGoFoward = wheelMsg->velR+wheelMsg->velL > 0 ? true : false;
    float rotationCenter =  robotW*(1/2-abs(wheelMsg->velL)/(abs(wheelMsg->velR)+abs(wheelMsg->velL)));
    omega = abs(wheelMsg->velL)/(robotW/2-rotationCenter);
    dTheta = omega*dtOdom;
    dx = abs(rotationCenter)*sin(dTheta);
    dx = isGoFoward ? dx : -dx;
    dy = abs(rotationCenter)*(1-cos(dTheta));
    dy = rotationCenter > 0 ? dy : -dy;
    dTheta = isPositiveTurn ? dTheta : -dTheta;
  }

  auto standardAxisDXDYVec = tf::Vector3(dx, dy, 0.0).rotate(zVec, -this->theta);

  this->velXY.x = standardAxisDXDYVec.getX()/dtOdom;
  this->velXY.y = standardAxisDXDYVec.getY()/dtOdom;

  if(isReceivedOdomData)
  {
    if(this->oldPosXYVec.size() >= MAX_OLD_ELEMENT_SZ)
    {
      this->oldPosXYVec.erase(this->oldPosXYVec.begin());
      this->oldYawVec.erase(this->oldYawVec.begin());
    }

    this->oldPosXYVec.emplace_back(this->posXY);
    this->oldYawVec.emplace_back(this->yaw);
  }

  this->yaw += dTheta;
  this->posXY.x += standardAxisDXDYVec.getX();
  this->posXY.y += standardAxisDXDYVec.getY();

  if(isReceivedOdomData)
  {
    this->LowPassFilt(this->posXY.x, this->oldPosXYVec.back().x);
    this->LowPassFilt(this->posXY.y, this->oldPosXYVec.back().y);
  }

  if(this->isReceivedGPSData)
    this->posXYGPS = this->posXY.Cvt2GPSData();
}

void LocalizationNode::SendMsg(void)
{
  geometry_msgs::Pose2D posXYMsg;
  posXYMsg.x = this->posXY.x;
  posXYMsg.y = this->posXY.y;
  posXYMsg.theta = this->theta;

  sensor_msgs::NavSatFix posXYGPSMsg;
  posXYGPSMsg.latitude = this->posXYGPS.lat;
  posXYGPSMsg.longitude = this->posXYGPS.lon;

  if(ros::ok())
  {
    this->posXYPub.publish(posXYMsg);
    if(this->isReceivedGPSData)
      this->posXYGPSPub.publish(posXYGPSMsg);
  }
}

double LocalizationNode::CalcTheta(const int idx)
{
  float angleByLoc = atan2(this->posXY.y-this->oldPosXYVec[idx].y, this->posXY.x-this->oldPosXYVec[idx].x);
  float angleByYaw = this->yaw-this->oldYawVec[idx];
  return angleByLoc+angleByYaw;
}

void LocalizationNode::EstimateTheta(void)
{
  if(this->oldPosXYVec.empty())
    return;

  this->oldTheta = this->theta;
  this->theta = 0;
  int oldPosXYVecSz = this->oldPosXYVec.size();
  for(int i = 0; i<oldPosXYVecSz; i++)
    this->theta += (i+1)*OLD_ELEMENT_IDX_WEIGHT*this->CalcTheta(i);

  this->LowPassFilt(this->theta, this->oldTheta);
}

void LocalizationNode::CallbackOdom(const kanu_msgs::WheelMsg::ConstPtr& wheelMsg)
{
  this->EstimateXY(wheelMsg);
  this->isReceivedOdomData = true;
  this->EstimateTheta();
  this->SendMsg();
}

void LocalizationNode::CallbackGPS(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
{
  if(!this->isReceivedOdomData)
    return;

  auto nowTime = ros::Time::now();
  double dtGPS = (nowTime-this->recentGPSTime).toSec();
  this->recentGPSTime = nowTime;
  XY gpsXY = GPSData(gpsMsg->latitude, gpsMsg->longitude).Cvt2XY();

  this->kalmanA << 1, 0, (this->posXY.x-this->kalmanx(kalmanxPosXIdx, 0))/this->kalmanx(kalmanxVelXIdx, 0), 0,
                   0, 1, 0, (this->posXY.y-this->kalmanx(kalmanxPosYIdx, 0))/this->kalmanx(kalmanxVelYIdx, 0),
                   0, 0, this->velXY.x/this->kalmanx(kalmanxVelXIdx, 0), 0,
                   0, 0, 0, this->velXY.y/this->kalmanx(kalmanxVelYIdx, 0);

  this->kalmanQ << dtGPS*unitValOdomPosX, 0, 0, 0,
                   0, dtGPS*unitValOdomPosY, 0, 0,
                   0, 0, valOdomVelX, 0,
                   0, 0, 0, valOdomVelY;

  auto kalmanz = Eigen::MatrixXd(kalmanM, 1);
  kalmanz << gpsXY.x,
             gpsXY.y;

  this->UpadtePosXYByKalmanFilter(kalmanz);
  this->isReceivedGPSData = true;
}

void LocalizationNode::UpadtePosXYByKalmanFilter(const Eigen::MatrixXd& kalmanz)
{
  auto kalmanPredx = Eigen::MatrixXd(kalmanN, 1);
  kalmanPredx = this->kalmanA*this->kalmanx;
  auto kalmanPredP = Eigen::MatrixXd(kalmanN, kalmanN);
  kalmanPredP = this->kalmanA*this->kalmanP*this->kalmanA.transpose()+this->kalmanQ;

  auto kalmanHTrans = Eigen::MatrixXd(kalmanN, kalmanM);
  kalmanHTrans = this->kalmanH.transpose();
  auto kalmanK = Eigen::MatrixXd(kalmanN, kalmanM);
  kalmanK = kalmanPredP*kalmanHTrans*((this->kalmanH*kalmanPredP*kalmanHTrans+this->kalmanR).inverse());

  this->kalmanx = kalmanPredx+kalmanK*(kalmanz-this->kalmanH*kalmanPredx);

  this->kalmanP = kalmanPredP-kalmanK*this->kalmanH*kalmanPredP;

  this->posXY.x = this->kalmanx(kalmanxPosXIdx, 0);
  this->posXY.y = this->kalmanx(kalmanxPosYIdx, 0);
}



LocalizationNode::LocalizationNode(int argc, char **argv)
{
  ros::init(argc, argv, THIS_NODE_NAME);
  ros::NodeHandle nh;

  this->recentGPSTime = ros::Time::now();
  ros::Subscriber odomSub = nh.subscribe(ODOM_NODE_NAME, ODOM_MSG_BUFFER_SZ, &LocalizationNode::CallbackOdom, this);
  ros::Subscriber gpsSub = nh.subscribe(GPS_NODE_NAME, GPS_MSG_BUFFER_SZ, &LocalizationNode::CallbackGPS, this);

  this->posXYPub = nh.advertise<geometry_msgs::Pose2D>(POS_XY_PUB_NODE_NAME, POS_XY_MSG_BUFFER_SZ);
  this->posXYGPSPub = nh.advertise<geometry_msgs::Pose2D>(POS_XY_GPS_PUB_NODE_NAME, POS_XY_GPS_MSG_BUFFER_SZ);

  this->kalmanA = Eigen::MatrixXd(kalmanN, kalmanN);
  this->kalmanQ = Eigen::MatrixXd(kalmanN, kalmanN);

  this->kalmanH = Eigen::MatrixXd(kalmanM, kalmanN);
  this->kalmanH << 1, 0, 0, 0,
                   0, 1, 0, 0;

  this->kalmanR = Eigen::MatrixXd(kalmanM, kalmanM);
  this->kalmanR << gpsVarX, 0,
                   0, gpsVarY;

  this->kalmanx = Eigen::MatrixXd(kalmanN, 1);
  this->kalmanx << initPosX,
                   initPosY,
                   initVelX,
                   initVelY;

  this->kalmanP = Eigen::MatrixXd(kalmanN, kalmanN);
  this->kalmanP << initValPosX, 0, 0, 0,
                   0, initValPosY, 0, 0;
                   0, 0, initValVelX, 0;
                   0, 0, 0, initValVelY;

  this->oldPosXYVec.reserve(MAX_OLD_ELEMENT_SZ);
  this->oldYawVec.reserve(MAX_OLD_ELEMENT_SZ);
  this->theta = 0;
  this->oldTheta = 0;
  this->yaw = 0;
  this->isReceivedOdomData = false;
  this->isReceivedGPSData = false;
  this->isReceivedYaw = false;

  ros::spin();
}

int main(int argc, char** argv)
{
  XY xy = GPSData(37.29716514948569, 126.83552243953464).Cvt2XY();
  GPSData gpsData = XY(20, 10).Cvt2GPSData();
  printf("%f %f %f %f\n", xy.x, xy.y, gpsData.lat, gpsData.lon);
  LocalizationNode LocalizationNode(argc, argv);
}