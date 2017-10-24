#define USING_SERIAL_PORT_
#define SAM_ACTUATOR_
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
using namespace std;
#include "sensor_hub/sensormodule.h"
#include "sensor_hub/sammodule.h"
#include "sensor_hub/mainsensorhub.h"
#include "sensor_hub/dataSensorMsg.h"
#include "sensor_hub/cmdSensorMsg.h"
#include "sensor_hub/tfSensorMsg.h"
#include "sensor_hub/SAMtfCalMsg.h"
#include "sensor_hub/SAMJointPos12UpperMsg.h"

SensorModule  *mySensor;
SAMmodule  *mySAM;

unsigned int SAMpos12[25];
unsigned char SAMmode[25];

struct Sys_flag_struct{
    unsigned char getFilteredData_50Hz:1;
    unsigned char getRawData_50Hz:1;

    unsigned char getFilteredData_125Hz:1;
    unsigned char getRawData_125Hz:1;

    unsigned char communicationBusy:1;
    unsigned char readyToSend:1;

    unsigned char readyToSetTorque:1;
    unsigned char readyToSetPID:1;
    unsigned char readyToSendSAM22;
}sys_flag;



#define HAND_MOTION_1 'm'//109
#define HAND_MOTION_2 'n'//110
#define HAND_MOTION_3 'f'//114
#define HAND_MOTION_4 'h'//104
#define HAND_MOTION_5 'l'//108


#define ZMP_POSITION_0_X 0
#define ZMP_POSITION_0_Y 0

#define ZMP_POSITION_1_X 71
#define ZMP_POSITION_1_Y 0

#define ZMP_POSITION_2_X 80.5
#define ZMP_POSITION_2_Y 200

#define ZMP_POSITION_3_X 7
#define ZMP_POSITION_3_Y 200
#define ZMP_OFSET_ORIGIN_X_RIGHT 40
#define ZMP_OFSET_ORIGIN_Y_RIGHT 70

#define ZMP_POSITION_4_X 0
#define ZMP_POSITION_4_Y 0

#define ZMP_POSITION_5_X 71
#define ZMP_POSITION_5_Y 0

#define ZMP_POSITION_6_X 64
#define ZMP_POSITION_6_Y 200

#define ZMP_POSITION_7_X -9.5
#define ZMP_POSITION_7_Y 200

#define ZMP_OFSET_ORIGIN_X_LEFT 30
#define ZMP_OFSET_ORIGIN_Y_LEFT 70


double testPoint[3]={0,0,0};

void sub_samTf_function(const sensor_hub::SAMtfCalMsg::ConstPtr& msg){
    testPoint[0]=msg->CN1;
    testPoint[1]=msg->CN2;
    testPoint[2]=msg->CN3;
}
void sub_function(const sensor_hub::cmdSensorMsg::ConstPtr& msg){
    switch (msg->command){
    case HAND_MOTION_1:
        ROS_INFO("HAND_MOTION_1 : %d", msg->command);
//        sys_flag.getFilteredData_50Hz=1;
//        sys_flag.getFilteredData_125Hz=0;
//        sys_flag.getRawData_50Hz=0;
//        sys_flag.getRawData_125Hz=0;
        mySAM->handMotion(HAND_MOTION_1);
        break;
    case HAND_MOTION_2:
        ROS_INFO("HAND_MOTION_2: %d", msg->command);
//        sys_flag.getFilteredData_50Hz=0;
//        sys_flag.getFilteredData_125Hz=1;
//        sys_flag.getRawData_50Hz=0;
//        sys_flag.getRawData_125Hz=0;
          mySAM->handMotion(HAND_MOTION_2);
        break;

    case HAND_MOTION_3:
        ROS_INFO("HAND_MOTION_3 : %d", msg->command);
//        sys_flag.getFilteredData_50Hz=0;
//        sys_flag.getFilteredData_125Hz=0;
//        sys_flag.getRawData_50Hz=1;
//        sys_flag.getRawData_125Hz=0;
          mySAM->handMotion(HAND_MOTION_3);
        break;
    case HAND_MOTION_4:
        ROS_INFO("HAND_MOTION_4: %d", msg->command);
//        sys_flag.getFilteredData_50Hz=0;
//        sys_flag.getFilteredData_125Hz=0;
//        sys_flag.getRawData_50Hz=0;
//        sys_flag.getRawData_125Hz=1;
          mySAM->handMotion(HAND_MOTION_4);
        break;
    default:
        ROS_INFO("receive msg, turn off sensor : %d", msg->command);
        sys_flag.getFilteredData_50Hz=0;
        sys_flag.getFilteredData_125Hz=0;
        sys_flag.getRawData_50Hz=0;
        sys_flag.getRawData_125Hz=0;
        break;
    }
}

void sub_function_pos12(const sensor_hub::SAMJointPos12UpperMsg::ConstPtr& msg){

    ROS_INFO("data come");
    for (unsigned char i=0; i<25;i++)
    {
        if(msg->SAMMode[i]==1)
        {
            SAMpos12[i]=msg->SAMPos12[i];
            SAMmode[i]=1;
        }else if(msg->SAMMode[i]==2){// this mode is usefulless if reading position after this command
            SAMpos12[i]=msg->SAMPos12[i];
            SAMmode[i]=2;
        }
    }
    sys_flag.readyToSend=1;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_hub");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

    //#ifdef USING_SERIAl_PORT_
    //    mySensor =new SensorModule;
    mySAM =new SAMmodule;

    ros::Publisher sensor_pub =  n.advertise<sensor_hub::dataSensorMsg>("sensor_pub",1000);
    ros::Subscriber sensor_sub=n.subscribe<sensor_hub::cmdSensorMsg>("sensor_sub",200,sub_function);

    ros::Subscriber samPos12_sub=n.subscribe<sensor_hub::SAMJointPos12UpperMsg>("sam_pos12_upper_sub",1000,sub_function_pos12);

    sensor_hub::dataSensorMsg sensorMsg;
    //    ros::Publisher tf_sensor_pub =  n.advertise<sensor_hub::tfSensorMsg>("tf_sensor_pub",1000);


    //    ros::Subscriber sam_tf_sub=n.subscribe<sensor_hub::SAMtfCalMsg>("sam_tfCal_pub",200,sub_samTf_function);
    //    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //    sensor_hub::tfSensorMsg tfSensorMsg;

    //    tf::TransformListener listener(ros::Duration(5));
    //    tf::StampedTransform transform_left;
    //    tf::StampedTransform transform_right;
    //    geometry_msgs::PointStamped foot_left_point,foot_right_point;
    //    geometry_msgs::PointStamped base_left_point,base_right_point;

    //    foot_left_point.point.x=0;
    //    foot_left_point.point.y=-0.083;
    //    foot_left_point.point.z=0;
    //    foot_right_point.point.x=0;
    //    foot_right_point.point.y=0.083;
    //    foot_right_point.point.z=0;
    //    foot_left_point.header.frame_id="L1";
    //    foot_right_point.header.frame_id="L0";
    //    //========= test point==============
    //    geometry_msgs::PointStamped base_test_point,test_point;
    //    test_point.header.frame_id="base_link";

    sys_flag.getFilteredData_50Hz=0;
    sys_flag.getFilteredData_125Hz=0;
    sys_flag.getRawData_50Hz=0;
    sys_flag.getRawData_125Hz=0;

#ifdef USING_SERIAL_PORT_
    //    if(mySensor->Serial!= -1)
    //    {
    if(mySAM->Serial!= -1)
    {
#endif
        while(ros::ok())
        {
            if(FlagTimer.Hz_25){
                FlagTimer.Hz_25=0;
                //=================
                //                try{
                //                    foot_left_point.header.stamp=foot_right_point.header.stamp=test_point.header.stamp=ros::Time();
                //                    //                    listener.transformPoint("base_link", foot_left_point, base_left_point);
                //                    listener.transformPoint("base_link", foot_right_point, base_right_point);

                //                    tfSensorMsg.CN1= base_left_point.point.x;
                //                    //                    tfSensorMsg.CN2=base_right_point.point.x;
                //                    tf_sensor_pub.publish(tfSensorMsg);
                //                    test_point.point.x=testPoint[0];
                //                    test_point.point.y=testPoint[1];
                //                    test_point.point.z=testPoint[2];
                //                    //                    listener.transformPoint("base_link", test_point, base_test_point);



                //                    ROS_INFO("test_point: (%.5f, %.5f. %.5f)|left_point: (%.5f, %.5f, %.5f)| at time %.2f",
                //                             test_point.point.x, test_point.point.y, test_point.point.z,
                //                             base_right_point.point.x, base_right_point.point.y, base_right_point.point.z,base_right_point.header.stamp.toSec());


                //                    //                    ROS_INFO("test_point: (%.5f, %.5f. %.5f)|left_point: (%.5f, %.5f, %.5f)| at time %.2f",
                //                    //                             base_test_point.point.x, base_test_point.point.y, base_test_point.point.z,
                //                    //                             base_right_point.point.x, base_right_point.point.y, base_right_point.point.z,base_right_point.header.stamp.toSec());


                //                    //                    ROS_INFO("right: (%.2f, %.2f. %.2f)|left: (%.2f, %.2f, %.2f)|delta =%f at time %.2f",
                //                    //                             base_right_point.point.x, base_right_point.point.y, base_right_point.point.z,
                //                    //                             base_left_point.point.x, base_left_point.point.y, base_left_point.point.z, base_right_point.point.x-base_left_point.point.x,base_left_point.header.stamp.toSec());
                //                    //========== my function==============
                //                    visualization_msgs::Marker points;
                //                    points.header.frame_id ="/base_link";
                //                    points.header.stamp = ros::Time::now();
                //                    points.ns  = "points_and_lines";
                //                    points.action  = visualization_msgs::Marker::ADD;
                //                    points.pose.orientation.w = 1.0;
                //                    points.id = 0;
                //                    points.type = visualization_msgs::Marker::POINTS;
                //                    points.scale.x = .005;//0.2;
                //                    points.scale.y = .005;//0.2;

                //                    // Points are green
                //                    points.color.g = 1.0f;
                //                    points.color.a = 1.0;

                //                    geometry_msgs::Point p;

                //                    p.x = base_right_point.point.x;
                //                    p.y = base_right_point.point.y;
                //                    p.z = base_right_point.point.z;
                //                    points.points.push_back(p);



                //                    p.x = test_point.point.x;
                //                    p.y = test_point.point.y;
                //                    p.z =test_point.point.z;
                //                    points.points.push_back(p);

                //                    marker_pub.publish(points);

                //                }    catch(tf::TransformException& ex){
                //                    //                    ROS_ERROR("Received an exception trying to transform a poin: %s", ex.what());
                //                }
            }
            if(FlagTimer.Hz_50)
            {
                FlagTimer.Hz_50=0;
                //================

                //                if(sys_flag.getFilteredData_50Hz)
                //                {
                //                    mySensor->getFilteredData();
                //                }
                //                else if(sys_flag.getRawData_50Hz)
                //                {
                //                    mySensor->getRawData();
                //                }

            }
            if(FlagTimer.Hz_100)
            {
                FlagTimer.Hz_100=0;
            }
            if(FlagTimer.Hz_125)
            {
                FlagTimer.Hz_125=0;
                //===============
                //                if(sys_flag.getFilteredData_125Hz)
                //                {
                //                    mySensor->getFilteredData();
                //                }
                //                else if(sys_flag.getRawData_125Hz)
                //                {
                //                    mySensor->getRawData();
                //                }
            }


            /*==========================================
                        mySensor->Recev_Data_hanlder();
                        if(mySensor->flagDataReceived_readProcessed){
                            mySensor->flagDataReceived_readProcessed=0;


                //===========================================
                unsigned char Trans_chr[1] ={ '1'};
                mySensor->Send_Serial_String(mySensor->Serial, Trans_chr, 1);
                sensorMsg.zmp_P0=mySensor->sensorData[0];
                sensorMsg.zmp_P1=mySensor->sensorData[1];
                sensorMsg.zmp_P2=mySensor->sensorData[2];
                sensorMsg.zmp_P3=mySensor->sensorData[3];
                sensorMsg.zmp_P4=mySensor->sensorData[4];
                sensorMsg.zmp_P5=mySensor->sensorData[5];
                sensorMsg.zmp_P6=mySensor->sensorData[6];
                sensorMsg.zmp_P7=mySensor->sensorData[7];
                sensorMsg.body_roll=mySensor->sensorData[8]/100.0;
                sensorMsg.body_pitch=mySensor->sensorData[9]/100.0;
                sensorMsg.body_yaw=mySensor->sensorData[10]/100.0;
                sensorMsg.body_roll_rate=mySensor->sensorData[11]/100.0;
                sensorMsg.body_pitch_rate=mySensor->sensorData[12]/100.0;
                sensorMsg.body_yaw_rate=mySensor->sensorData[13]/100.0;

                sensorMsg.right_amplitude=sensorMsg.zmp_P0+sensorMsg.zmp_P1+sensorMsg.zmp_P2+sensorMsg.zmp_P3;
                if(sensorMsg.right_amplitude==0){
                    sensorMsg.right_x=0;
                    sensorMsg.right_y=0;

                }
                else{
                    sensorMsg.right_x=(sensorMsg.zmp_P0*ZMP_POSITION_0_X+sensorMsg.zmp_P1*ZMP_POSITION_1_X+sensorMsg.zmp_P2*ZMP_POSITION_2_X+sensorMsg.zmp_P3*ZMP_POSITION_3_X)/sensorMsg.right_amplitude-ZMP_OFSET_ORIGIN_X_RIGHT;
                    sensorMsg.right_y=(sensorMsg.zmp_P0*ZMP_POSITION_0_Y+sensorMsg.zmp_P1*ZMP_POSITION_1_Y+sensorMsg.zmp_P2*ZMP_POSITION_2_Y+sensorMsg.zmp_P3*ZMP_POSITION_3_Y)/sensorMsg.right_amplitude-ZMP_OFSET_ORIGIN_Y_RIGHT;
                }
                sensorMsg.left_amplitude=sensorMsg.zmp_P4+sensorMsg.zmp_P5+sensorMsg.zmp_P6+sensorMsg.zmp_P7;
                if(sensorMsg.left_amplitude==0){
                    sensorMsg.left_x=0;
                    sensorMsg.left_y=0;
                }else{
                    sensorMsg.left_x=(sensorMsg.zmp_P4*ZMP_POSITION_4_X+sensorMsg.zmp_P5*ZMP_POSITION_5_X+sensorMsg.zmp_P6*ZMP_POSITION_6_X+sensorMsg.zmp_P7*ZMP_POSITION_7_X)/sensorMsg.left_amplitude-ZMP_OFSET_ORIGIN_X_LEFT;
                    sensorMsg.left_y=(sensorMsg.zmp_P4*ZMP_POSITION_4_Y+sensorMsg.zmp_P5*ZMP_POSITION_5_Y+sensorMsg.zmp_P6*ZMP_POSITION_6_Y+sensorMsg.zmp_P7*ZMP_POSITION_7_Y)/sensorMsg.left_amplitude-ZMP_OFSET_ORIGIN_Y_LEFT;
                }
                sensor_pub.publish(sensorMsg);
            }
            */

            if(sys_flag.readyToSend){
                if(sys_flag.communicationBusy==0){
                    sys_flag.readyToSend=0;
                    ROS_INFO("ready to send 1");

#ifdef SAM_ACTUATOR_
                    //                    mySAM->setAllPos12(SAMpos12,SAMmode,12);
                    unsigned char numOfSam=0;
                    for(unsigned char i=0;i<25;i++){
                        if(SAMmode[i])
                            numOfSam++;
                    }
                    mySAM->setAllPos12_upper(SAMpos12,SAMmode,numOfSam);
#endif

                }
            }

            ros::spinOnce();
            loop_rate.sleep();
            Timer_handler();
        }
#ifdef USING_SERIAL_PORT_
    }else{
        cout << "SERIAL : " << mySensor->Serial << " Connection error." << endl;
    }
    cout << endl;
    cout << "SERIAL : " << mySensor->Serial << " Device close." << endl;
    close(mySensor->Serial);
    cout << "SERIAL : " << "uxa_serial node terminate." << endl;
#endif


    return 0;
}
