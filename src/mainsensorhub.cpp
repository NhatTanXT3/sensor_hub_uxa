#define USING_SERIAL_PORT_
#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "sensor_hub/sensormodule.h"
#include "sensor_hub/mainsensorhub.h"
#include "sensor_hub/dataSensorMsg.h"
#include "sensor_hub/cmdSensorMsg.h"
SensorModule  *mySensor;

struct Sys_flag_struct{
    unsigned char getFilteredData_50Hz:1;
    unsigned char getRawData_50Hz:1;

    unsigned char getFilteredData_125Hz:1;
    unsigned char getRawData_125Hz:1;
}sys_flag;


#define SENSOR_TURNOFF 0
#define SENSOR_SEND_FILTERED_50HZ 1
#define SENSOR_SEND_FILTERED_125HZ 2
#define SENSOR_SEND_RAW_50HZ 3
#define SENSOR_SEND_RAW_125HZ 4
void sub_function(const sensor_hub::cmdSensorMsg::ConstPtr& msg){
    switch (msg->command){
    case SENSOR_SEND_FILTERED_50HZ:
        ROS_INFO("SENSOR_SEND_FILTERED_50HZ : %d", msg->command);
        sys_flag.getFilteredData_50Hz=1;
        sys_flag.getFilteredData_125Hz=0;
        sys_flag.getRawData_50Hz=0;
        sys_flag.getRawData_125Hz=0;
        break;
    case SENSOR_SEND_FILTERED_125HZ:
        ROS_INFO("SENSOR_SEND_FILTERED_125HZ: %d", msg->command);
        sys_flag.getFilteredData_50Hz=0;
        sys_flag.getFilteredData_125Hz=1;
        sys_flag.getRawData_50Hz=0;
        sys_flag.getRawData_125Hz=0;
        break;

    case SENSOR_SEND_RAW_50HZ:
        ROS_INFO("SENSOR_SEND_RAW_50HZ : %d", msg->command);
        sys_flag.getFilteredData_50Hz=0;
        sys_flag.getFilteredData_125Hz=0;
        sys_flag.getRawData_50Hz=1;
        sys_flag.getRawData_125Hz=0;
        break;
    case SENSOR_SEND_RAW_125HZ:
        ROS_INFO("SENSOR_SEND_RAW_125HZ: %d", msg->command);
        sys_flag.getFilteredData_50Hz=0;
        sys_flag.getFilteredData_125Hz=0;
        sys_flag.getRawData_50Hz=0;
        sys_flag.getRawData_125Hz=1;
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
int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_hub");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

//#ifdef USING_SERIAl_PORT_
    mySensor =new SensorModule;

    ros::Publisher sensor_pub =  n.advertise<sensor_hub::dataSensorMsg>("sensor_pub",1000);
    ros::Subscriber sensor_sub=n.subscribe<sensor_hub::cmdSensorMsg>("sensor_sub",200,sub_function);
    sensor_hub::dataSensorMsg sensorMsg;

    sys_flag.getFilteredData_50Hz=0;
    sys_flag.getFilteredData_125Hz=0;
    sys_flag.getRawData_50Hz=0;
    sys_flag.getRawData_125Hz=0;

#ifdef USING_SERIAL_PORT_
    if(mySensor->Serial!= -1)
    {
#endif
        while(ros::ok())
        {
            if(FlagTimer.Hz_50)
            {
                FlagTimer.Hz_50=0;
                //================
                //                unsigned char Trans_chr[1] ={ '0'};
                //                mySensor->Send_Serial_String(mySensor->Serial, Trans_chr, 1);
                //                 mySensor->getFilteredData();
                if(sys_flag.getFilteredData_50Hz)
                {
                    mySensor->getFilteredData();
                }
                else if(sys_flag.getRawData_50Hz)
                {
                    mySensor->getRawData();
                }
                //                ROS_INFO("%s", "get data");
            }
            if(FlagTimer.Hz_100)
            {
                FlagTimer.Hz_100=0;
            }
            if(FlagTimer.Hz_125)
            {
                FlagTimer.Hz_125=0;
                //===============
                if(sys_flag.getFilteredData_125Hz)
                {
                    mySensor->getFilteredData();
                }
                else if(sys_flag.getRawData_125Hz)
                {
                    mySensor->getRawData();
                }
            }


            //==========================================
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
                sensor_pub.publish(sensorMsg);
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
