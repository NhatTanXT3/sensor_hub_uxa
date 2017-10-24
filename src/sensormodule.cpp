#include <iostream>
#include <ros/ros.h>
#include <fcntl.h>
#include <termio.h>
#include <sys/stat.h>

using namespace std;

#include "sensor_hub/sensormodule.h"

SensorModule::SensorModule()
{
    Serial = InitSerial(_SERIAL_PORT_SENSOR);
}

void SensorModule::Recev_Data_hanlder()
{
    int dataSize=read(Serial, Recev_chr, 60);
    if(dataSize > 0)
    {
        for (unsigned char i=0;i<dataSize;i++)
        {
            if(Recev_chr[i]==PC2MCU_HEADER_){
                dataIndex=0;
                flagEnableCapute=1;
                Store_chr[dataIndex++]=Recev_chr[i];
            }else if((Recev_chr[i]==PC2MCU_TERMINATOR_)&&(flagEnableCapute)){
                flagDataReceived=1;
                flagEnableCapute=0;
                Store_chr[dataIndex++]=Recev_chr[i];

            }else if((flagEnableCapute)&&(dataIndex<_SERIAL_BUFF_SIZE))
            {
                Store_chr[dataIndex++]=Recev_chr[i];
            }else if(dataIndex>=_SERIAL_BUFF_SIZE){
                dataIndex=0;
            }
        }

        if(flagDataReceived)
        {
            flagDataReceived=0;
            memset(Recev_chr, '\0', sizeof(Recev_chr));
            /*
             * begin coding
             */
            if(Store_chr[1]==PC_SENSOR_READ_RAW){
                flagDataReceived_readRaw=1;
                memset(sensorDataAvail, '\0', sizeof(sensorDataAvail));
                //===============================
                NumofSensor=(dataIndex-3)/4;
                for (unsigned char i=0;i<NumofSensor;i++)
                {
                    if(Store_chr[i*4+5]==((Store_chr[i*4+2]^Store_chr[i*4+3]^Store_chr[i*4+4])&0x7F))
                    {
                        sensorData[Store_chr[i*4+2]&0x1F]=(Store_chr[i*4+4]&0x7F)|((Store_chr[i*4+3]&0x7F)<<7)|((Store_chr[i*4+2]&0x60)<<9);
                        sensorDataAvail[Store_chr[i*4+2]&0x1F]=1;
                    }
                    else
                        cout<<"error checksum 1"<<endl;
                }

                for (unsigned char i=0; i<20;i++)
                {
                    if(sensorDataAvail[i])
                    {
                        cout<<(int)i<<" : "<<sensorData[i]<<endl;
                    }
                }

            }
            else if(Store_chr[1]==PC_SENSOR_READ_PROCESSED){
                flagDataReceived_readProcessed=1;
                memset(sensorDataAvail, '\0', sizeof(sensorDataAvail));
                //===============================
                NumofSensor=(dataIndex-3)/4;
                for (unsigned char i=0;i<NumofSensor;i++)
                {
                    if(Store_chr[i*4+5]==((Store_chr[i*4+2]^Store_chr[i*4+3]^Store_chr[i*4+4])&0x7F))
                    {
                        sensorData[Store_chr[i*4+2]&0x1F]=(Store_chr[i*4+4]&0x7F)|((Store_chr[i*4+3]&0x7F)<<7)|((Store_chr[i*4+2]&0x60)<<9);
                        sensorDataAvail[Store_chr[i*4+2]&0x1F]=1;
                    }
                    else
                        cout<<"error checksum 2"<<endl;
                }

                for (unsigned char i=0; i<20;i++)
                {
                    if(sensorDataAvail[i])
                    {
                        if(i<8)
                            cout<<(int)i<<" : "<<sensorData[i]<<endl;
                        else
                            cout<<(int)i<<" : "<<sensorData[i]/100.0<<endl;
                    }
                }
            }
            else if(dataIndex==6){
            }
        }
    }
}

int SensorModule::getRawData()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xaa;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

int SensorModule::getFilteredData()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xdd;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}


void SensorModule::Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size)
{
    write(Serial, Trans_chr, Size);
}

int SensorModule::InitSerial(const char *Serial_Port)
{
    termios Serial_Setting;
    if((Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY)) == -1)
    {
        cout << "SERIAL : " << Serial_Port << " Device open error" << endl;
        cout << "SERIAL : " << Serial_Port << " Device permission change progress...." << endl;
        for(int temp = 0; temp < 5; temp++)
        {
            if(chmod(Serial_Port, __S_IREAD | __S_IWRITE) == 0){

                cout << "SERIAL : " << Serial_Port << " Device permission change complete" << endl;
                Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY);

                if(Serial == -1)
                {
                    cout << "SERIAL : " << Serial_Port << " Device Not Found" << endl;
                    return -1;
                }
                else
                    cout << "SERIAL : " << Serial_Port <<" Device open" << endl;


            }
            else
            {
                cout << "SERIAL : " << Serial_Port << " Device permission change error" << endl;
                //return -1;
            }
        }
    }
    else
        cout << "SERIAL : " << Serial_Port << " Device open" << endl;


    memset(&Serial_Setting, 0, sizeof(Serial_Setting));
    Serial_Setting.c_iflag = 0;
    Serial_Setting.c_oflag = 0;
    Serial_Setting.c_cflag = _BAUDRATE | CS8 | CREAD | CLOCAL;
    Serial_Setting.c_lflag = 0;
    Serial_Setting.c_cc[VMIN] = 1;
    Serial_Setting.c_cc[VTIME] = 0;

    cfsetispeed(&Serial_Setting, _BAUDRATE);
    cfsetospeed(&Serial_Setting, _BAUDRATE);

    tcflush(Serial, TCIFLUSH);
    tcsetattr(Serial, TCSANOW, &Serial_Setting);
    return Serial;
}
