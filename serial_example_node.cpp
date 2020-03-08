/***
 20헤르츠 버그 수정필요
 왼쪽 엔코더 편향상태 보정공식 적용필요

 보내는 배열 14Byte, 받는 배열 18Byte
 1. 원하는 좌표 통과하기
 
 2. 원하는 좌표에서 멈추기
 
 3. 만해광장 한바퀴
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <macaron/erp42_read.h>
#include <macaron/erp42_write.h>

#define PI acos(-1) //PI값 상수 설정
#define MAX 18 //배열 크기 고정해줄 상수(erp42에서 받아올 때)

serial::Serial ser;//"serial" namespace의 "Serial"의 class(ser) 선언, 59열 참고
 
//imu value
double yaw_imu = 0, yaw0_imu = 0; //??????
int firstrun = 1; //????????

//write value
int steer=0; //모니터링 하기 위한 값
bool E_stop; //주고받는 값
uint8_t steer1,steer2,speed,brake,gear; //주고받는 값


double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property

double ENC_saver[4]; //엔코더값 받아오기

double vel_saver[4]; //??????

double dt=0; //미분 위한 변수 선언

 

void write_callback(const macaron::erp42_write::ConstPtr& write){ //콜백함수, 받아옴. 보내는 값

    E_stop=write->write_E_stop; //write 내의 write_E_stop 저장
    gear=write->write_gear;//전진,중립,후진
    steer= -write->write_steer;
//저희차가 z축이 아래다. 그래서 -를 준거다
//steer/256, %256이 진수때매 그런거다.
    steer1=(steer/256); //앞 8개 비트
    steer2=(steer%256); //뒤 8개 비트
    if(steer<0) steer1=steer1-1; //보수처리??????
    speed= write->write_speed;
    brake= write->write_brake;
}

 

 

int main (int argc, char** argv){

    ros::init(argc, argv, "serial_example_node"); //노드명

    ros::NodeHandle nh;

 

    ros::Time current_time, last_time; //ros namespace의 Time class 2개 선언

    current_time = ros::Time::now(); //dt 만들기 위한 과정

    last_time = ros::Time::now(); //dt 만들기 위한 과정

    ros::Subscriber write_sub = nh.subscribe("erp_write", 1, write_callback); //섭스크라이버 선언

    ros::Publisher read_pub = nh.advertise<macaron::erp42_read>("erp_read", 1); //퍼블리셔 선언

    macaron::erp42_read erp42_state;//erp42_state라는 class 선언(퍼블리시할 메시지 저장)

    

    try

    {

        ser.setPort("/dev/erp42"); //포트명 erp42로 설정(기본 ttyUSB0)

        ser.setBaudrate(115200); //시리얼 통신 위해 통신 Baudrate 맞춤

        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //???

        ser.setTimeout(to);//???

        ser.open();//???

        }

 

    catch (serial::IOException& e)//포트 열지 못하면 에러 메시지, 함수 끝냄

    {

        ROS_ERROR_STREAM("Unable to open port ");

        return -1;

    }

 

    if(ser.isOpen()){

        ROS_INFO_STREAM("Serial Port initialized");

 

    }else{

        return -1;

    }

 

    ros::Rate loop_rate(50); //주기 20msec

        uint8_t answer[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //받는 값

        uint8_t answer_quere[MAX]={0,}; //?????

        uint8_t answer_tester[1]={0x00}; //첫번째부터 잘못된 값 나오면 바로 닫아버릴 수 있도록 따로 변수 설정

 

     if(ser.available()){

           // ROS_INFO_STREAM("Reading from serial port");

 

            while(answer_tester[0]! =0x53){ //0x53이 될 때까지 계속 굴림(맨처음에는 0x00이었음)

                ser.read(answer_tester,1); //answer_tester에서 값 1개 읽어옴

//                printf("%x ",answer_tester[0]);

            }

            answer_tester[0]={0x00}; //다시 0x00 넣어줌

            ser.read(answer,17); //answer에서 값 17개 읽어옴

 

            if(answer[0]==0x54 && answer[1]==0x58)

            {

                if(answer[15]==0x0D && answer[16]==0x0A) //처음, 끝 조건 만족하면

            { //  for(int i=0;i<17;i++)         printf("%x ",answer[i]);

              //  printf("\n");

                    erp42_state.read_AorM=bool(answer[2]); //erp42_state 클래스의 raad_AorM에 값 저장

                    erp42_state.read_E_stop=bool(answer[3]);

                    erp42_state.read_gear=answer[4];

                    erp42_state.read_speed=int(answer[5]);

                    erp42_state.read_brake=int(answer[9]);

                    erp42_state.read_ENC=int(answer[13])*256*256*256+int(answer[12])*256*256+int(answer[11])*256+int(answer[10]);

                    //12,13,14,15번째 인자에 각각 (2^8)^0, (2^8)^1, (2^8)^2, (2^8)^3 곱한 후 모두 더함 ==> 엔코더 값 알아냄

                    ENC_saver[0]=erp42_state.read_ENC; //????

                    ENC_saver[1]=erp42_state.read_ENC;

                    ENC_saver[2]=erp42_state.read_ENC;

                    ENC_saver[3]=erp42_state.read_ENC;

 

            }

        }

     }

 

 

    while(ros::ok()){

    ros::spinOnce();

    last_time = current_time; //이전 시간 현재 시간으로 초기화(t0)

    current_time = ros::Time::now(); //현재 시간 초기화(t)

    dt = (current_time - last_time).toSec(); //tosec 로스에서 지원

 

    uint8_t a; //allive값 카운팅(매번 값 받아올 때마다 1씩 증가,255가 넘으면 다시 0부터 시작)

    a++;

 

    uint8_t ask[14]={0x53,0x54 ,0x58,0x01,0x00,gear,0x00,speed,steer1,steer2,brake,a,0x0D,0x0A}; //보내는 값

    ser.write(ask,14);//S  0x53

 

        if(ser.available()){

            ser.read(answer_quere,18);

            

            for(int i=0; i<MAX;i++){

                printf("%x ",answer_quere[i]);

            }

            printf("%f \n",dt);

        }

 

    if(answer_quere[0]!=0x53 || answer_quere[1]!=0x54 || answer_quere[2]!=0x58 || answer_quere[16]!=0x0D || answer_quere[17]!=0x0A)

    {

        ser.flushOutput();

        for(int i=0; i<MAX;i++){

            answer_quere[i]={0x00};

        }

        while(answer_tester[0]!=0x0A){ //0x0A가 아니면

            ser.read(answer_tester,1); //1개 읽어옴, 0x0D일 때 0x0A까지 읽음(이 다음엔 0x53 읽을 차례가 됨)

            ROS_INFO("DORMAMU %x",answer_tester[0]); //지금 어디까지 읽어왔나를 표시해줌

        }

    }

 

    else{

        erp42_state.read_AorM=bool(answer_quere[3]);

        erp42_state.read_E_stop=bool(answer_quere[4]);

        erp42_state.read_gear=answer_quere[5];

        erp42_state.read_speed=int(answer_quere[6]);

        erp42_state.read_brake=int(answer_quere[10]);

        erp42_state.read_ENC=int(answer_quere[14])*256*256*256+int(answer_quere[13])*256*256+int(answer_quere[12])*256+int(answer_quere[11]);

    

        for(int i=2; i>=0; i--)

        {

            ENC_saver[i+1]=ENC_saver[i];

        }

        ENC_saver[0]=erp42_state.read_ENC;

        erp42_state.read_velocity=0.01651*((ENC_saver[0]-ENC_saver[2]))/2/dt; //dt는 미분

//0.01651이 지름이라 2로 나눠서 반지름표현

 

    

        for(int i=2; i>=0; i--)

        {

        vel_saver[i+1]=vel_saver[i];

        }

        vel_saver[0]=erp42_state.read_velocity;

 

        erp42_state.read_accel=((vel_saver[0]-vel_saver[1]))/dt;

        erp42_state.read_steer=int(answer_quere[9])*256+int(answer_quere[8]);

 

        if(erp42_state.read_steer>32768) erp42_state.read_steer=erp42_state.read_steer-65536+1;

 

        erp42_state.read_yaw=erp42_state.read_yaw+0.01652*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;

        erp42_state.read_s=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*cos( erp42_state.read_yaw/180/PI); //사실 y임

        erp42_state.read_l=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*sin( erp42_state.read_yaw/180/PI); //사실 x임

        read_pub.publish(erp42_state);

        ser.flush();

        loop_rate.sleep();

 

    }

                  //엔코더로 yaw를 구하는 식. 보정필요, IMU로 대체.

                    //erp42_state.read_yaw=erp42_state.read_yaw+0.01651*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;

 }

}

        

 
