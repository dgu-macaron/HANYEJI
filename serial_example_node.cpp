/*
 20헤르츠 버그 수정필요
 왼쪽 엔코더 편향상태 보정공식 적용필요

 보내는 배열 14Byte, 받는 배열 18Byte
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <macaron/erp42_read.h>
#include <macaron/erp42_write.h>

#define PI acos(-1) //PI값 상수 설정
#define MAX 18 //배열 크기 고정해줄 상수(erp42에서 받아올 때)

serial::Serial ser;//"serial" namespace의 "Serial"의 class(ser) 선언
 
//imu value
double yaw_imu = 0, yaw0_imu = 0; //적분할 때 필요한 이전값이랑 현재값 설정해주는듯
int firstrun = 1; //????????

//write value
int steer=0; //모니터링 하기 위한 값
bool E_stop; //주고받는 값
uint8_t steer1,steer2,speed,brake,gear; //주고받는 값


double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property

double ENC_saver[4]; //최근 4개 엔코더값 저장

double vel_saver[4]; //최근 4개 속도값 저장

double dt=0; //미분 위한 변수 선언

 

void write_callback(const macaron::erp42_write::ConstPtr& write){ //받아온 erp_write 토픽 메시지값 각 변수들에 변환/저장

    E_stop=write->write_E_stop; //write 내의 write_E_stop 저장
    gear=write->write_gear;//전진,중립,후진
    steer= -write->write_steer;
//저희차가 z축이 아래다. 그래서 -를 준거다
//steer/256, %256이 진수때매 그런거다.
    steer1=(steer/256); //앞 8개 비트
    steer2=(steer%256); //뒤 8개 비트
    if(steer<0) steer1=steer1-1; //보수처리
    //만일 steer = -1506이라고 하자. 이 때 steer1 = -5, steer2 = -226이다.
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

        ser.setPort("/dev/erp42"); //포트 erp42로 설정(기본 ttyUSB0)

        ser.setBaudrate(115200); //시리얼 통신 위해 통신 Baudrate 맞춤

        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //serial::Timeout struct 생성해줌(1000ms)

        ser.setTimeout(to);//시리얼 데이터 기다리는 최대 시간 설정해줌.

        ser.open();//시리얼 포트 열어줌(세팅은 되어있으나 아직 열리지 않았을 경우에)

        }

 

    catch (serial::IOException& e)//포트 열지 못하면 에러 메시지, 함수 끝냄

    {

        ROS_ERROR_STREAM("Unable to open port ");

        return -1;

    }

 

    if(ser.isOpen()){ //시리얼 포트의 오픈 상태를 받아옴(열려있으면 TRUE)

        ROS_INFO_STREAM("Serial Port initialized");

    }else{ //닫혀있으면(FALSE), 함수 끝냄

        return -1;

    }

 

    ros::Rate loop_rate(50); //주기 20msec

        uint8_t answer[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //받아오는 배열

        uint8_t answer_quere[MAX]={0,}; //ros::ok() 안에서 사용됨

        uint8_t answer_tester[1]={0x00}; //첫번째부터 잘못된 값 나오면 바로 닫아버릴 수 있도록 따로 변수 설정

 

     if(ser.available()){ //버퍼에 있는 character(=받아온 인자들) 개수 반환, 0보다 크면 if문은 TRUE로 판단함

           // ROS_INFO_STREAM("Reading from serial port");

 

            while(answer_tester[0]! =0x53){ //0x53이 될 때까지 계속 굴림(맨처음에는 0x00이었음), 0x53 돼서야 while문 안걸리고 빠져나옴

                ser.read(answer_tester,1); //answer_tester에 값 1개 읽어옴

//                printf("%x ",answer_tester[0]);

            }

            answer_tester[0]={0x00}; //answer_tester에 다시 0x00 넣어줌 ???

            ser.read(answer,17); //answer에 값 17개 읽어옴

 

            if(answer[0]==0x54 && answer[1]==0x58)

            {

                if(answer[15]==0x0D && answer[16]==0x0A) //처음, 끝 조건 만족하면

            { //  for(int i=0;i<17;i++)         printf("%x ",answer[i]);

              //  printf("\n");
                
                
                /*주의!!
                 위 answer_tester에서 버퍼에 있던 0x53을 이미 읽어들였으므로
                 answer에는 0x54, 0x58....순으로 들어가게 됨. 즉, erp42 프로토콜 상 2번째 인자부터만 받아오는 것.
                 */

                    erp42_state.read_AorM=bool(answer[2]); //erp42_state 클래스의 raad_AorM에 값 저장.

                    erp42_state.read_E_stop=bool(answer[3]);

                    erp42_state.read_gear=answer[4];

                    erp42_state.read_speed=int(answer[5]);//speed0, speed1은 어디에?

                    erp42_state.read_brake=int(answer[9]);

                    erp42_state.read_ENC=int(answer[13])*256*256*256+int(answer[12])*256*256+int(answer[11])*256+int(answer[10]);

                    //12,13,14,15번째 인자에 각각 (2^8)^0, (2^8)^1, (2^8)^2, (2^8)^3 곱한 후 모두 더함 ==> 엔코더 값 알아냄

                    ENC_saver[0]=erp42_state.read_ENC; //읽은 엔코더값 저장해줌

                    ENC_saver[1]=erp42_state.read_ENC;

                    ENC_saver[2]=erp42_state.read_ENC;

                    ENC_saver[3]=erp42_state.read_ENC;

 

            }

        }

     }

 

 

    while(ros::ok()){ //ctrl+c를 누르거나 ros::shutdown()을 실행하진 않았나 체크함, 여전히 OK면 TRUE

    ros::spinOnce(); //한 번만 콜백 함수 호출. ros::spin()의 경우 콜백 함수 내에서만 태스크 모두 수행할 때 사용함

    last_time = current_time; //아까 받아뒀던 current_time 값으로 초기화해줌(t0)

    current_time = ros::Time::now(); //현재 시간 초기화(t)

    dt = (current_time - last_time).toSec(); //dt를 초 단위로 변환해줌

 

    uint8_t a; //allive값 카운팅(매번 값 받아올 때마다 1씩 증가,255가 넘으면 다시 0부터 시작)

    a++;

 

    uint8_t ask[14]={0x53,0x54,0x58,0x01,0x00,gear,0x00,speed,steer1,steer2,brake,a,0x0D,0x0A}; //보내는 값, speed0은 사용하지 않음(0x00)

    ser.write(ask,14);//시리얼 포트에 ask에 있는 14개 인자 넣어줌 = erp42가 값 받음

 

        if(ser.available()){ //버퍼에 있는 character(=받아온 인자들) 개수 반환, 0보다 크면 if문은 TRUE로 판단함

            ser.read(answer_quere,18); //answer_quere에 버퍼에 있던 18개 인자 읽어옴

            

            for(int i=0; i<MAX;i++){

                printf("%x ",answer_quere[i]); //answer_quere 안에 있는 인자들 차례대로 출력해줌, 모니터링

            }

            printf("%f \n",dt); //dt값 출력

        }

 

    if(answer_quere[0]!=0x53 || answer_quere[1]!=0x54 || answer_quere[2]!=0x58 || answer_quere[16]!=0x0D || answer_quere[17]!=0x0A)
        //처음, 끝 조건 확인했는데 한개라도 삑나면

    {

        ser.flushOutput(); //output 버퍼 비움

        for(int i=0; i<MAX;i++){ //answer_quere 모든 인자 0x00으로 초기화시킴

            answer_quere[i]={0x00};

        }

        while(answer_tester[0]!=0x0A){ //0x53이 될 때까지 굴려줌.

            ser.read(answer_tester,1); //0x0A가 아니면 1개 읽어옴, ex)현재까지 갖고온 값이 0x0D이면 여기서 0x0A를 읽어옴(이 다음엔 0x53 읽을 차례가 됨)

            ROS_INFO("DORMAMU %x",answer_tester[0]); //지금 어디까지 읽어왔나를 표시해줌

        }

    }

 

    else{ //처음, 끝 조건 제대로 맞으면(값 제대로 받아오면) answer_quere에 받아온 현재 erp42의 상태값 각 변수들에 저장해줌

        erp42_state.read_AorM=bool(answer_quere[3]);

        erp42_state.read_E_stop=bool(answer_quere[4]);

        erp42_state.read_gear=answer_quere[5];

        erp42_state.read_speed=int(answer_quere[6]); //speed0, speed1은 어디에?

        erp42_state.read_brake=int(answer_quere[10]);

        erp42_state.read_ENC=int(answer_quere[14])*256*256*256+int(answer_quere[13])*256*256+int(answer_quere[12])*256+int(answer_quere[11]);
        //12,13,14,15번째 인자에 각각 (2^8)^0, (2^8)^1, (2^8)^2, (2^8)^3 곱한 후 모두 더함 ==> 엔코더 값 알아냄
    

        for(int i=2; i>=0; i--)

        {

            ENC_saver[i+1]=ENC_saver[i]; //ENC_saver의 i+1번째 인자의 값 i번째 인자로 바꿔줌(i+1번째는 바로 뒤에서 현재 ENC값으로 바꿔줌)

        }

        ENC_saver[0]=erp42_state.read_ENC; //ENC_saver의 첫 번째 인자에 현재 엔코더값 넣어줌. 즉, 0번으로 갈수록 최신값임.

        erp42_state.read_velocity=0.01651*((ENC_saver[0]-ENC_saver[2]))/2/dt;
        //0.01651이 지름이라 2로 나눠서 반지름표현, v = s/t꼴이라 볼 수 있음 ( s = r*theta 정도로 보임 )

 

    

        for(int i=2; i>=0; i--)

        {

        vel_saver[i+1]=vel_saver[i]; //vel_saver의 i+1번째 인자의 값 i번째 인자로 바꿔줌(i+1번째는 바로 뒤에서 현재 velocity값으로 바꿔줌)

        }

        vel_saver[0]=erp42_state.read_velocity; //vel_saver의 첫 번째 인자에 현재 속도값 넣어줌. 즉, 0번으로 갈수록 최신값임.

 

        erp42_state.read_accel=((vel_saver[0]-vel_saver[1]))/dt; //a = v/t꼴이라 볼 수 있음

        erp42_state.read_steer=int(answer_quere[9])*256+int(answer_quere[8]); //steer값 모니터링할 수 있게 변환해줌

 

        if(erp42_state.read_steer>32768) erp42_state.read_steer=erp42_state.read_steer-65536+1; //보수처리

 

        erp42_state.read_yaw=erp42_state.read_yaw+0.01652*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;
        //yaw값 저장(heading 관련인듯)

        erp42_state.read_s=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*cos( erp42_state.read_yaw/180/PI); //사실 y임
        //y값 저장(적분해서 계속 더해감, 위치 관련)

        erp42_state.read_l=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*sin( erp42_state.read_yaw/180/PI); //사실 x임
        //x값 저장(적분해서 계속 더해감, 위치 관련)

        read_pub.publish(erp42_state); //현재 상태 값 퍼블리시, 각 노드들에서 해당 정보 이용할듯

        ser.flush(); //input, output 버퍼 모두 비움

        loop_rate.sleep(); //loop 다시 돌 때까지 sleep

 

    }

                  //엔코더로 yaw를 구하는 식. 보정필요, IMU로 대체.

                    //erp42_state.read_yaw=erp42_state.read_yaw+0.01651*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;

 }

}

        

 
