#include "Fire_motor_485.h"
#include <GCS_MAVLink/GCS.h>    //地面站


// Fire_motor_485::Fire_motor_485(/* args */)
// {
// }

void Fire_motor_485::motor_init()
{
    // hal.serial(2)->begin(19200); // 初始化串口程序，电机波特率设置成57600

    // hal.scheduler->delay(100); // 等待初始化串口
    // write_one(100,0x2009,6);
    // hal.scheduler->delay(100); // 设置串口速度
    // write_one(101,0x2009,6);
    // hal.scheduler->delay(100); // 设置串口速度
    // hal.serial(2)->begin(57600); // 初始化串口程序
    // hal.scheduler->delay(100);   // 延时等待串口初始化
    hal.serial(2)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(2)->set_unbuffered_writes(true);
    hal.scheduler->delay(5); // 设置串口速度
    write_two(100, 0x2003, 10, 5);
    hal.scheduler->delay(5); // 设置电机加减速时间
    write_two(101, 0x2003, 10, 5);
    hal.scheduler->delay(5); // 等待初始化串口
}

void Fire_motor_485::read_one(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x03;       // 读取的功能码为03
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(2)->write(data_to_send, cnt);
}

void Fire_motor_485::write_one(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x06;       // 写入的功能码为06
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(2)->write(data_to_send, cnt);
}

void Fire_motor_485::write_two(uint8_t address_ID, uint16_t start_reg_adress, uint16_t val_1, uint16_t val_2) // 写两个寄存器，用于归零
{
    uint8_t data_to_send[15];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num = 0x0002;      // 怀疑写入地址错误，未验证，若验证清删除
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x10;       // 写入的功能码为0x10
    data_to_send[cnt++] = BYTE1(start_reg_adress);
    data_to_send[cnt++] = BYTE0(start_reg_adress);
    data_to_send[cnt++] = BYTE1(write_num); // 写入数量
    data_to_send[cnt++] = BYTE0(write_num);
    data_to_send[cnt++] = 0x04; // 写入字节数
    data_to_send[cnt++] = BYTE1(val_1);
    data_to_send[cnt++] = BYTE0(val_1);
    data_to_send[cnt++] = BYTE1(val_2);
    data_to_send[cnt++] = BYTE0(val_2);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(2)->write(data_to_send, cnt);
}

void Fire_motor_485::change_address(uint8_t addressID,uint16_t val) //初始化需要进行设置
{
    // read_one()
    write_one(0x01,0x2008,100);    //设定电机控制器ID为100和101,只有调试运行一次
    // write_one(0x2008,101);    //设定电机控制器ID为100和101,只有调试运行一次
}

void Fire_motor_485::set_Forward(uint8_t addressID)
{
    write_one(addressID,0x2000,0x0001);
}

void Fire_motor_485::set_Reverse(uint8_t addressID)
{
    write_one(addressID,0x2000,0x0002);
}

void Fire_motor_485::shutdown(uint8_t addressID)                     //停机
{
    write_one(addressID,0x2000,0x0005);
}

void Fire_motor_485::free_stop(uint8_t addressID)                   //自由停机(紧急停机) 
{
    write_one(addressID,0x2000,0x0006);
}

void Fire_motor_485::agent_stop(uint8_t addressID)                   //电磁刹车停机
{
    write_one(addressID,0x2000,0x0009);
}

void Fire_motor_485::set_RPM(uint8_t addressID,uint16_t RPM)         //设定电机转速
{
    write_one(addressID,0x2001,RPM);
}

void Fire_motor_485::set_acc_time(uint8_t addressID,uint16_t time)   //设定加速时间单位0.1S
{
    write_one(addressID,0x2003,time);
}

void Fire_motor_485::set_redu_time(uint8_t addressID,uint16_t time)  //设定减速时间单位0.1S
{
    write_one(addressID,0x2004,time);
}

void Fire_motor_485::set_motor_poles(uint8_t addressID,uint16_t num) //设定电机极对数
{
    write_one(addressID,0x2002,num);
}

void Fire_motor_485::read_drive1_status(uint8_t addressID)           //驱动器状态字1
{
    read_one(addressID,0x2100,0x0001);
}

void Fire_motor_485::read_drive2_status(uint8_t addressID)           //驱动器状态字2
{
    read_one(addressID,0x2101,0x0001);
}

void Fire_motor_485::drive_error(uint8_t addressID)                 //故障代码
{
    read_one(addressID,0x2102,0x0001);
}

void Fire_motor_485::read_RPM(uint8_t addressID)
{
    read_one(addressID, 0x3005, 0x0002);                          //读取电机输出设定转速和输出转速
    // read_one(addressID, 0X0914, 0x0002);
}

void Fire_motor_485::update_status()
{
    read_drive1_status(Left_motor);
    read_drive1_status(Right_motor);
}


void Fire_motor_485::function_fire_motor_485(uint8_t DT_ms)
{
    const float dwb_2 = 0.2f;       //系数为6.666
    const float vc = 0.00288;
    const float w = 0.0144;
    const float x_x = 2314.814f;
    float V_L,V_R,x,y;          //设定速度量为（0～100）
    // uint16_t under_offset = 1550;   //死区设置
    // uint16_t low_offset = 1450;
    uint16_t mid_offset = 1500;
    uint8_t static  stop_button = 0;
    uint16_t rcin_1 = hal.rcin->read(1); 
    uint16_t rcin_0 = 3000 - hal.rcin->read(0); 
    uint16_t rcin_9 = hal.rcin->read(9); 
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin(9):%d",rcin_9);
    static uint16_t last_rcin_9 = mid_offset;    
    static uint8_t golab_cnt = 0; //判断朝向，当为0的时候没有朝向，为1的时候向前，2的时候向后    
    // static uint8_t L_cnt=0,R_cnt=0;
    // static int16_t last_V_R  = 0;
    // static int16_t last_V_L  = 0;

    // gcs().send_text(MAV_SEVERITY_CRITICAL,"按键标志位:%d",abs(rcin_9 - last_rcin_9));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_1标志位:%d",abs(rcin_1 - mid_offset));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_0志位:%d",abs(rcin_0 - mid_offset));
    if((abs(rcin_1 - mid_offset) >50) || (abs(rcin_0 - mid_offset) >50))
    {
        y = (rcin_1 - mid_offset)*vc;
        x = (rcin_0 - mid_offset)*w;
        V_R = (y + dwb_2 * x)*x_x;
        V_L = (y - dwb_2 * x)*x_x;

    }
    else
    {
        V_R = 0;
        V_L = 0;
    }
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"V_L:%f",V_L);
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"V_R:%f",V_R);
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"IF:%d",(abs(rcin_1 - mid_offset) >100) || (abs(rcin_0 - mid_offset) >100));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"abs:%d",abs(rcin_1 - mid_offset));


    if(abs(rcin_9 - last_rcin_9) > 800) //判断是否有案件按下
    {
        stop_button = ~stop_button;//相当于按键被按下
    }
    last_rcin_9 = rcin_9; 

    V_L = LIMIT(V_L,-2950,2950);   //输出限幅
    V_R = LIMIT(V_R,-2950,2950);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "右期望值为:%d", (int16_t)V_R);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "V_L:%d", (int16_t)V_L);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "V_R:%d", (int16_t)V_R);
    if( golab_cnt == 0)  //如果更新数值没有改变，则见不输出V_L != last_V_L &&
    {
        // V_L = -V_L;
        if (V_L > 5)
        {

            write_two(Left_motor,0X2000,2,(uint16_t)V_L);     //左边轮子反转
            
        }
        else if(V_L < -5)
        {

            write_two(Left_motor, 0X2000, 1, (uint16_t)(-V_L)); // 左边轮子正转
        }
        else if(abs(V_L) < 5)
        {

            if(stop_button)
            {
                write_two(Left_motor,0X2000,9,0);     //左边轮子刹车
            }
            else
            {
                write_two(Left_motor,0X2000,6,0);     //左边轮子刹车
            }

        } 
        // last_V_L = V_L;     

    }

    
    else if(golab_cnt == 1)  //如果更新数值没有改变，则见不输出V_R != last_V_R 
    {
        V_R = -V_R;
        if (V_R > 5)
        {

            write_two(Right_motor,0X2000,2,(uint16_t)V_R); 
        }
        else if(V_R < -5)
        {
 
            write_two(Right_motor,0X2000,1,(uint16_t)(-V_R)); 
        }
        else if(abs(V_R) < 5)
        { 
            if(stop_button)
            {
                write_two(Right_motor,0X2000,9,0); 
            }
            else
            {
                write_two(Right_motor,0X2000,6,0); 
            }

        }
        // last_V_R = V_R;
    }
    else if (golab_cnt == 2)
    {
        // read_RPM(100);
    }
    else if (golab_cnt == 3)
    {
        // read_RPM(101);
    }

    golab_cnt++;

    if (golab_cnt == 4 /* condition */)
    {
        golab_cnt = 0;
        /* code */
    }

    
        /* code */
 
    
    

}

void Fire_motor_485::motor_input(int16_t motor_left, int16_t motor_right)
{
    // const float dwb_2 = 0.2f; // 系数为6.666
    // const float vc = 0.00288;
    // const float w = 0.0144;
    // const float x_x = 2314.814f;
    float V_L, V_R;;//, x, y; // 设定速度量为（0～100）
    // uint16_t under_offset = 1550;   //死区设置
    // uint16_t low_offset = 1450;
    uint16_t mid_offset = 1500;
    uint8_t static stop_button = 0;
    // uint16_t rcin_1 = hal.rcin->read(1);
    // uint16_t rcin_0 = 3000 - hal.rcin->read(0);
    uint16_t rcin_9 = hal.rcin->read(9);
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin(9):%d",rcin_9);
    static uint16_t last_rcin_9 = mid_offset;
    static uint8_t golab_cnt = 0; // 判断朝向，当为0的时候没有朝向，为1的时候向前，2的时候向后
    // static uint8_t L_cnt=0,R_cnt=0;
    // static int16_t last_V_R  = 0;
    // static int16_t last_V_L  = 0;

    // gcs().send_text(MAV_SEVERITY_CRITICAL,"按键标志位:%d",abs(rcin_9 - last_rcin_9));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_1标志位:%d",abs(rcin_1 - mid_offset));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_0志位:%d",abs(rcin_0 - mid_offset));
    if (abs(rcin_9 - last_rcin_9) > 800) // 判断是否有案件按下
    {
        stop_button = ~stop_button; // 相当于按键被按下
    }
    last_rcin_9 = rcin_9;

    V_L = LIMIT(motor_left, -2950, 2950); // 输出限幅
    V_R = LIMIT(motor_right, -2950, 2950);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "右期望值为:%d", (int16_t)V_R);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "V_L:%d", (int16_t)V_L);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "V_R:%d", (int16_t)V_R);
    if (golab_cnt == 0) // 如果更新数值没有改变，则见不输出V_L != last_V_L &&
    {
        // V_L = -V_L;
        if (V_L > 400)
        {

            write_two(Left_motor, 0X2000, 2, (uint16_t)V_L); // 左边轮子反转
        }
        else if (V_L < -400)
        {

            write_two(Left_motor, 0X2000, 1, (uint16_t)(-V_L)); // 左边轮子正转
        }
        else if (abs(V_L) < 400)
        {

            if (stop_button)
            {
                write_two(Left_motor, 0X2000, 6, 0); // 左边轮子刹车
            }
            else
            {
                write_two(Left_motor, 0X2000, 9, 0); // 左边轮子刹车
            }
        }
        // last_V_L = V_L;
    }

    else if (golab_cnt == 1) // 如果更新数值没有改变，则见不输出V_R != last_V_R
    {
        V_R = -V_R;
        if (V_R > 400)
        {

            write_two(Right_motor, 0X2000, 2, (uint16_t)V_R);
        }
        else if (V_R < -400 )
        {

            write_two(Right_motor, 0X2000, 1, (uint16_t)(-V_R));
        }
        else if (abs(V_R) < 400 )
        {
            if (stop_button)
            {
                write_two(Right_motor, 0X2000, 6, 0);
            }
            else
            {
                write_two(Right_motor, 0X2000, 9, 0);
            }
        }
        // last_V_R = V_R;
    }
    // else if (golab_cnt == 2)
    // {
    //     // read_RPM(100);
    // }
    // else if (golab_cnt == 3)
    // {
    //     // read_RPM(101);
    // }
    // if (abs(V_R) < 100 && (abs(V_L) < 100))
    // {
    //     FL.stop_motor();
    // }
    golab_cnt++;

    if (golab_cnt == 2 /* condition */)
    {
        golab_cnt = 0;
        /* code */
    }

    /* code */
}


