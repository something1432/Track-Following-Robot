#include <stdio.h>
#include <p18f4525.h>
#include "sumovore.h"
#include "interrupts.h"
#include <xc.h>

//PID controller constants
const unsigned int P_CONSTANT = 150;  //Proportional constant, value of 400 can just saturate motors by itself
const unsigned int I_CONSTANT = 50;  //Inversly proportional constant
const unsigned int D_CONSTANT = 8000;  //Proportional constant

//Time values for certain actions
const unsigned int TIME_TURN_ms = 800;          //Time in ms, it takes to turn around
const unsigned int TIME_UPDATE_D_ms = 500;      //Maximum time in ms to wait before updating D term if no change in error occured
const unsigned int TIME_SHARP_TURN_ms = 500;    //Time in ms, to look for a sharp turn after triggering sharp turn flag
const unsigned int TIME_OFF_TRACK_ms = 2000;    //Time in ms, off track before robot turns around
const unsigned int TIME_ON_BLACK_ms = 1000;     //Time in ms, on back before robot stops

//Global Variables
enum e_direction {reverse = 0, CCW =0, forward = 1, CW = 1}
CCW_rotation_flag = 0, CW_rotation_flag = 0, flag_counter_enabled = 0;//Also using as a general boolian
int I = 0;  //Integral component of PID

unsigned int I_flag_counter = 0;        //Flag counter for Integral term
unsigned int D_flag_counter = 0;        //Flag counter for Derivative term
unsigned int flag_counter;              //General timer0 flag counter

void PID_LineFollowing(signed char error[]);
void MotorControl(int delta_velocity);
void GetBackonTrack(signed char error[]);
void SharpTurn(enum e_direction direction);
void AllSensorsTriggered();
void FilterNearbyTrack();

void main(void)
{
    signed char error[3]={0};   //Keep last 3 values of error
    
    //From Sample code
    initialization(); // function from sumovore.c
                      // it sets up pwm (using timer2),
                      // IO pins, the ADC, the 
                      // USART and the default
                      // threshold
    printf("\n\rKwantlen APSC1299 simple curve follower -- with error codes\n\r"); 
    ClrWdt();         // defined in <p18f4525.h>
    threshold = 575u; // to change from default value
                     // uncomment and change to any unsigned int <1024u -- most usually <512u
    
    //Open timer
    OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_8BIT & T0_PS_1_2);
    WriteTimer0(0);
    TMR0IF = 0;
    
    while(1)
    {
        check_sensors();        //Write sensor status to SeeLine.B variable
        
        FilterNearbyTrack();    //Disable sensor detecting nearby track
        
        //Set error value based on sensor status or trigger appropriate function based on sensor status
        //The more the robot deviates to the left of the line (right side sensors being triggered)
        //The more positive the error value up to +4
        //The more the robot deviates to the right of the line (left side sensors being triggered)
        //The more negative the error value down to -4
        switch(SeeLine.B)
        {
           case 0b00110u:
               error[2]=1;    break;
           case 0b00010u:
               error[2]=2;    break;
           case 0b00011u:
               error[2]=3;    break;
           case 0b00001u:
               error[2]=4;    break;
           case 0b01100u:
               error[2]=-1;   break;
           case 0b01000u:
               error[2]=-2;   break;
           case 0b11000u:
               error[2]=-3;   break;
           case 0b10000u:
               error[2]=-4;   break;
           case 0b00111u:
           {
               flag_counter_enabled = 1;
               flag_counter = 0;
               CW_rotation_flag = 1;    //Set CW rotation flag
               CCW_rotation_flag = 0;   //Clear CCW rotation flag
           }break;
           case 0b11100u:
           {
               flag_counter_enabled = 1;
               flag_counter = 0;
               CCW_rotation_flag = 1;   //Set CCW rotation flag
               CW_rotation_flag = 0;    //Clear CW rotation flag
           }break;
           case 0b00000u:                       //If no sensors triggered
               GetBackonTrack(error);   break;  //Try to find the track
           case 0b11111u:                       //If all sensors triggered
               AllSensorsTriggered();   break;  //Do all sensors triggered action
           default:
               error[2]=0;    break;
         }
        
        //Increment timers
        if (TMR0IF)
        {
            I_flag_counter++;
            D_flag_counter++;
            
            if(flag_counter_enabled)
            {
                flag_counter++;
                if(flag_counter < 16*TIME_SHARP_TURN_ms)  //If no turn after ~500ms
                {
                    CW_rotation_flag = 0;
                    CCW_rotation_flag = 0;
                    flag_counter_enabled = 0;
                }
            }

            TMR0IF = 0;
        }
        
        set_leds();         // function from sumovore.c
        PID_LineFollowing(error);               //Execute Line following PID
        
        ClrWdt();           // defined in <p18f4525.h>
        if(lvd_flag_set())  LVtrap();
    }
}

//Function to calculate delta velocity using PID
void PID_LineFollowing(signed char error[])
{
    static int delta_velocity = 0;  //How much to change velocity
    //Integral defined as global variable
    static int D = 0;   //Derivative
    
    //Calculate I
    if (I_flag_counter == I_CONSTANT)  //Integral increments every I_CONSTANT*512 cycles
    {
        if(!(delta_velocity>=1600 || delta_velocity<=1600) || (delta_velocity>=1600 && error[2]<0) || (delta_velocity<=-1600 && error[2]>0))
            //If motor is saturated Integral will not wind up
            if( !(I>=3273 || I<=-3274) || (I>=3273 && error[2]<0) || (I<=-3274 && error[2]>0))
                //If Integral is not near max magnitude, or if I is near max magnitude but will decrease 
                I += error[2]; //Sigma error*time (error vs time graph integral/reimann sum)
        
        //Reset integral flag counter
        I_flag_counter = 0;
    }
    
    //Calculate D
    if((error[1]!=error[2]) || (D_flag_counter == 16*TIME_UPDATE_D_ms)) //If there is a change in error or TIME_UPDATE_D_ms has passed
    {
        D = D_CONSTANT/D_flag_counter*(error[2]-error[1]);    //delta_error/delta_time (error vs time graph derivative)
        
        //Record error
        error[0]=error[1];
        error[1]=error[2];
        
        //Reset derivative flag counter
        D_flag_counter = 0;
    }
    
    //Calculate Delta Velocity
    delta_velocity = P_CONSTANT*error[2] + I + D;   //The amount to change velocity is sum of PID terms
    //Motor saturates at a delta velocity of |1600|
    
    MotorControl(delta_velocity);  //Send Delta velocity to Motor control function
}

//Function to set motor speed based on a delta velocity
void MotorControl(int delta_velocity)
{
    int left_duty_cycle = 700, right_duty_cycle = 700;  //Base speed of motors set to 700 out of 800
    enum e_direction left_dir_modifier= forward, right_dir_modifier= forward;
    
    //if delta velocity negative turn CCW toward line
    if ( delta_velocity < 0 )
    {
        left_duty_cycle += delta_velocity*15/16;    //Decrease left motor by |15/16*delta_velocity|
        right_duty_cycle -= delta_velocity/16;      //Increase right motor by |delta_velocity/16|
    }
    //if delta velocity positive turn CW toward line
    else
    {
        left_duty_cycle += delta_velocity/16;       //Increase right motor by delta_velocity/16
        right_duty_cycle -= delta_velocity*15/16;   //Decrease right motor by 15/16*delta_velocity
    }
    //Since the motor speed can be decreased by up to 1500 units (700 10 -800)
    //But the motor speed can only be increased by up to 100 (700 to 800)
    //They take 15/16 and 1/16 of the delta velocity respectively
    
    //set left motor, Adapted from sumovore.c
    if ( left_duty_cycle < 0 ) 
    {
        left_dir_modifier = reverse;
        left_duty_cycle *= -1;
    }
    if ( left_duty_cycle > 800 ) left_duty_cycle = 800;
    
    SetDCPWM2((unsigned int) left_duty_cycle );
    if ( left_dir_modifier == reverse ) LmotorGoFwd = NO;
    else LmotorGoFwd = YES;
    LmotorGoFwdCmp = !LmotorGoFwd;
    
    //set right motor, Adapted from sumovore.c
    if ( right_duty_cycle < 0 ) 
    {
        right_dir_modifier = reverse;
        right_duty_cycle *= -1;
    }
    if ( right_duty_cycle > 800 ) right_duty_cycle = 800;
    
    SetDCPWM1((unsigned int) right_duty_cycle );
    if ( right_dir_modifier == reverse ) RmotorGoFwd = NO;
    else RmotorGoFwd = YES;
    RmotorGoFwdCmp = !RmotorGoFwd;
}
 
//Function to get robot on track if it is off
void GetBackonTrack(signed char error[])
{
    if((error[2]>1) || (error[1]>1) || CW_rotation_flag)        //If one of the last two recorded errors is 2 or higher
        SharpTurn(CW);                       //Do a CW turn
    else if((error[2]<-1) || (error[1]<-1) || CCW_rotation_flag) //If one of the last two recorded errors is -2 or lower
        SharpTurn(CCW);                       //Do a CCW turn
    else
    {
        flag_counter = 0;   //Initialize flag counter
        
        while (!SeeLine.B && (flag_counter < 16*TIME_OFF_TRACK_ms)) //While no sensor triggered and less than TIME_OFF_TRACK_ms elapsed
        {
            check_sensors();        //Update sensors
            set_leds();
            MotorControl(0);      //go straight
            if(TMR0IF)//Increment timer
            {
                flag_counter++;
                TMR0IF = 0;
            }
        }

        if(!SeeLine.B)    //If no sensor is triggered
        {
            motors_brake_all();     //Stop
            _delay(50000);
            MotorControl(1600);     //Start turning CW
            
            for(unsigned int i=0; i < TIME_TURN_ms; i++)   //Keep spinning until robot turns around
                _delay(7991);           //~1 millisecond delay

            while (!SeeLine.B)      //While no sensor triggered
            {
                check_sensors();    //Update sensors
                set_leds();
                MotorControl(0);    //Go forward
            }
        }
    }
}

//Function for sharp turns
void SharpTurn(enum e_direction direction)
{
    int delta_velocity;
    
    motors_brake_all(); //Stop
    _delay(50000);
    if(direction)   //if direction = CW
    {
        delta_velocity = 1600;      //Turn CW
      while (!SeeLine.b.CntLeft && delta_velocity)//If center left sensor not triggered
          //and robot has not straightened itself out
      {
          check_sensors();          //Update sensors
          set_leds();
          MotorControl(delta_velocity);
          if (SeeLine.b.Center)     //Start going more and more forward if center triggered
              delta_velocity--;
      }
        CW_rotation_flag = 0;       //Clear CW rotation flag after rotation complete
        flag_counter_enabled = 0;   //Disable flag counter used to time how long CW flag is triggered
    }
    else            //otherwise direction = CCW
    {
        delta_velocity = -1600;     //Turn CCW
      while (!SeeLine.b.CntRight && delta_velocity) //If center right sensor not triggered
          //and robot has not straightened itself out
      {
          check_sensors();          //Update sensors
          set_leds();
          MotorControl(delta_velocity);
          if (SeeLine.b.Center)     //Start going more and more forward if center sensor triggered
              delta_velocity++;
      }
        CCW_rotation_flag = 0;      //Clear CCW rotation flag after rotation complete
        flag_counter_enabled = 0;   //Disable flag counter used to time how long CCW flag is triggered
    }
    I = 0;          //Reset Integral term
}

//Function to deal with if all sensors get triggered
void AllSensorsTriggered()
{
    flag_counter = 0;   //Initialize flag counter
    
    while (SeeLine.B=0b11111u && (flag_counter<16*TIME_ON_BLACK_ms)) //While all sensor triggered and less than TIME_ON_BLACK_ms elapsed
    {
        check_sensors();        //Update sensors
        set_leds();
        MotorControl(0);      //go straight
        if(TMR0IF)//Increment timer
        {
            flag_counter++;
            TMR0IF = 0;
        }
    }
    if (SeeLine.B=0b11111u)         //If all sensors still triggered
    {
        motors_brake_all();         //Stop
        while (SeeLine.B=0b11111u)  //While all sensors triggered do nothing
        {
            check_sensors();
            set_leds();
        }
    }   
}

//Function to filter out nearby track
void FilterNearbyTrack()
{
    //If far left sensor triggered but center left sensor not triggered
    //and either center or center right sensor also triggered
    if((SeeLine.b.Left && !SeeLine.b.CntLeft) && (SeeLine.b.Center || SeeLine.b.CntRight))
        SeeLine.b.Left = 0;     //Disable far left sensor
    
    //If far right sensor triggered but center right sensor not triggered
    //and either center or center left sensor also triggered
    if((SeeLine.b.Right && !SeeLine.b.CntRight) && (SeeLine.b.Center || SeeLine.b.CntLeft))
        SeeLine.b.Right = 0;    //Disable far right sensor
}