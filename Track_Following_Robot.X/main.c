#include <stdio.h>
#include <p18f4525.h>
#include "sumovore.h"
#include "motor_control.h"
#include "interrupts.h"
#include <xc.h>

//PID controller constants
const char P_CONSTANT = 0;
const char I_CONSTANT = 0;
const char D_CONSTANT = 0;

void PID_LineFollowing(char error[]);
void PID_SetMotorSpeed(int delta_velocity);

void main(void)
{
    char error[2]={0};
    initialization(); // function from sumovore.c
                      // it sets up pwm (using timer2),
                      // IO pins, the ADC, the 
                      // USART and the default
                      // threshold
    printf("\n\rKwantlen APSC1299 simple curve follower -- with error codes\n\r"); 
    ClrWdt();         // defined in <p18f4525.h>

    threshold = 620u; // to change from default value
                     // uncomment and change to any unsigned int <1024u -- most usually <512u
    
    
    OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_8BIT & T0_PS_1_4);
    WriteTimer0(0);
    TMR0IF = 0;
    
    while(1)
    {
        check_sensors();    //Write sensor status to SeeLine.B variable
        
        //Set error value based on sensor status
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
           default:
               error[2]=0;    break;
         }
        set_leds();         // function from sumovore.c
        PID_LineFollowing(error);
        
        ClrWdt();           // defined in <p18f4525.h>
        if(lvd_flag_set())  LVtrap();
    }
}

//Function to calculate delta velocity using PID
void PID_LineFollowing(char error[])
{
    static int delta_velocity, P = 0, I = 0, D = 0;
    static unsigned int flag_count = 0;
    
    //Calculate P
    P = error[2];
    
    //Calculate I & increment flag counter
    if (TMR0IF)
    {
        flag_count++;
        TMR0IF = 0;
        if(!(delta_velocity>=1600 || delta_velocity<=1600) || (delta_velocity>=1600 && error[2]<0) || (delta_velocity<=-1600 && error[2]>0))
            //This is to prevent integral windup
        if( !(I>=3273 || I<=-3274) || (I>=3273 && error[2]<0) || (I<=-3274 && error[2]>0))
            //If I is not near max magnitude, or if I is near max magnitude but will decrease 
            I += error[2]; //Integral value, maximum magnitude reached when held at max error for 1s
    }
    
    //Calculate D
    if((error[1]!=error[2]) || flag_count >= 16384) //If there is a delta error or ~2s have passed
    {
        D = (error[2]-error[1])/flag_count;
        flag_count = 0;
        error[1]=error[2];
    }
    
    //Calculate Delta Velocity
    delta_velocity = P_CONSTANT*P + I_CONSTANT*I + D_CONSTANT*D;
    
    PID_SetMotorSpeed(int delta_velocity);
}

//Function to set motor speed based on a delta velocity
void PID_SetMotorSpeed(int delta_velocity)
{
    int left_duty_cycle = 700, right_duty_cycle = 700;  //Base speed of motors set to 700 out of 800
    enum e_direction {reverse,forward} left_dir_modifier= forward, right_dir_modifier= forward;
    
    //if delta velocity negative turn CCW toward line
    if ( delta_velocity < 0 )
    {
        left_duty_cycle += delta_velocity*15/16;
        right_duty_cycle -= delta_velocity/16;
    }
    //if delta velocity positive turn CW toward line
    else
    {
        left_duty_cycle += delta_velocity/16;
        right_duty_cycle -= delta_velocity*15/16;
    }
    //Since the motor speed can be decreased by up to 1500 units (700 10 -800)
    //But the motor speed can only be increased by up to 100 (700 to 800)
    //They take 15/16 and 1/16 of the delta velocity respectively
    
    //set left motor
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
    
    //set right motor
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