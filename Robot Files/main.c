#include <stdio.h>
#include <p18f4525.h>
#include "sumovore.h"
#include "interrupts.h"
#include <xc.h>

//PID controller constants
const unsigned int P_CONSTANT = 0;  //proportional constant
const unsigned int I_CONSTANT = 0;  //inversly proportional constant
const unsigned int D_CONSTANT = 0;  //Proportional constant

unsigned int flag_count = 0;    //Timer0 flag counter
int I = 0;  //Integral

void PID_LineFollowing(signed char error[]);
void MotorControl(int delta_velocity);
void SharpTurn(char direction);
void GetBackonTrack(signed char last_error);

void main(void)
{
    signed char error[3]={0};   //Keep last 3 values of error
    initialization(); // function from sumovore.c
                      // it sets up pwm (using timer2),
                      // IO pins, the ADC, the 
                      // USART and the default
                      // threshold
    printf("\n\rKwantlen APSC1299 simple curve follower -- with error codes\n\r"); 
    ClrWdt();         // defined in <p18f4525.h>

    threshold = 620u; // to change from default value
                     // uncomment and change to any unsigned int <1024u -- most usually <512u
    
    
    OpenTimer0(TIMER_INT_OFF & T0_SOURCE_INT & T0_16BIT & T0_PS_1_128);
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
           case 0b00111u:
           case 0b00101u:
               //SharpTurn(1);    //Right turn
           case 0b11100u:
           case 0b10100u:
               //SharpTurn(0);    //Left turn
           case 0b00000u:
               //GetBackonTrack(error[1]);
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
void PID_LineFollowing(signed char error[])
{
    static int delta_velocity = 0;
    //Integral defined as global variable
    static int D = 0;   //Derivative
    
    //Calculate I
    if (!(ReadTimer0()%I_CONSTANT))  //Integral will only increment if Timer reads a multiple of I_CONSTANT
    {
        if(!(delta_velocity>=1600 || delta_velocity<=1600) || (delta_velocity>=1600 && error[2]<0) || (delta_velocity<=-1600 && error[2]>0))
            //If motor is saturated Integral will not wind up
        if( !(I>=3273 || I<=-3274) || (I>=3273 && error[2]<0) || (I<=-3274 && error[2]>0))
            //If I is not near max magnitude, or if I is near max magnitude but will decrease 
            I += error[2]; //Integral value
    }
    
    //Calculate D
    if(error[1]!=error[2]) //If there is a delta error
    {
        D = D_CONSTANT/ReadTimer0()*(error[2]-error[1]);
        //Record error
        error[0]=error[1];
        error[1]=error[2];
    }
    
    if(TMR0IF) //If ~1s have passed
    {
        D = D_CONSTANT/65535*(error[1]-error[0]);
        TMR0IF = 0;
    }
    
    
    //Calculate Delta Velocity
    delta_velocity = P_CONSTANT*error[2] + I + D;    //Motor saturates at a delta velocity of |1600|
    
    MotorControl(delta_velocity);  //Send Delta velocity to Motor control function
}

//Function to set motor speed based on a delta velocity
void MotorControl(int delta_velocity)
{
    int left_duty_cycle = 700, right_duty_cycle = 700;  //Base speed of motors set to 700 out of 800
    enum e_direction {reverse,forward} left_dir_modifier= forward, right_dir_modifier= forward;
    
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
 
//Function for sharp turns
void SharpTurn(char direction)
{
    int delta_velocity;
    
    flag_count = 0; //Reset counter
    while (SeeLine.B && flag_count!=7813) //While any sensor triggered and less than 1 sec elapsed
    {
        MotorControl(I*I_CONSTANT);        //Continue on line
        if (TMR0IF)                             //increment counter
        {
            flag_count++;
            TMR0IF = 0;
        }
        
    }
    
    if(!SeeLine.B) //If no sensor triggered
    {
        motors_brake_all(); //Stop
        if(direction)   //If right side sensors triggered last
        {
            delta_velocity = 1600;      //Turn CW
          while (!SeeLine.b.CntLeft && delta_velocity)    //Start going more and more forward if center triggered
          {
              MotorControl(delta_velocity);
              if (SeeLine.b.Center)
                  delta_velocity--;
          }
        }
        else            //If left side sensors triggered last
        {
            delta_velocity = -1600;     //Turn CCW
          while (!SeeLine.b.CntRight && delta_velocity)   //Start going more and more forward if center triggered
          {
              MotorControl(delta_velocity);
              if (SeeLine.b.Center)
                  delta_velocity++;
          }
        }
        I = 0;          //Reset Integral term
    }
    
    flag_count = 0; //Reset counter
}

//Function to get robot on track if it is off
void GetBackonTrack(signed char last_error)
{
    if(last_error>1)        //If lost track on a right turn
        SharpTurn(1);
    else if(last_error<-1)  //If lost track on a left turn
        SharpTurn(0);
    else
    {
        flag_count = 0; //Reset counter
        while (!SeeLine.B && flag_count!=7813) //While no sensor triggered and less than 1 sec elapsed
        {
            MotorControl(0);        //go straight
            if (TMR0IF)             //increment counter
            {
                flag_count++;
                TMR0IF = 0;
            }
        }

        if(!SeeLine.B)    //If no sensor is triggered
        {
            motors_brake_all(); //Stop
            MotorControl(1600); //Start turning CW
            for(char i=0; i < 80; i++)   //Keep spinning until a full rotation
                _delay(100000);

            while (!SeeLine.B)      //While no sensor triggered
                MotorControl(0);    //Go forward 
        }
        flag_count = 0; //Reset counter
    }
}