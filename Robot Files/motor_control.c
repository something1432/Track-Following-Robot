#include "sumovore.h"
#include "motor_control.h"

void follow_simple_curves(void);
void spin_left(void);
void turn_left(void);
void straight_fwd(void);
void turn_right(void);
void spin_right(void);

void motor_control(void)
{
    char error;
     // very simple motor control
     switch(SeeLine.B)
     {
        case 0b00110u:
            error=1;    break;
        case 0b00010u:
            error=2;    break;
        case 0b00011u:
            error=3;    break;
        case 0b00001u:
            error=4;    break;
        case 0b01100u:
            error=-1;   break;
        case 0b01000u:
            error=-2;   break;
        case 0b11000u:
            error=-3;   break;
        case 0b10000u:
            error=-4;   break;
        case 0b11100u:
        case 0b10100u:
            SharpLeftTurn(e
        default:
            error=0;    break;
      } 
}

void follow_simple_curves(void)
{
     if ( SeeLine.b.Center ) straight_fwd();
     else if (SeeLine.b.Left) spin_left();
     else if (SeeLine.b.CntLeft) turn_left();
     else if (SeeLine.b.CntRight) turn_right();
    else if (SeeLine.b.Right) spin_right();
}

void spin_left(void)
{
  set_motor_speed(left, rev_fast, 0); 
  set_motor_speed(right, fast, 0); 
}

void turn_left(void)
{
  set_motor_speed(left, stop, 0); 
  set_motor_speed(right, fast, 0); 
}
void straight_fwd(void)
{
  set_motor_speed(left, fast, 0); 
  set_motor_speed(right, fast, 0); 
}
void spin_right(void)
{
  set_motor_speed(left, fast, 0); 
  set_motor_speed(right, rev_fast, 0); 
}
void turn_right(void)
{
  set_motor_speed(left, fast, 0); 
  set_motor_speed(right, stop, 0); 
}
