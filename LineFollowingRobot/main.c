/*
 * LineFollowingRobot.c  -- race version
 */
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

/* Output defines */
#define ledLeftOn      PORTA |= (1<<PA1)
#define ledLeftOff     PORTA &= ~(1<<PA1)
#define ledRightOn     PORTC |= (1<<PC4)
#define ledRightOff    PORTC &= ~(1<<PC4)

#define LeftControlCW   PORTB |= (1<<PB0)
#define LeftControlCCW  PORTB &= ~(1<<PB0)
#define RightControlCW  PORTB |= (1<<PB3)
#define RightControlCCW PORTB &= ~(1<<PB3)

/* Input defines */
#define leftBumper    (!(PINA & (1 << PA2)))   // active low
#define rightBumper   (!(PINA & (1 << PA0)))   // active low
#define leftSensor    (PINA & (1 << PA7))
#define centreSensor  (PINA & (1 << PA6))
#define rightSensor   (PINA & (1 << PA3))

#define CCW 1
#define CW  0

/* ---- Tunable speeds (trial and error on the track) ---- */
#define FWD_SPEED    50   // straight line
#define SOFT_OUTER   70   // outer wheel in a gentle turn
#define SOFT_INNER   30   // inner wheel in a gentle turn
#define HARD_OUTER   70   // outer wheel in a hard turn
#define HARD_INNER   0   // inner wheel in a hard turn (use e.g. 20 for smoother)
#define REV_SPEED    60
#define REV_TIME_MS 400

/* Motor macros (Left: CW = forward, Right: CCW = forward) */
#define Forward       { SetRightMotorSpeedandDirection(FWD_SPEED, CCW);   SetLeftMotorSpeedandDirection(FWD_SPEED, CW); }
#define GentleRight   { SetRightMotorSpeedandDirection(SOFT_INNER, CCW);  SetLeftMotorSpeedandDirection(SOFT_OUTER, CW); }
#define GentleLeft    { SetRightMotorSpeedandDirection(SOFT_OUTER, CCW);  SetLeftMotorSpeedandDirection(SOFT_INNER, CW); }
#define HardRight     { SetRightMotorSpeedandDirection(HARD_INNER, CCW);  SetLeftMotorSpeedandDirection(HARD_OUTER, CW); }
#define HardLeft      { SetRightMotorSpeedandDirection(HARD_OUTER, CCW);  SetLeftMotorSpeedandDirection(HARD_INNER, CW); }
#define ReverseStraight  { SetRightMotorSpeedandDirection(REV_SPEED, CW); SetLeftMotorSpeedandDirection(REV_SPEED, CCW); }
#define ReverseSwingLeft  { SetRightMotorSpeedandDirection(REV_SPEED, CW); SetLeftMotorSpeedandDirection(REV_SPEED/2, CCW); }
#define ReverseSwingRight { SetRightMotorSpeedandDirection(REV_SPEED/2, CW); SetLeftMotorSpeedandDirection(REV_SPEED, CCW); }
#define Halt          { SetRightMotorSpeedandDirection(0, CCW); SetLeftMotorSpeedandDirection(0, CCW); }

void setup(void);
void SetLeftMotorSpeedandDirection(unsigned char Speed, char Direction);
void SetRightMotorSpeedandDirection(unsigned char Speed, char Direction);
void updateLEDs(void);
void bumperControls(void);
void sensorControls(void);

/* remembers which side the line was last seen on (for lost-line recovery) */
static char lastSide = 0;   // 0 = left, 1 = right

int main(void)
{
	setup();
	//_delay_ms(2000);        // time to place the robot

	while (1)
	{
		bumperControls();   // blocks briefly only if a bumper is pressed
		sensorControls();
		updateLEDs();
	}
}

void setup(void)
{
	DDRA |= (1 << DDA1) | (1 << DDA5);   // LED1 and PWM1
	DDRC |= (1 << DDC4) | (1 << DDC0);   // LED2 and PWM2
	DDRB |= (1 << DDB0) | (1 << DDB3);   // Direction pins

	// If bumper switches have no external pull-ups, uncomment:
	// PORTA |= (1 << PA0) | (1 << PA2);

	TCCR0A = (1 << WGM01) | (1 << WGM00);   // fast PWM, TOP = 0xFF
	TCCR0B = (1 << CS01) | (1 << CS00);     // /64 -> ~490 Hz (was /1024 = 30 Hz, choppy)
	OCR0A = 0;
	OCR0B = 0;
}

/************************ Left motor ************************/
void SetLeftMotorSpeedandDirection(unsigned char Speed, char Direction)
{
	if (Speed > 100) Speed = 100;

	if (Speed == 0)
	{
		TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
		OCR0B = 0;
		LeftControlCCW;
	}
	else
	{
		OCR0B = (unsigned char)(((unsigned int)Speed * 255) / 100);

		if (Direction == CW)    // forward
		{
			TCCR0A |= (1 << COM0B1);
			TCCR0A &= ~(1 << COM0B0);
			LeftControlCCW;
		}
		else                    // reverse
		{
			TCCR0A |= (1 << COM0B1) | (1 << COM0B0);
			LeftControlCW;
		}
	}
}

/************************ Right motor ************************/
/* Forward (CCW) drives the DIR pin high, which inverts the PWM at the bridge,
 * so forward uses inverting mode (the same "100 - speed" idea as the left motor
 * in reverse). Now Speed = 40 really means ~40%. */
void SetRightMotorSpeedandDirection(unsigned char Speed, char Direction)
{
	if (Speed > 100) Speed = 100;

	if (Speed == 0)
	{
		TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
		OCR0A = 0;
		RightControlCCW;
	}
	else
	{
		OCR0A = (unsigned char)(((unsigned int)Speed * 255) / 100);

		if (Direction == CCW)   // forward
		{
			TCCR0A |= (1 << COM0A1) | (1 << COM0A0);   // inverting
			RightControlCW;
		}
		else                    // reverse
		{
			TCCR0A |= (1 << COM0A1);                   // non-inverting
			TCCR0A &= ~(1 << COM0A0);
			RightControlCCW;
		}
	}
}

/* Left LED = left sensor, right LED = right sensor (centre ignored) */
void updateLEDs(void)
{
	if (leftSensor)  ledLeftOn;  else ledLeftOff;
	if (rightSensor) ledRightOn; else ledRightOff;
}

/* Bumpers: reverse and swing away from the obstacle */
void bumperControls(void)
{
	if (leftBumper && rightBumper)
	{
		ledLeftOn; ledRightOn;
		ReverseStraight;
		_delay_ms(REV_TIME_MS);
	}
	else if (leftBumper)
	{
		ledLeftOn;
		ReverseSwingLeft;
		_delay_ms(REV_TIME_MS);
	}
	else if (rightBumper)
	{
		ledRightOn;
		ReverseSwingRight;
		_delay_ms(REV_TIME_MS);
	}
}

/* Sensor truth table (L C R), 1 = black line seen
 *  000 -> lost line: keep turning toward the side the line was last seen
 *  001 -> hard right        100 -> hard left
 *  010 -> forward
 *  011 -> gentle right      110 -> gentle left
 *  101 -> forward (unusual)  111 -> forward (junction/wide line)
 */
void sensorControls(void)
{
	unsigned char s = 0;
	if (leftSensor)   s |= 4;
	if (centreSensor) s |= 2;
	if (rightSensor)  s |= 1;

	switch (s)
	{
		case 0b010: Forward;      break;
		case 0b011: GentleRight;  lastSide = 1; break;
		case 0b110: GentleLeft;   lastSide = 0; break;
		case 0b001: HardRight;    lastSide = 1; break;
		case 0b100: HardLeft;     lastSide = 0; break;
		case 0b000:
		if (lastSide) { HardRight }
		else          { HardLeft }
		break;
		default:    Forward;      
		break;   
	}
}