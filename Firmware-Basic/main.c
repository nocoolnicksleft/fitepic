#include <p18cxxx.h>
#include <p18f4550.h>
#include <delays.h>
#include <usart.h>
#include <i2c.h>
#include <timers.h>
#include <stdio.h>
#include <string.h>

#pragma config FOSC = INTOSCIO_EC, CPUDIV = OSC1_PLL2
#pragma config FCMEN = OFF, BOR = OFF
#pragma config WDT = OFF, MCLRE = ON, LVP = OFF

#define DEBUG_MODE 1

#define PORT_EX PORTAbits.RA0
#define PORT_EY PORTAbits.RA1
#define M1L PORTAbits.RA2
#define M1R PORTAbits.RA3
#define M2L PORTAbits.RA4
#define M2R PORTAbits.RA5

//#define I2C_SDA PORTBbits.RB0
//#define I2C_SCL PORTBbits.RB1
#define PORT_E7 PORTBbits.RB2
#define PORT_E8 PORTBbits.RB3
#define SW_RUN PORTBbits.RB4
#define LED_HEARTBEAT PORTBbits.RB5
#define ICSP_PGC PORTBbits.RB6
#define ICSP_PGD PORTBbits.RB7

#define M1EN PORTCbits.RC1
#define M2EN PORTCbits.RC2
#define M3EN PORTCbits.RC4
#define M4EN PORTCbits.RC5
#define M4L PORTCbits.RC6
#define M4R PORTCbits.RC7

#define PORT_E1 PORTDbits.RD0
#define PORT_E2 PORTDbits.RD1
#define PORT_E3 PORTDbits.RD2
#define PORT_E4 PORTDbits.RD3
#define PORT_E5 PORTDbits.RD4
#define PORT_E6 PORTDbits.RD5
#define SERIAL_TX PORTDbits.RD6
#define SERIAL_RX PORTDbits.RD7

#define M3L PORTEbits.RE0
#define M3R PORTEbits.RE1

#define MCLR PORTEbits.RE3


#define INPUT_MASK_PORTA 0b00000011
#define INPUT_MASK_PORTB 0b00001100
#define INPUT_MASK_PORTC 0b11000000 // Bit 6 / 7: UART
#define INPUT_MASK_PORTD 0b00111111
#define INPUT_MASK_PORTE 0b00000000





#define M1_ON M1_ENABLE = 1;
#define M1_OFF M1_ENABLE = 0;

#define M1 0
#define M2 1
#define M3 2
#define M4 3

#define E1 0x01
#define E2 0x02
#define E3 0x03
#define E4 0x04
#define E5 0x05
#define E6 0x06
#define E7 0x07
#define E8 0x08

#define RIGHT 0
#define LEFT 1

#define SPEED_FULL 255
#define SPEED_HALF 128
#define SPEED_DEADSLOW  32
#define SPEED_STOP 0

#define STATUS_STOPPED 0x0
#define STATUS_RUNNING 0x1
#define STATUS_ALLUP 0x2
#define STATUS_ALLDOWN 0x3
#define STATUS_1UP 0x4
#define STATUS_2UP 0x5
#define STATUS_OFF 0x6

#define TIMEOUT_DEFAULT 200

#define NO_REPEAT 0

struct drivercommand {
 unsigned char driverid;
 unsigned char direction;
 unsigned char speed;
 unsigned char endswitchid;
 unsigned int timeout;
 unsigned int countswitchid;
 unsigned int counts;
 unsigned int repeat;
 void (* endfunction)(struct drivercommand * command);
};

void high_isr(void);
void low_isr(void);
/*
void motor1_direction(int);
void motor1_on();
void motor1_off();
*/
void set_driver_command(struct drivercommand *);

void handlekeycode(unsigned int);
void dokeyboard();
char pop();

unsigned int global_input_state = 0;
  
/**********************************************************
/ Motor Functions
/**********************************************************/
#define NUMBER_OF_DRIVERS 4

struct driver {
	int driverid;
    int status;
    int countdown;
    struct drivercommand * currentcommand;
};

struct driver drivers[NUMBER_OF_DRIVERS] = {{0,0,0,0},{1,0,0,0}};
/*
void motor1_direction(int direction) 
{
  if (direction) {

    M1_INPUT1 = 0;
    M1_INPUT2 = 1;

  } else {

    M1_INPUT1 = 1;
    M1_INPUT2 = 0;

  }
}

void motor1_on() 
{
  M1_ENABLE = 1;
}

void motor1_off() 
{
  M1_ENABLE = 0;
}
*/
void driver_run_command(struct drivercommand * cmd)
{

	drivers[cmd->driverid].currentcommand = cmd;
    drivers[cmd->driverid].status = STATUS_RUNNING;
	drivers[cmd->driverid].countdown = cmd->timeout;

    if (cmd->driverid == M1) {
		
		if (cmd->direction == LEFT) {

	    	M1L = 1;
	    	M1R = 0;
	
	 	} else {
	
	    	M1L = 0;
	    	M1R = 1;

		}

		M1EN = 1;

	} else if (cmd->driverid == M2) {

		if (cmd->direction) {

	    	M2L = 0;
	    	M2R = 1;
	
	 	} else {
	
	    	M2L = 1;
	    	M2R = 0;

		}

		M2EN = 1;
	}

}

int get_switch_status(int switchid)
{
	return (global_input_state & (1 << (switchid - 1)) );
}

void driver_status_update(int driverid)
{
	struct driver * drv = &drivers[driverid];

	if (drv->status == STATUS_RUNNING) {

		if (drv->currentcommand->timeout) {
/*
			if (drv->countdown) {
				drv->countdown--;
			} else {

				if (drv->driverid == M1) M1_ENABLE = 0;
				else if (drv->driverid == M2) M2_ENABLE = 0;
				drv->status = STATUS_STOPPED;


			}
*/
		}

		if (drv->currentcommand->endswitchid) {

			if (get_switch_status(drv->currentcommand->endswitchid)) {

				if (drv->driverid == M1) M1EN = 0;
				else if (drv->driverid == M2) M2EN = 0;
				drv->status = STATUS_STOPPED;
                drv->currentcommand->endfunction(drv->currentcommand);

			}

		}

	}

}

void driver_set(int driverid, int status)
{
	struct driver * drv = &drivers[driverid];

	if (drv->driverid == M1) {
        if (status == STATUS_OFF) {
			M1EN = 0;
		} else {
            if (status == STATUS_ALLUP) {
				M1L = 1;
				M1R = 1;
			} else if (status == STATUS_ALLDOWN) {
				M1L = 0;
				M1R = 0;
			} else if (status == STATUS_1UP) {
				M1L = 1;
				M1R = 0;
			} else if (status == STATUS_2UP) {
				M1L = 0;
				M1R = 1;
			}
			drv->status = status;
			M1EN = 1;
		}
	} else if (drv->driverid == M2) {
        if (status == STATUS_OFF) {
			M2EN = 0;
		} else {
            if (status == STATUS_ALLUP) {
				M2L = 1;
				M2R = 1;
			} else if (status == STATUS_ALLDOWN) {
				M2L = 0;
				M2R = 0;
			} else if (status == STATUS_1UP) {
				M2L = 1;
				M2R = 0;
			} else if (status == STATUS_2UP) {
				M2L = 0;
				M2R = 1;
			}
			drv->status = status;
			M2EN = 1;
		}
	}
}

void driver_enable(int driverid, int status)
{

}

void driver_disable(int driverid, int status)
{

}


/**********************************************************
/ Global
/**********************************************************/

unsigned char timeout50msec = 0;
unsigned char timeout1sec = 0;
unsigned char timeout500msec = 0;

#define TIMEOUT_50MSEC_START 5
#define TIMEOUT_500MSEC_START 50
#define TIMEOUT_1SEC_START 100

unsigned char timer50msec = TIMEOUT_50MSEC_START;
unsigned char timer500msec = TIMEOUT_500MSEC_START;
unsigned char timer1sec = TIMEOUT_1SEC_START;

/**********************************************************
/ Interrupt Low
/**********************************************************/

#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
_asm GOTO low_isr _endasm
}


#pragma code // return to the default code section 
#pragma interruptlow low_isr
void low_isr (void)
{

}


/**********************************************************
/ Interrupt High
/**********************************************************/

#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
_asm GOTO high_isr _endasm
}

int state1 = 0;


#pragma code /* return to the default code section */
#pragma interrupt high_isr
void high_isr (void)
{

	if (INTCONbits.TMR0IF) { // TIMER0: 

		// TMR0H = 0x65; // 20msec
		TMR0H = 0xB3; // 10msec
        TMR0L = 0x00;

		INTCONbits.TMR0IF = 0; // CLEAR INTERRUPT FLAG
		

		#if defined DEBUG_TIMER
			if (state1) {
				state1 = 0;
			} else {
				state1 = 1;
			}
		#endif

		if (!timer50msec)
 		{
			timeout50msec = 1;
			timer50msec = TIMEOUT_50MSEC_START;
		} else timer50msec--;
 
	    if (!timer500msec) {
			timeout500msec = 1;
			timer500msec = TIMEOUT_500MSEC_START;
		} else timer500msec--;

	    if (!timer1sec) {
			timeout1sec = 1;
			timer1sec = TIMEOUT_1SEC_START;
		} else timer1sec--;
		
	}

}


/**********************************************************
/ Queue
/**********************************************************/
#define QUEUE_LENGTH 30

int queue[QUEUE_LENGTH];

unsigned int queue_start = 0;
unsigned int queue_stop = 0;

void push(char c) {

// fprintf(terminal,"push %c qlen: %u qstart: %u qstop: %u \r\n",c,get_queue_length(),queue_start,queue_stop);
 queue[queue_stop] = c;
 if (queue_stop == (QUEUE_LENGTH-1)) {
   if (queue_start>0) {
    queue_stop = 0;
   }
 } else {
  if (queue_stop == (queue_start - 1)) {
  } else {
   queue_stop++;
  }
 } 
// output_high(INT_PIN);
}

unsigned int get_queue_length()
{
  if (queue_start == queue_stop) return 0;
  if (queue_start < queue_stop) return  (queue_stop - queue_start);
  else return  (QUEUE_LENGTH - queue_start) + queue_stop;
}

char pop()
{
 char c = 0;
 if (queue_start != queue_stop) {
  c = queue[queue_start];
//  fprintf(terminal,"pop %c qstart: %u qstop: %u \r\n",c,queue_start,queue_stop);
  if (queue_start == (QUEUE_LENGTH-1)) queue_start = 0;
  else {
    queue_start++;
  }
 }
 return c;
}

//**********************************************************
// 1x6 STATIC KEYBOARD HANDLING 
//**********************************************************
#define MBIT_MAKE  0b00010000
#define MBIT_BREAK 0b00100000

#define MBIT_ID_ROTARYPUSH 0x0

void dokeyboard()
{
  static unsigned int keystate1 = 0xFF;
  static unsigned int keystate;
  unsigned int keydelta;
  unsigned int i;
//printf("A %X B %X\r\n",(PORTA & 0b00110000),(PORTB & 0b11110000));
 // keystate = ((PORTA & 0b00110000) >> 4) | ((PORTB & 0b11110000) >> 2);
  keystate = ((PORTD & INPUT_MASK_PORTD) >> 0) | ((PORTB & INPUT_MASK_PORTB) << 4);
  global_input_state = ~keystate;
 // printf("K %d K1 %d\r\n",keystate,keystate1);
  if (keystate != keystate1) 
  {
    keydelta = (keystate ^ keystate1);
    for (i=0; i<8; i++) {
     if (keydelta & ( 1 << i)) {
        if (keystate & (1 << i)) push (MBIT_ID_ROTARYPUSH | MBIT_BREAK | (i+1) );
       else push (MBIT_ID_ROTARYPUSH | MBIT_MAKE | (i+1) );
     }
    }    
    keystate1 = keystate;
  }
}


/**********************************************************/
// Bar Commands
/**********************************************************/
struct drivercommand bardown;
struct drivercommand barup;

#define STATUS_BARUP 1
#define STATUS_BARDOWN 2
#define STATUS_BARUPPING 3
#define STATUS_BARDOWNING 4

int barstatus = STATUS_BARUPPING;

void driveDown();
void driveUp();

void driveDown()
{
	barstatus = STATUS_BARDOWNING;
	driver_run_command(&bardown);
	driver_set(M2,STATUS_2UP);
}

void driveUp()
{
	barstatus = STATUS_BARUPPING;
	driver_run_command(&barup);
}

void endUp(struct drivercommand *command)
{
//	driver_run_command(&bardown);
	driver_set(M2,STATUS_1UP);
    barstatus = STATUS_BARUP;
}

void endDown(struct drivercommand *command)
{
//	driver_run_command(&barup);
    barstatus = STATUS_BARDOWN;
}


/**********************************************************/
// Handle Digital Input 
/**********************************************************/
void handledigitalinput(unsigned int keycode) {

	int keyid = 0;

    keyid = (keycode & 0b00001111);

	// printf("KEY %d ", keyid);

#ifdef DEBUG_MODE
	if (keycode & MBIT_MAKE) {
	  printf("MAKE  ");
	} else {
	  printf("BREAK ");
	}

	
	if      (keyid == E1) printf ("E1\r\n");
	else if (keyid == E2) printf ("E2\r\n");
	else if (keyid == E3) printf ("E3\r\n");
	else if (keyid == E4) printf ("E4\r\n");
	else if (keyid == E5) printf ("E5\r\n");
	else if (keyid == E6) printf ("E6\r\n");
	else if (keyid == E7) printf ("E7\r\n");
	else if (keyid == E8) printf ("E8\r\n");
#endif 

 if (keycode & MBIT_MAKE) {
	if (keyid == E3) {
		if (barstatus == STATUS_BARUP) {
			driveDown();
		} else if (barstatus == STATUS_BARDOWN) {
			driveUp();
		}
	}
 } else {

 }

}

/**********************************************************
/ MAIN
/**********************************************************/
void main (void)
{
    int debug1 = 0;
	int i;
			

    bardown.driverid = M1;
    bardown.direction = RIGHT;
    bardown.speed = SPEED_HALF;
    bardown.endswitchid = E1;
    bardown.timeout = TIMEOUT_DEFAULT;
    bardown.repeat = NO_REPEAT;
    bardown.endfunction = &endDown;
    
    barup.driverid = M1;
    barup.direction = LEFT;
    barup.speed = SPEED_FULL;
    barup.endswitchid = E2;
    barup.timeout = TIMEOUT_DEFAULT;
	barup.repeat = NO_REPEAT;
    barup.endfunction = &endUp;

	OSCCON = 0x70;  // bit 6-4 IRCF<2:0>: Internal Oscillator Frequency Select bits = 8MHZ

//	ANSEL  = 0b00000000; // All Pins digital
//	ANSELH = 0b00000011; // RC6 and RC7 analog input
	ADCON0 = 0x00; // ADC Off
    ADCON1bits.PCFG0 = 0x1;
	ADCON1bits.PCFG1 = 0x1;
	ADCON1bits.PCFG2 = 0x1;
	ADCON1bits.PCFG3 = 0x1;

    PORTA = 0x0;
    PORTB = 0x0;
    PORTC = 0x0;
    PORTD = 0x0;
    PORTE = 0x0;

	TRISA = INPUT_MASK_PORTA;
	TRISB = INPUT_MASK_PORTB;
	TRISC = INPUT_MASK_PORTC;
	TRISD = INPUT_MASK_PORTD;
	TRISE = INPUT_MASK_PORTE;

//	IOCA  = INPUT_MASK_PORTA; // All input pins will int-on-change
//	IOCB  = INPUT_MASK_PORTB; // All input pins will int-on-change
	INTCON2bits.RBPU = 0x0;  // Port B pull-ups enabled
    PORTEbits.RDPU = 0x1; // Port D pull-ups enabled

//RDPU = 0xFF;
//RBPU = INPUT_MASK_PORTB;
	// WPUA  = INPUT_MASK_PORTA; // All input pins will have pullups enabled
	// WPUB  = INPUT_MASK_PORTB; // All input pins will have pullups enabled

	OpenTimer0( 
		TIMER_INT_ON &
		T0_16BIT &
		T0_SOURCE_INT &
		T0_PS_1_1 );

	INTCONbits.TMR0IE = 1;   // TMR0 Overflow Interrupt Enable bit

	INTCONbits.GIE = 1; // General Interrupt Enable
	INTCONbits.GIEH = 1; // High Priority Interrupt Enable
//    INTCONbits.GIEL = 1; // Low Priority Interrupt Enable
/*
	INTCONbits.RABIE = 1; // RA and RB Port Change Interrupt Enable
	INTCONbits.RABIF = 0; // RA and RB Port Change Interrupt Flag
    INTCON2bits.RABIP = 1; // RABIP: RA and RB Port Change Interrupt Priority (1=High priority)
*/
//    INTCON2bits.RABPU = 0; // Enable Pullups


    #ifdef DEBUG_MODE
	
		OpenUSART( 
			USART_TX_INT_OFF &
			USART_RX_INT_OFF &
			USART_ASYNCH_MODE &
			USART_EIGHT_BIT &
			USART_CONT_RX &
			USART_BRGH_HIGH,
			25 ); // 19.230 baud

		printf((const far rom char*)"\r\nBuzz Fischertechnik Controller v.1.0\r\n");

    #endif

	driver_set(M2,STATUS_2UP);

	driver_run_command(&barup);
//	driver_run_command(&bardown);

	while (1)
	{

		if (queue_start != queue_stop) {
        	handledigitalinput(queue[queue_start]);
	    	pop();
       	}

		if (timeout50msec) {
			timeout50msec = 0;

			dokeyboard();
			for (i = 0; i < NUMBER_OF_DRIVERS; i++)
			{
				driver_status_update(i);
			}

		}

		if (timeout500msec) {
			timeout500msec = 0;
            
            if (debug1) {
				debug1 = 0;
				LED_HEARTBEAT  = 0; 
			} else {
				debug1 = 1;
				LED_HEARTBEAT  = 1; 
			}
		}

		if (timeout1sec) {
			timeout1sec = 0;
		}


	}

}


