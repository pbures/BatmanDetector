#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRIG_DDR    DDRC           // Trigger Port
#define TRIG_PORT   PORTC
#define TRIG_PIN    PINC
#define TRIG_BIT    PC2             // Trigger Pin

#define ECHO_DDR    DDRC            // Echo Port
#define ECHO_PORT   PORTC
#define ECHO_PIN    PINC
#define ECHO_BIT    PC3             // Echo Pin

#define EPWR_DDR	DDRD            // TIP120 power switching mosfet.
#define EPWR_PORT 	PORTD
#define	EPWR_PIN	PIND
#define EPWR_BIT	PD5

#define ECHO_LED_DDR    DDRD	// LED Pin
#define ECHO_LED_PORT	PORTD
#define ECHO_LED_PIN	PIND
#define ECHO_LED_BIT	PD2

/* Define pins for debug LED */
#define RED_DDR     DDRD
#define RED_PORT	PORTD
#define RED_PIN		PIND
#define RED_BIT		PD3

#define BLUE_DDR    DDRD
#define BLUE_PORT	PORTD
#define BLUE_PIN	PIND
#define BLUE_BIT	PD4

#define GRN_DDR     DDRD
#define GRN_PORT	PORTD
#define GRN_PIN		PIND
#define GRN_BIT		PD7

#define SPKR_DDR 	DDRD
#define SPKR_PORT	PORTD
#define SPKR_PIN	PIND
#define SPKR_BIT	PD6

#define SPEED_OF_SOUND  343                                           /* meters in second. */
#define MAX_SONAR_RANGE 10                                            /* meters */
#define SONAR_TIMEOUT_CNTMAX ((F_CPU*MAX_SONAR_RANGE)/SPEED_OF_SOUND) /* range/speed = time until timeout. */
/* time * F_CPU = number of ticks until timeout */

#define DELAY_BETWEEN_DETECTION_MS 100  /* Time between measurements */
#define TIMER_MAX 65535                 /* 65535 for 16 bit timer, Timer1 is 16 bit one */
#define DISTANCE_SIGNAL_MAX_CM 60       /* Above this distance, it is considered as without obstacle. */
#define DISTANCE_SIGNAL_MIN_CM 20       /* Below this distance, the signal pitch is maxed out. */

#define TRIG_ERROR -1
#define ECHO_ERROR -2

#define CYCLES_PER_US (F_CPU/1000000)// instructions per microsecond


#define SET_INPUT_MODE(ddr,bit) ddr &= ~(1<<bit)
#define SET_OUTPUT_MODE(ddr,bit) ddr |= (1<<bit)
#define SET_HIGH(port,bit) port |= (1<<bit)
#define SET_LOW(port,bit) port &= ~(1<<bit)

#define SET_LOW_AND_DELAY(port,bit,delay) port &= ~(1<<bit);_delay_us(delay)
#define SET_HIGH_AND_DELAY(port,bit,delay) port |= (1<<bit);_delay_us(delay)

#define TICKS_TO_CM(ticks) (ticks/(((10000*2)/SPEED_OF_SOUND) * CYCLES_PER_US))

volatile uint32_t timerOverflowCnt = 0;
volatile uint32_t trigCnt = 0;

void initSonar() {

	SET_INPUT_MODE(ECHO_DDR, ECHO_BIT);

	SET_OUTPUT_MODE(TRIG_DDR, TRIG_BIT);
	SET_OUTPUT_MODE(ECHO_LED_DDR, ECHO_LED_BIT);
	SET_OUTPUT_MODE(RED_DDR, ECHO_LED_BIT);
	SET_OUTPUT_MODE(GRN_DDR, GRN_BIT);
	SET_OUTPUT_MODE(EPWR_DDR, EPWR_BIT);

	SET_HIGH(EPWR_PORT, EPWR_BIT);
}

void triggerSonar() {
	SET_LOW_AND_DELAY(TRIG_PORT, TRIG_BIT,1); /* First clear the pin */
	SET_HIGH_AND_DELAY(TRIG_PORT, TRIG_BIT,12); /* Send high impulse for sth over 10 us */
	SET_LOW_AND_DELAY(TRIG_PORT, TRIG_BIT,1); /* Set back to low */
}

/* The cheap sonar sensors get stuck when no echo is received. If timeout is reached, reset the device
 * by powering down for 10ms.
 */
void rebootSonar() {
	SET_LOW_AND_DELAY(EPWR_PORT, EPWR_BIT,200);
	SET_HIGH(EPWR_PORT, EPWR_BIT);
}

/* Count number of timer overflows, this tells us how many cycles passed */
ISR(TIMER1_OVF_vect) {
	timerOverflowCnt++;
	TCNT1 = 0;
}

int16_t readSonar() {

//	init_sonar();                       // Setup pins and ports
	triggerSonar();                    // send a 10us high pulse

	while (!(ECHO_PIN & (1 << ECHO_BIT))) {
		trigCnt++;
		uint32_t max_response_time = SONAR_TIMEOUT_CNTMAX;
		if (trigCnt > max_response_time) {
			return TRIG_ERROR; /* Echo signal was never sent */
		}
	}

	trigCnt = 0;

	/* Reset the Timer1, start without prescaler and enable overflow interrupt */
	TCNT1 = 0;
	TCCR1B |= (1 << CS10);
	TIMSK1 |= (1 << TOIE1);
	timerOverflowCnt = 0;
	sei();

	/* Wait for signal on echo pin */
	while ((ECHO_PIN & (1 << ECHO_BIT))) {
		if (((timerOverflowCnt * TIMER_MAX) + TCNT1) > SONAR_TIMEOUT_CNTMAX) {
			return ECHO_ERROR;          /* We reached timeout, echo is still up */
		}
	};

	TCCR1B = 0x00;                      // stop 16 bit timer with no prescaler
	cli();

	return TICKS_TO_CM((timerOverflowCnt * TIMER_MAX) + TCNT1);
}

void speakerOnByDistance(uint16_t distanceCm) {
	/* Distance is 0 .. 150, need to map to 33 .. 255*/
	/* 30 cm or lower is the highest pitch anyway */

	uint16_t distance =
			(distanceCm < DISTANCE_SIGNAL_MIN_CM) ?
					0 : distanceCm - DISTANCE_SIGNAL_MIN_CM;
	uint16_t wavelength = 33
			+ (distance * 222
					/ (DISTANCE_SIGNAL_MAX_CM - DISTANCE_SIGNAL_MIN_CM));

	OCR0A = (wavelength > 255) ? 255 : wavelength;
	SPKR_DDR |= (1 << SPKR_BIT);
}

void speakerOff() {
	SPKR_DDR &= ~(1 << SPKR_BIT);
}

void initSpeaker() {
	speakerOff();
	TCCR0A |= (1 << WGM01);  /* CTC Mode */
	TCCR0A |= (1 << COM0A0); /* Toggle PD6 on cycle through */
	TCCR0B |= ((1 << CS00) | (1 << CS01)); /* Prescaler to 64 */
}

void debugLedsBlink() {
	SET_HIGH_AND_DELAY(RED_PORT, RED_BIT, 500);
	SET_LOW(RED_PORT, RED_BIT);
	SET_HIGH_AND_DELAY(BLUE_PORT, BLUE_BIT, 500);
	SET_LOW(BLUE_PORT, BLUE_BIT);
	SET_HIGH_AND_DELAY(GRN_PORT, GRN_BIT, 500);
	SET_LOW(GRN_PORT, GRN_BIT);
}

int main() {
	int16_t distanceCm = 0;
	initSonar();
	initSpeaker();
	debugLedsBlink();

	while (1) {
		distanceCm = readSonar();
		if (distanceCm == TRIG_ERROR) {
			speakerOff();
			SET_LOW(BLUE_PORT, BLUE_BIT);
			SET_LOW(RED_PORT, RED_BIT);
			SET_HIGH(GRN_PORT, GRN_BIT);
			_delay_ms(DELAY_BETWEEN_DETECTION_MS);
		} else if (distanceCm == ECHO_ERROR) {
			speakerOff();
			SET_LOW(ECHO_LED_PORT, ECHO_LED_BIT);
			SET_LOW(BLUE_PORT, BLUE_BIT);
			SET_LOW(GRN_PORT, GRN_BIT);
			SET_HIGH(RED_PORT, RED_BIT);

			rebootSonar();
		} else {
			SET_HIGH(BLUE_PORT, BLUE_BIT);
			SET_LOW(RED_PORT, RED_BIT);
			SET_LOW(GRN_PORT, GRN_BIT);
			if (distanceCm < DISTANCE_SIGNAL_MAX_CM) {
				speakerOnByDistance(distanceCm);
				SET_HIGH(ECHO_LED_PORT, ECHO_LED_BIT);
			} else {
				speakerOff();
				SET_LOW(ECHO_LED_PORT, ECHO_LED_BIT);
			}
			_delay_ms(DELAY_BETWEEN_DETECTION_MS);
		}
	}
	return 0;
}

