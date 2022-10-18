/*  same as v 0.2 other than color detection sequence inside timer1_compa_vect*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adc.h"
#include "multimeter_click.h"
#include "spi.h"
#include "timers.h"
#include "uart.h"

#define r_channel   0x00
#define g_channel   0x03
#define b_channel   0x01
#define PS          0x5350                                          // send 'P' and 'S' in message_packet_t to indicate pass (LOWER BYTE SENT FIRST)
#define FL          0x4C46                                          // send 'F' and 'L' in message_packet_t to indicate fail (LOWER BYTE SENT FIRST)

/*  RT67 controls connection between multimeter/relay_16 GND and arduino GND
    Because relay_16 IN pins are pulled to ground when power is removed from arduino
    relay_16 GND needs to be suspended/connected by software
    multimeter GND needs to be suspended/connected when UART is used/measuring
    both GNDs are connected to J2.3
    one side of RT67 coil is connected to GND
    the other side is connected PK1 (A9)
 */
#define RT67_ON     DDRK|=(1<<PK1);PORTK|=(1<<PK1);
#define RT67_OFF    PORTK&=~(1<<PK1);
#define RT68_ON     PORTB|=(1<<PB7);
#define RT68_OFF    PORTB&=~(1<<PB7);

/* or use array of structs if automatic iterations are involved */
struct relay_module_t {
    uint8_t pin;
    char* descriptor;
};

uint8_t buffer[100] = {0};

volatile uint8_t color_value[3];

volatile uint16_t i = 0;
volatile uint8_t s = 0;
// volatile uint8_t t_0 = 0;
volatile uint8_t t_1 = 0;
volatile uint8_t t_2 = 0;
volatile uint8_t pulse[100][2];
volatile bool pwm_pulse_ready = false;
volatile bool timer3_flag = false;

volatile uint8_t adc_temp;
volatile uint32_t rms_sum;
volatile uint16_t rms_num;
volatile uint32_t true_rms_sum;
volatile uint16_t true_rms_num;
volatile uint16_t color_array_index = 0;
volatile uint8_t color_channel = 0;
// volatile uint8_t counts_r = 0;
// volatile uint8_t counts_g = 0;
// volatile uint8_t counts_b = 0;
// volatile uint8_t counts_r_old = 0;
// volatile uint8_t counts_g_old = 0;
// volatile uint8_t counts_b_old = 0;
// volatile uint32_t buzzer_pwm_pulse = 0;
volatile uint32_t tick = 0;
volatile uint8_t beeps = 0;
volatile uint8_t blinks = 0;
volatile uint16_t color_pulse_count = 0;
volatile uint8_t message;
volatile bool color_data_ready = false;
volatile bool message_ready = false;
volatile bool beep_flag = false;
volatile bool adc_measure_start = false;

void init_system();
void enable_beep();
void disable_beep();
void enable_blink();
void disable_blink();
void switch_color_channel(uint8_t color);
void test_test_points();
void test_1();
void test_2();
void test_3();
void test_4();
void test_5();
void test_6();
void test_7();
void test_8();
void test_9();
void test_10();
void test_11();
void test_12();
void _abort();
void end();

static void relay_call(uint8_t pin);

/* for new algorithm, 6 color patterns including off(all zeor) */
uint8_t led_sequence[][3] = {
    {0,0,0},
    {192,9,19},
    {194,46,121},
    {192,9,19},
    {192,9,19},
    {2,37,103}
};
/* define all possible states */

struct relay_module_t relay_16[] = {
    {PC0, "SW_L1_IN"},
    {PC1, "SW_L2_IN"},
    {PC2, "SW_GND_IN"},
    {PC3, "SW_STAK_L1"},
    {PC4, "SW_STAK_L2"},
    {PC5, "SW_L1_LL"},
    {PK7, "SW_L1_HL"},
    {PK6, "SW_L2_LL"},
    {PK5, "SW_L2_HL"},
    {PK4, "SW_STATE_B"},
    {PK3, "SW_STATE_A"},
    {PK2, "SW_DIODE"},
    {PA4, "TP29"},
    {PA5, "TP30"},
    {PA6, "TP4"},
    {PA7, "TP5"}
};

/* use the same struct for onboard relays */
struct relay_module_t test_points[] = {
    {0b01110000, "TP12"},        
    {0b01111000, "TP19"},       
    {0b01110100, "TP21"},      
    {0b01111100, "TP22"},      
    {0b01110010, "TP25"},      
    {0b01111010, "TP26"},      
    {0b01110110, "TP35"},        
    {0b01111110, "TP38"},      
    {0b01110001, "NONE"},
    {0b01111001, "NONE"},
    {0b01110101, "NONE"},
    {0b01111101, "NONE"},
    {0b01110011, "NONE"},
    {0b01111011, "NONE"},
    {0b01110111, "NONE"},
    {0b01111111, "NONE"},
    {0b10110000, "JLINK"}                                           // RT17 ON TB1
};

enum state_t {
    _ENTRY,
    _TEST_1,
    _TEST_2,
    _TEST_3,
    _TEST_4,
    _TEST_5,
    _TEST_6,
    _TEST_7,
    _TEST_8,
    _TEST_9,
    _TEST_10,
    _TEST_11,
    _TEST_12,
    _ABORT,
    _END
};

/* define all events */
enum event_t {
    test_start      =   0x30,
    test_abort      =   0x31,
    test_complete   =   0x32,
    test_next       =   0x33,
    test_debug      =   0x2A                        
};

/* define a row in state transition matrix */
struct state_transit_row_t {
    enum state_t current_state;
    enum event_t event;
    enum state_t next_state;
};

/* define a matrix of state transistion */
static struct state_transit_row_t state_transition_matrix[] = {
    {_ENTRY, test_start, _TEST_1},
    {_TEST_1, test_next, _TEST_2},
    {_TEST_2, test_next, _TEST_3},
    {_TEST_3, test_next, _TEST_4},
    {_TEST_4, test_next, _TEST_5},
    {_TEST_5, test_next, _TEST_6},
    {_TEST_6, test_next, _TEST_7},
    {_TEST_7, test_next, _TEST_8},
    {_TEST_8, test_next, _TEST_9},
    {_TEST_9, test_next, _TEST_10},
    {_TEST_10, test_next, _TEST_11},
    {_TEST_11, test_next, _ABORT},
    {_TEST_1, test_abort, _ABORT},
    {_TEST_2, test_abort, _ABORT},
    {_TEST_3, test_abort, _ABORT},
    {_TEST_4, test_abort, _ABORT},
    {_TEST_5, test_abort, _ABORT},
    {_TEST_6, test_abort, _ABORT},
    {_TEST_7, test_abort, _ABORT},
    {_TEST_8, test_abort, _ABORT},
    {_TEST_9, test_abort, _ABORT},
    {_TEST_10, test_abort, _ABORT},
    {_TEST_11, test_abort, _ABORT},
    {_TEST_12, test_abort, _ABORT},
    {_ENTRY, test_abort, _ABORT}
};

/* define a row in state function matrix */
struct state_function_row_t {
    const char* name;
    void (*func)(void);
};

/* define a matrix of state functions */
static struct state_function_row_t state_function_matrix[] = {
    {"POWER ON", test_1},
    {"GROUND", test_2},
    {"PILOT STATE A", test_3},
    {"PILOT STATE B", test_4},
    {"DIODE", test_5},
    {"OVER CURRENT", test_6},
    {"GFCI_L1_LOW_LEAKAGE", test_7},
    {"GFCI_L1_HIGH_LEAKAGE", test_8},
    {"GFCI_L2_LOW_LEAKAGE", test_9},
    {"GFCI_L2_HIGH_LEAKAGE", test_10},
    {"STUCK RELAY", test_11},
    {"END", _abort},
    {"STATE ABORT 1", _abort},
    {"STATE ABORT 2", _abort},
    {"STATE ABORT 3", _abort},
    {"STATE ABORT 4", _abort},
    {"STATE ABORT 5", _abort},
    {"STATE ABORT 6", _abort},
    {"STATE ABORT 7", _abort},
    {"STATE ABORT 8", _abort},
    {"STATE ABORT 9", _abort},
    {"STATE ABORT 10", _abort},
    {"STATE ABORT 11", _abort},
    {"STATE ABORT 12", _abort},
    {"THE END", _abort}
};

struct state_machine_t {
    enum state_t current_state;
};

void transition_look_up(struct state_machine_t* state_machine, enum event_t event) {
    for (uint8_t i=0;i<sizeof(state_transition_matrix)/sizeof(state_transition_matrix[0]);i++) {
        if (state_transition_matrix[i].current_state == state_machine->current_state) {
            if (state_transition_matrix[i].event == event) {
                // uart0_puts(state_function_matrix[i].name);
                // uart0_puts("\r\n");

                (state_function_matrix[i].func)();
                state_machine->current_state = state_transition_matrix[i].next_state;
                break;
            }
        }
    }
}

int main() {
    init_system();

    /* initialize state machine */
    struct state_machine_t state_machine; 
    state_machine.current_state = _ENTRY;

    enum event_t x;
    
    for (;;) {
        if (message_ready) {
            x = message;
            message_ready = false;

            PORTK |= (1 << PK0);                                    // activity light on
            transition_look_up(&state_machine, x);
            PORTK &= ~(1 <<PK0);                                    // activity light off
            sleep_mode();                                           // put MCU to sleep after function returns
        }

        /* if look-up is placed here, it gets executed on every timer0 interrupt */
        // transition_look_up(&state_machine, x);
        // PORTB ^= (1 << PB7);                                        // toggling LED provides visual indication
        // sleep_mode();                                               // put MCU to sleep after function returns
    }
}

void init_system() {
    DDRA |= (1 << PA4) | (1 << PA5) | (1 << PA6) | (1 << PA7);      // 13/14/15/16 on RELAY_16
    PORTA |= (1 << PA4) | (1 << PA5) | (1 << PA6) | (1 << PA7);     // put in default OFF

    DDRB |= (1 << PB6) | (1 << PB7);                                // RT68 (MULTIMETER) COIL

    DDRE |= (1 << PE3) | (1 << PE4) | (1 << PE5);                   // D5/2/3 selects HC154
    PORTE |= (1 << PE3) | (1 << PE4) | (1 << PE5);                  

    DDRG |= (1 << PG5);                                             // D4
    PORTG |= (1 << PG5);

    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5) | (1 << PH6);      // A0/1/2/3 on HC154
    PORTH |= (1 << PH3) | (1 << PH4) | (1 << PH5) | (1 << PH6);

    DDRK |= (1 << PK0);

    DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);      // s0/s1/s2/s3 on tcs3200
    PORTA |= (1 << PA0);                                            // frequency scaling 20%

    init_adc();
    uart0_init();
    uart2_init();
    uart3_init();
    init_timer0();
    init_timer1();
    multimeter_init();

    set_sleep_mode(0);                                              // in Idle Mode, UART still runs

    sei();                                                          // enable global interrupt

    uint8_t system_status[2] = {'F', 'F'};

    /* PUS status check */
    uint8_t i = 0;
    uint32_t t_0 = tick;

    uart3_puts(":01w12=0,\r\n");

    for (;;) {
        if (UCSR3A & (1 << RXC3)) {
            char u = UDR3;
            if (i == 3 && u == 'o') {                              // PSU returns ":01ok" if connected
                system_status[0] = 'P';
            }
            i++;
        }
        if (tick - t_0 > 100) {                                     // if PSU is offline, make sure this process times out
            break;
        }
    }
    uart3_puts(":01w20=0,0,\r\n");

    /* L1_IN & L2_IN status */
    // TODO

    uart0_transmit(system_status[0]);                               // PSU status
    uart0_transmit(system_status[1]);                               // L1_IN & L2_IN status        
}

void enable_beep() {
    EIFR |= (1 << INTF2);                                           // INT2 will false trigger at first, clear flag by writing 1 to EIFR
    cli();
    EICRA |= (1 << ISC21) | (1 << ISC20);                           // set bits to choose rising edge triggering
    EIMSK |= (1 << INT2);                                           // enable interrupt
    sei();
}

void disable_beep() {
    EIMSK &= ~(1 << INT2);
}

void enable_blink() {
    EICRA |= (1 << ISC31) | (1 << ISC30);                           // rising edge on INT3 generates an interrupt reques
    EIMSK |= (1 << INT3);                                           // enable INT3(PD3/D18)
}

void disable_blink() {
    EIMSK &= ~(1 << INT3);
}

void test_1() {
    /* if JLINK probe is connected but not powered, relay-16 should not be opened, otherwise it causes wierd behaviour */
    // RT68_ON
    // _delay_ms(500);
    // relay_call(test_points[16].pin);
    // _delay_ms(500);
    // int volt_jlink_tref = multimeter_read_voltage();
    // // sprintf(buffer, "%u\r\n", volt_jlink_tref);
    // // uart0_puts(buffer);
    // if (volt_jlink_tref > 2400 && volt_jlink_tref < 2500) {         // JLINK powered
    //     PORTA &= ~(1 << relay_16[15].pin);                          // T_REF on JLINK is on NO(16), close this relay cuts this connection
    //     _delay_ms(1000);
    // }
    // RT68_OFF

    DDRC |= (1 << relay_16[2].pin);
    _delay_ms(500);
    DDRC |= (1 << relay_16[0].pin) | (1 << relay_16[1].pin);

    start_adc();
    select_adc3();
    enable_blink();
    start_timer1();

    uint32_t t_0 = tick;
    uint16_t led_time = 0;
    uint8_t j = 0;
    uint8_t blink_fail = 0;
    uint8_t test_result[7] = {0};

    for (;;) {
        if (color_data_ready) {                                                 // whenever a complete set of {r,g,b} is sampled
            // if ((led_time > 75) && (led_time < 142)) {                          // between 810 ~ 1440 ms
            //     for (uint8_t i=0;i<3;i++) {                                     // iterate r/g/b    
            //         if (abs(color_value[i] - led_sequence[j][i]) > 50) {        // compare r/g/b sampled to r/g/b stored in a array (sometime error can get up to 35, 50 is a pretty safe margin)
            //             blink_fail += 1;                                        // if each set matches to stored value, correct led flash sequence must have been observed
            //             // sprintf(buffer, "%u\r\n", led_time);
            //             // uart0_puts(buffer);
            //         }
            //     }
            //     j++;                                                            // increment this index only during this compare time window
            // }

            /* a smimpler algorithm, only look at number of transitions, not led on time */
            if (j < 6) {
                for (uint8_t i=0;i<3;i++) {
                    if (abs(color_value[i] - led_sequence[j][i]) > 50) {
                        j++;
                        // uart0_transmit('^');
                    }
                }
            }

            led_time++;                                                         // led_time starts from begining of sampling, gets incremented in each sampling period
            color_data_ready = false;

            /* rms calculation */
            if (rms_sum > 115000) {                                             // collect all samples above mid-value, this also indicates L1/L2 output comes on
                // ac_out_on = true;
                true_rms_sum += rms_sum;
                true_rms_num += rms_num;
            }
            rms_sum = 0;
            rms_num = 0;
        }
        
        if (tick - t_0 > 2000) {                                                // the blink test should time out after 2 seconds
            // sprintf(buffer, "%u\r\n", color_array_index);
            // uart0_puts(buffer);
            break;
        }
    }

    disable_adc();
    stop_timer1();
    disable_blink();

    // sprintf(buffer, "%u\r\n", j);
    // uart0_puts(buffer);

    /* TP voltage measurement */
    RT68_ON                                                         // connect multimeter v- to DUT GND
    _delay_ms(50);

    relay_call(test_points[6].pin);
    _delay_ms(1000);
    int volt_tp35 = multimeter_read_voltage();

    // sprintf(buffer, "TP35: %d\r\n", volt_tp35);
    // uart0_puts(buffer);

    relay_call(test_points[2].pin);
    _delay_ms(1000);
    int volt_tp21 = multimeter_read_voltage();

    // sprintf(buffer, "TP21: %d\r\n", volt_tp21);
    // uart0_puts(buffer);

    relay_call(test_points[3].pin);
    _delay_ms(1000);
    int volt_tp22 = multimeter_read_voltage();
    // for(;;);
    // sprintf(buffer, "TP22: %d\r\n", volt_tp22);
    // uart0_puts(buffer);

    relay_call(test_points[4].pin);
    _delay_ms(1000);
    int volt_tp25 = multimeter_read_voltage();

    // sprintf(buffer, "TP25: %d\r\n", volt_tp25);
    // uart0_puts(buffer);

    relay_call(test_points[5].pin);
    _delay_ms(1000);
    int volt_tp26 = multimeter_read_voltage();

    // sprintf(buffer, "TP26: %d\r\n", volt_tp26);
    // uart0_puts(buffer);

    RT68_OFF

    /* blink DEBUG */
    //

    /* L1_OUT & L2_OUT DEBUG */
    uint16_t x = sqrt(true_rms_sum/true_rms_num);
    //

    /* C code no longer make pass/fail test on TP, values are passed to Python code only */
    struct message_packet_t
    {
        char blink;
        char output;
        uint16_t TP35;
        uint16_t TP21;
        uint16_t TP22;
        uint16_t TP25;
        uint16_t TP26;
    };
    
    struct message_packet_t message_packet;

    (j == 5) ? (message_packet.blink = 'P') : (message_packet.blink = 'F');
    // blink_fail ? (message_packet.blink = 'F') : (message_packet.blink = 'P');
    (x > 44 || x < 46) ? (message_packet.output = 'P') : (message_packet.output = 'F');
    message_packet.TP35 = volt_tp35;
    message_packet.TP21 = volt_tp21;
    message_packet.TP22 = volt_tp22;
    message_packet.TP25 = volt_tp25;
    message_packet.TP26 = volt_tp26;

    char* p = (char*)&message_packet;

    for (uint8_t i=0;i<sizeof(message_packet);i++) {
        uart0_transmit(*p++);
    }
}

void test_2() {
    PORTC |= (1 << relay_16[2].pin);

    uint32_t t_0 = tick;

    /* do not call enable_blink(), won't work, must clear flag first */
    EIFR |= (1 << INTF2);                                           // INT2 will false trigger at first, clear flag by writing 1 to EIFR
    cli();
    EICRA |= (1 << ISC21) | (1 << ISC20);                           // set bits to choose rising edge triggering
    EIMSK |= (1 << INT2);                                           // enable interrupt
    sei();

    enable_blink();
    start_timer1();
    RT68_ON

    uint8_t color_temp[3] = {0};
    uint16_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint16_t num_beep = 0;
    uint8_t test_result[3] = {0};
    bool volt_crossed = false;
    bool volt_print = true;
    bool falling_edge = false;
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {         // 90 is not sensitive enough
                // sprintf(buffer, "blink: %u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* beep detection */
        if (beep_flag) {
            switch (edge_direction)
            {
            case 0:                                                 // rising edge
                t_1 = tick;
                edge_direction = 1;
                break;
            
            case 1:                                                 // falling edge
                if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: derived empirically
                    num_beep++;
                }
                edge_direction = 0;
                break;

            default:
                break;
            }
            
            beep_flag = false;
        }


        if (tick - t_0 > 9000) {                                    // a string of beep spaced 2900ms apart
            /* timeout */
            break;
        }
    }

    relay_call(test_points[7].pin);
    _delay_ms(500);
    int volt_tp38 = multimeter_read_voltage();
    // sprintf(buffer,"TP38: %d\r\n", volt_tp38);                        // DEBUG
    // uart0_puts(buffer);

    RT68_OFF
    stop_timer1();
    disable_blink();
    disable_beep();

    PORTC |= (1 << relay_16[0].pin) | (1 << relay_16[1].pin);
    _delay_ms(1500);                                                // after relay opens, it take ~ 1.3 s for the board to emit beep, shoulde wait for the until the next test

    (led_blink == 3) ? (test_result[0] = 0) : (test_result[0] = 1);
    (num_beep == 3) ? (test_result[1] = 0) : (test_result[1] = 1);
    (volt_tp38 > 2400 && volt_tp38 < 2500) ? (test_result[2] = 0) : (test_result[2] = 1);
    
    // for (uint8_t i=0;i<3;i++) {
    //     if (test_result[i]) {
    //         uart0_transmit(0x31 + i);
    //         uart0_transmit('F');
    //         return;
    //     }
    // }
    // uart0_transmit(0x31 + 2);
    // uart0_transmit('P');

    // return;
    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, num_beep);
    // uart0_puts(buffer);

    struct message_packet_t
    {
        uint16_t blink;
        uint16_t beep;
        uint16_t TP38;
    };
    
    struct message_packet_t message_packet;

    message_packet.blink = led_blink;
    message_packet.beep = num_beep;
    message_packet.TP38 = volt_tp38;

    char* p = (char*)&message_packet;

    for (uint8_t i=0;i<sizeof(message_packet);i++) {
        uart0_transmit(*p++);
    }
}

void test_3() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRK |= (1 << relay_16[10].pin);

    RT67_ON

    uint32_t t_0 = tick;
    // PORTF |= (1<<PF2);                                              // start of test (pwm on TP1 comes on in 777 ms)

    enable_blink();
    start_timer1();
    start_timer2();
    rms_sum = 0;
    rms_num = 0;
    start_adc();
    select_adc3();

    /* TP1 PWM on INTO */
    EICRA |= (1 << ISC01) | (1 << ISC00);                           // rising edge on INT0 generates an interrupt reques
    EIMSK |= (1 << INT0);                                           // enable INT0(PD0/D21)

    uint8_t color_temp[3] = {0}; 
    uint8_t led_blink = 0;
    uint8_t test_result[3] = {0};
    bool rms_read = true;
    bool rms_pass = false;
    bool period_pass = false;
    bool duty_cycle_pass = false;

    /*  because DUT starts from OFF, there's red/teal/read/blue blinking sequece
        there might be some red component still exist at this point
    */
   
    for (;;) {
        if (color_data_ready) {
            // sprintf(buffer, "{%d\t%d\t%d}\r\n", color_value[0]-color_temp[0], color_value[1]-color_temp[1], color_value[2]-color_temp[2]);
            // sprintf(buffer, "{%u,%u,%u}\t(%u,%u,%u)\r\n", color_value[0], color_value[1], color_value[2], color_temp[0], color_temp[1], color_temp[2]);
            // sprintf(buffer, "{%u\t%u\t%u}\r\n", color_value[0], color_value[1], color_value[2]);
            // uart0_puts(buffer);
            /*  differences of blue channel values exceed 50 when LED goes from on/off or off/on, i.e. edge
                thresholds of 10/20/50 work even though color_temp[3] gets assigned seemingly random values
                width of edges are either 400 ms or 600 ms, don't know which one is on, which one is off
                I'll leave it like this for now
            */ 
            if (abs(color_value[2] - color_temp[2]) > 50) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                led_blink++;
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* TP1 PWM comes on 691 ms after SW_STATE_A closes */
        if (pwm_pulse_ready) {
            uint16_t period_sum = 0;
            uint16_t off_width_sum = 0;

            for (uint16_t j=0;j<100;j++) {
                // sprintf(buffer, "%u\t%u\r\n", pulse[j][0], pulse[j][1]);
                // uart0_puts(buffer);
                period_sum += pulse[j][1];
                off_width_sum += pulse[j][0];
            }

            /*  (timer2 clocks at f/64, 250 counts indicate 1kHz) */
            (period_sum/(sizeof(pulse)/sizeof(pulse[0])) > 247 && period_sum/(sizeof(pulse)/sizeof(pulse[0])) < 262) ? (test_result[1] = 0) : (test_result[1] = 1);
                
            (off_width_sum/(sizeof(pulse)/sizeof(pulse[0])) > 79 && off_width_sum/(sizeof(pulse)/sizeof(pulse[0])) < 89) ? (test_result[1] = 0) : (test_result[1] = 1);

            // sprintf(buffer, "%u\t%u\r\n", period_sum/(sizeof(pulse)/sizeof(pulse[0])), off_width_sum/(sizeof(pulse)/sizeof(pulse[0])));
            // uart0_puts(buffer);

            // sprintf(buffer, "pwm pulse ready: %u\r\n", tick-t_0);
            // uart0_puts(buffer);

            pwm_pulse_ready = false;
        }

        /*  L1_OUT & L2_OUT comes on ~1s before SW_STATE_A closes for ~230ms
            but we only care about what happens afterwards
        */
        if (tick - t_0 > 500 && rms_read) {                         // 
            (rms_sum/rms_num < 10) ? (test_result[2] = 0) : (test_result[2] = 1);
            rms_read = false;
        }

        if (tick - t_0 > 5000) {
            /* timeout */
            break;
        }
    }

    disable_adc();
    disable_blink();
    stop_timer1();
    stop_timer2();
    EIMSK &= ~(1 << INT0);                                          // disable PWM detectioin on TP1
// sprintf(buffer, "blink: %u\r\n", led_blink);
//     uart0_puts(buffer);
    (led_blink > 5) ? (test_result[0] = 0) : (test_result[0] = 1);      // if more than 5 edges detected, it pretty much means blinking
    
    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;
}

void test_4() {
    DDRK |= (1 << relay_16[9].pin);

    uint32_t t_0 = tick;
    // PORTF |= (1<<PF2);                                              // start of test (pwm on TP1 comes on in 777 ms)

    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection
    start_timer2();                                                 // need timer2 for TP1 PWM detection
    start_adc();
    select_adc3();

    /* TP1 PWM on INTO */
    EICRA |= (1 << ISC01) | (1 << ISC00);                           // rising edge on INT0 generates an interrupt reques
    EIMSK |= (1 << INT0);                                           // enable INT0(PD0/D21)

    uint8_t color_temp[3] = {0}; 
    uint8_t led_blink = 0;
    uint8_t rms = 0;
    uint8_t test_result[3] = {0};
    bool rms_read = true;
    bool rms_reset = true;
    bool period_pass = false;
    bool duty_cycle_pass = false;

    /*  because DUT starts from OFF, there's red/teal/read/blue blinking sequece
        there might be some red component still exist at this point
    */

    for (;;) {
        if (color_data_ready) {
            // sprintf(buffer, "{%d\t%d\t%d}\r\n", color_value[0]-color_temp[0], color_value[1]-color_temp[1], color_value[2]-color_temp[2]);
            // sprintf(buffer, "{%u,%u,%u}\t(%u,%u,%u)\r\n", color_value[0], color_value[1], color_value[2], color_temp[0], color_temp[1], color_temp[2]);
            // sprintf(buffer, "{%u\t%u\t%u}\r\n", color_value[0], color_value[1], color_value[2]);
            // uart0_puts(buffer);
            /*  differences of blue channel values exceed 50 when LED goes from on/off or off/on, i.e. edge
                thresholds of 10/20/50 work even though color_temp[3] gets assigned seemingly random values
                width of edges are either 400 ms or 600 ms, don't know which one is on, which one is off
                I'll leave it like this for now
            */ 
            if (abs(color_value[1] - color_temp[1]) > 50) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                led_blink++;
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* TP1 PWM comes on 691 ms after SW_STATE_A closes */
        if (pwm_pulse_ready) {
            uint16_t period_sum = 0;
            uint16_t off_width_sum = 0;

            for (uint16_t j=0;j<100;j++) {
                // sprintf(buffer, "%u\t%u\r\n", pulse[j][0], pulse[j][1]);
                // uart0_puts(buffer);
                period_sum += pulse[j][1];
                off_width_sum += pulse[j][0];
            }

            (period_sum/(sizeof(pulse)/sizeof(pulse[0])) > 247 && period_sum/(sizeof(pulse)/sizeof(pulse[0])) < 262) ? (test_result[1] = 0) : (test_result[1] = 1);
                
            (off_width_sum/(sizeof(pulse)/sizeof(pulse[0])) > 79 && off_width_sum/(sizeof(pulse)/sizeof(pulse[0])) < 89) ? (test_result[1] = 0) : (test_result[1] = 1);

            // sprintf(buffer, "%u\t%u\r\n", period_sum/(sizeof(pulse)/sizeof(pulse[0])), off_width_sum/(sizeof(pulse)/sizeof(pulse[0])));
            // uart0_puts(buffer);

            // sprintf(buffer, "pwm pulse ready: %u\r\n", tick-t_0);
            // uart0_puts(buffer);

            pwm_pulse_ready = false;
        }

        /*  L1_OUT & L2_OUT comes on ~1s after SW_STATE_B closes */
        if (tick - t_0 > 1500 && rms_reset) {                       // adc start running at t_0, need to reset these two parameters when measuring actually starts 
            // sprintf(buffer, "rms sum: %lu\trms num: %u\r\n", rms_sum, rms_num);
            // uart0_puts(buffer);
            // if (rms_sum/rms_num < 10) {
            //     uart0_puts("L1_OUT & L2_OUT OFF: PASS\r\n");
            // }
            // rms_read = false;
            rms_sum = 0;
            rms_num = 0;
            rms_reset = false;
        }
        if (tick - t_0 > 2000 && rms_read) {                        // measuring ends at t = 2000 ms
            // sprintf(buffer, "rms sum: %lu\trms num: %u\r\n", rms_sum, rms_num);
            // uart0_puts(buffer);
            rms = sqrt(rms_sum/rms_num);
            (rms > 40) ? (test_result[2] = 0) : (test_result[2] = 1);
            // sprintf(buffer, "rms: %u\r\n", rms);
            // uart0_puts(buffer);
            rms_read = false;
        }

        /* timeout */
        if (tick - t_0 > 5000) {
            break;
        }
    }

    disable_adc();
    stop_timer1();
    disable_blink();
    stop_timer2();
    EIMSK &= ~(1 << INT0);                                          // disable PWM detectioin on TP1
    RT67_OFF
// sprintf(buffer, "blink: %u\r\n", led_blink);
//     uart0_puts(buffer);
    (led_blink > 5) ? (test_result[0] = 0) : (test_result[0] = 1);      // if more than 5 edges detected, it pretty much means blinking
    
    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;
}

void test_5() {
    DDRK |= (1 << relay_16[11].pin);
DDRF|=(1<<PF5);PORTF|=(1<<PF5);
    uint32_t t_0 = tick;                                            // record start time

    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection
    start_adc();
    select_adc3();

    uint8_t a = 0;
    uint16_t rms = 0;
    uint8_t rms_count = 0;
    uint8_t color_temp[3] = {0}; 
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint8_t num_beep = 0;
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t test_result[3] = {0};
    bool red_on = true;
    bool rms_read = true;
    bool rms_reset = true;
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        /* blink detection */
        if (color_data_ready) {
            // sprintf(buffer, "{%d\t%d\t%d}\r\n", color_value[0]-color_temp[0], color_value[1]-color_temp[1], color_value[2]-color_temp[2]);
            // sprintf(buffer, "{%u,%u,%u}\t(%u,%u,%u)\r\n", color_value[0], color_value[1], color_value[2], color_temp[0], color_temp[1], color_temp[2]);
            // sprintf(buffer, "{%u\t%u\t%u}\r\n", color_value[0], color_value[1], color_value[2]);
            // uart0_puts(buffer);
            /*  differences of red channel values should exceed ~180 when LED goes from on/off or off/on, i.e. edge
                BUT... at transition points, red values can be 40/50 or 140/160, and SPREAD ACROSS TWO SETS OF DATA (EACH BOARD MIGHT BLINK AT SLIGHT DIFFERENT TIME)
                BUT... one number in the pair must be > (186/2) or < -(186/2)
                SO ... should count 18 edges in total
                width of "ON" should be 400 ms
                there's some portion of green blinking from previous test are caught in this test
            */ 
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            rms_count++;
            color_data_ready = false;
        }

        // if (tick - t_0 > 1100 && a == 0) {
        //     enable_beep();
        //     a = 1;
        // }
        /* beep detection */
        if (beep_flag) {
            beep_flag = false;

            // sprintf(buffer, "%u\r\n", tick-t_0);
            // uart2_puts(buffer);

            switch (edge_direction)
            {
            case 0:                                                 // rising edge
                t_1 = tick;
                edge_direction = 1;
                break;
            
            case 1:                                                 // falling edge
                if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5% (some boards beep ON width=707ms)
                    num_beep++;
                    // sprintf(buffer, "beep: %u\r\n", num_beep);
                    // uart2_puts(buffer);
                }
                edge_direction = 0;
                break;

            default:
                break;
            }
        }

        if (rms_count == 10) {
            rms = sqrt(rms_sum/rms_num);

            rms_sum = 0;
            rms_num = 0;
            rms_count = 0;
        }

        /* timeout */
        if (tick - t_0 > 10000) {
            PORTF &= ~(1<<PF5);
            break;
        }
    }
    
    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin);
    PORTK |= (1 << relay_16[11].pin);

    // sprintf(buffer, "blink: %u beep: %u rms: %u\r\n", led_blink, num_beep, rms);
    // uart0_puts(buffer);

    disable_adc();
    stop_timer1();
    disable_blink();
    disable_beep();

    (led_blink == 9) ? (test_result[0] = 0) : (test_result[0] = 1);
    (num_beep == 9) ? (test_result[1] = 0) : (test_result[1] = 1);
    (rms < 20) ? (test_result[2] = 0) : (test_result[2] = 1);       // to simplify things, take the last measurement at end of test, if it should be zero anyways
    
    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x30 + i);
    uart0_transmit('P');

    
    // uart0_transmit(num_beep);       // lower byte for unpack() 'h'
    // uart0_transmit(0x00);           // higher byte for unpack() 'h'

    return;
}

void test_6() {
    PORTA &= ~((1 << PA4) | (1 << PA5));
    _delay_ms(250);

    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(250);
    
    // PORTA &= ~((1 << PA4) | (1 << PA5));
    // _delay_ms(250);
    uart3_puts(":01w20=165,300,\r\n");
    _delay_ms(10);
    uart3_puts(":01w12=1,\r\n");

    uint32_t t_0 = tick;                                            // record start time

    enable_beep();                                               // turn on beep detection when L1_OUT & L2_OUT comes on
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection
    start_timer2();
    init_timer3();
    start_adc();
    select_adc3();
    
    uint8_t test_result[3] = {0};
    uint8_t rms_count = 0;
    uint8_t a = 1;
    uint8_t color_temp[3] = {0}; 
    int rms_diff = 0;
    uint32_t rms_temp[2][2];
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0; 
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint32_t t_set_psu = 0;
    uint32_t t_L1_L2_OUT_OFF = 0;
    uint16_t rms_old = 0;
    uint16_t rms = 0;
    uint16_t interval = 800;
    uint8_t base_volt = 0;
    uint8_t j = 0;
    uint8_t number_beep = 0;
    bool red_on = true;
    bool rms_read = true;
    bool rms_reset = true;
    bool positive_edge_detected = false;
    bool negative_edge_detected = false;

    char *psu_voltage[] = {"0", "166", "167", "168", "169", "170", "171", "172", "173", "174", "175", "176", "177", "178", "179", "180", "181", "182", "183", "184", "185", "186", "187", "188", "189"};

    /*  initial state: {blink: green, beep: none, out: on}
        next state: {blink: red (10/5sec), beep: (10/5sec), out:off}
        stiumulus: voltage transistion from 1.6 to 1.79 (V)
        The assumption is that if DUT passes test 4, when it enters test 6, it should already be in initial state
        detection of initial state can be omitted
    */
   
    t_set_psu = tick;
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {            
        if (color_data_ready) {
            /*  blink detection */
            if ((abs(color_value[0] - color_temp[0]) > 75) && negative_edge_detected){
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;
                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;
                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];
            
            rms_count++;
            color_data_ready = false;
        }

        if (rms_count == 10) {                                      // calculate RMS at 10Hz
            rms = sqrt(rms_sum/rms_num);
            rms_diff = rms - rms_old;
            rms_old = rms;

            // sprintf(buffer, "rms: %u\r\n", rms);
            // uart0_puts(buffer);

            if ((rms_diff > 23) && (!positive_edge_detected)) {
                // uart0_transmit('+');
                // positive_edge_detected = true;
            }

            if ((rms_diff < -23) && (!negative_edge_detected)) {
                t_L1_L2_OUT_OFF = tick;                             // negative edge of L1_OUT & L2_OUT  also coincides with the rising relay click which precedes 10 beeps
                // uart0_transmit('-');
                negative_edge_detected = true;
            }

            rms_sum = 0;
            rms_num = 0;
            rms_count = 0;
        }

        /* beep detection */
        if (beep_flag && negative_edge_detected) {
            switch (edge_direction)
            {
            case 0:                                                 // rising edge
                t_1 = tick;
                edge_direction = 1;
                break;
            
            case 1:                                                 // falling edge
                if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5%
                    number_beep++;
                }
                edge_direction = 0;
                break;

            default:
                break;
            }
            beep_flag = false;
        }
        if (tick - t_0 > 3500) {
            positive_edge_detected = true;
        }
        if (positive_edge_detected) {
            if (timer3_flag) {
                char cmd[40] = ":01w10=";
                strcat(cmd, psu_voltage[j++]);
                strcat(cmd, ",\r\n");
                uart3_puts(cmd);
                if (j == sizeof(psu_voltage)/sizeof(psu_voltage[0])) {
                    break;
                }
                if (negative_edge_detected) {
                    stop_timer3();
                }
                timer3_flag = false;
            }
        }

        /* last falling edge of 10th beep + 1/2 gap of two 10-beep */
        if ((tick - t_L1_L2_OUT_OFF > 9400) && negative_edge_detected) {
            // uart0_transmit('x');
            break;
        }

        /* timeout */
        if (tick - t_0 > 30000) {
            // uart0_transmit('X');
            break;
        }
    }

    _delay_ms(500);
    uart3_puts(":01w20=0,0,\r\n");
    _delay_ms(500);
    uart3_puts(":01w12=0,\r\n");
    _delay_ms(500);

    PORTA |= (1 << PA4) | (1 << PA5);                               // disconnect TP29, TP30 to dc variable supply (PD3603A)

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin);        

    disable_adc();
    stop_timer3();
    stop_timer2();
    stop_timer1();
    disable_blink();
    disable_beep();

    

    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, number_beep);
    // uart0_puts(buffer);

    (led_blink == 10) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 10) ? (test_result[1] = 0) : (test_result[1] = 1);
    (rms < 10) ? (test_result[2] = 0) : (test_result[2] = 1);
    
    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;
}

void test_7() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRC |= (1 << relay_16[5].pin);

    // TODO
    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection

    uint8_t color_temp[3] = {0};
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t number_beep = 0;
    uint8_t test_result[3] = {0};

    uint32_t t_0 = tick;                                            // record start time
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* beep detection */
        if (beep_flag) {
            // sprintf(buffer, "beep: %u\r\n", tick - t_0);
            // uart0_puts(buffer);

            if (tick - t_0 > 1800) {
                switch (edge_direction)
                {
                case 0:                                                 // rising edge
                    t_1 = tick;
                    edge_direction = 1;
                    break;
                
                case 1:                                                 // falling edge
                    if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5%
                        number_beep++;
                        // uart0_puts("beep!\r\n");
                    }
                    edge_direction = 0;
                    break;

                default:
                    break;
                }
            }
            
            beep_flag = false;
        }

        /* timeout */
        if (tick - t_0 > 6400) {                                    // middle of gap between to beep groups
            break;
        }
    }

    stop_timer1();
    disable_blink();
    disable_beep();

    RT68_ON
    relay_call(test_points[1].pin);                                 // TP19
    _delay_ms(500);
    int volt_tp19 = multimeter_read_voltage();
    // (volt_tp19 > ? || volt_tp19 < ?) ? (test_result[2] = 0) : (test_result[2] = 1);
    RT68_OFF

    (led_blink == 4) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 4) ? (test_result[1] = 0) : (test_result[1] = 1);

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin)| (1 << relay_16[5].pin);

    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, number_beep);
    // uart0_puts(buffer);

    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;

}

void test_8() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRK |= (1 << relay_16[6].pin);

    // TODO
    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection

    uint8_t color_temp[3] = {0};
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t number_beep = 0;
    uint8_t test_result[3] = {0};

    uint32_t t_0 = tick;                                            // record start time
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* beep detection */
        if (beep_flag) {
            // sprintf(buffer, "beep: %u\r\n", tick - t_0);
            // uart0_puts(buffer);

            if (tick - t_0 > 1200) {
                switch (edge_direction)
                {
                case 0:                                                 // rising edge
                    t_1 = tick;
                    edge_direction = 1;
                    break;
                
                case 1:                                                 // falling edge
                    if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5%
                        number_beep++;
                        // uart0_transmit('~');
                    }
                    edge_direction = 0;
                    break;

                default:
                    break;
                }
            }
            
            beep_flag = false;
        }

        /* timeout */
        if (tick - t_0 > 4000) {                                    // middle of gap between to beep groups
            break;
        }
    }

    stop_timer1();
    disable_blink();
    disable_beep();

    RT68_ON
    relay_call(test_points[1].pin);                                 // TP19
    _delay_ms(500);
    int volt_tp19 = multimeter_read_voltage();
    // (volt_tp19 > ? || volt_tp19 < ?) ? (test_result[2] = 0) : (test_result[2] = 1);
    RT68_OFF

    (led_blink == 2) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 2) ? (test_result[1] = 0) : (test_result[1] = 1);

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin);
    PORTK |= (1 << relay_16[6].pin);


    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, number_beep);
    // uart0_puts(buffer);

    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;

}

void test_9() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRK |= (1 << relay_16[7].pin);

    // TODO
    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection

    uint8_t color_temp[3] = {0};
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t number_beep = 0;
    uint8_t test_result[3] = {0};

    uint32_t t_0 = tick;                                            // record start time

    // PORTF |= (1<<PF2);
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* beep detection */
        if (beep_flag) {
            // sprintf(buffer, "beep: %u\r\n", tick - t_0);
            // uart0_puts(buffer);

            if (tick - t_0 > 1800) {
                switch (edge_direction)
                {
                case 0:                                                 // rising edge
                    t_1 = tick;
                    edge_direction = 1;
                    break;
                
                case 1:                                                 // falling edge
                    if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5%
                        number_beep++;
                        // uart0_puts("beep!\r\n");
                    }
                    edge_direction = 0;
                    break;

                default:
                    break;
                }
            }
            
            beep_flag = false;
        }

        /* timeout */
        if (tick - t_0 > 6400) {                                    // middle of gap between to beep groups
            break;
        }
    }

    stop_timer1();
    disable_blink();
    disable_beep();

    RT68_ON
    relay_call(test_points[1].pin);                                 // TP19
    _delay_ms(500);
    int volt_tp19 = multimeter_read_voltage();
    // (volt_tp19 > ? || volt_tp19 < ?) ? (test_result[2] = 0) : (test_result[2] = 1);
    RT68_OFF

    (led_blink == 4) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 4) ? (test_result[1] = 0) : (test_result[1] = 1);

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin);
    PORTK |= (1 << relay_16[7].pin);

    // PORTF &= ~(1<<PF2);                                             // end of test

    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, number_beep);
    // uart0_puts(buffer);

    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;

}

void test_10() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRK |= (1 << relay_16[8].pin);

    // TODO
    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection

    uint8_t color_temp[3] = {0};
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t number_beep = 0;
    uint8_t test_result[3] = {0};

    uint32_t t_0 = tick;                                            // record start time
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;

                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;

                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            color_data_ready = false;
        }

        /* beep detection */
        if (beep_flag) {
            // sprintf(buffer, "beep: %u\r\n", tick - t_0);
            // uart0_puts(buffer);

            if (tick - t_0 > 1200) {
                switch (edge_direction)
                {
                case 0:                                                 // rising edge
                    t_1 = tick;
                    edge_direction = 1;
                    break;
                
                case 1:                                                 // falling edge
                    if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5%
                        number_beep++;
                        // uart0_puts("beep!\r\n");
                    }
                    edge_direction = 0;
                    break;

                default:
                    break;
                }
            }
            
            beep_flag = false;
        }

        /* timeout */
        if (tick - t_0 > 4000) {                                    // middle of gap between to beep groups
            break;
        }
    }

    stop_timer1();
    disable_blink();
    disable_beep();

    RT68_ON
    relay_call(test_points[1].pin);                                 // TP19
    _delay_ms(500);
    int volt_tp19 = multimeter_read_voltage();
    // (volt_tp19 > ? || volt_tp19 < ?) ? (test_result[2] = 0) : (test_result[2] = 1);
    RT68_OFF

    (led_blink == 2) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 2) ? (test_result[1] = 0) : (test_result[1] = 1);

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin);
    PORTK |= (1 << relay_16[8].pin) | (1 << relay_16[9].pin) | (1 << relay_16[10].pin);

    // sprintf(buffer, "blink: %u beep: %u\r\n", led_blink, number_beep);
    // uart0_puts(buffer);

    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;

}

void test_11() {
    PORTC &= ~(1 << relay_16[2].pin);
    _delay_ms(500);
    PORTC &= ~((1 << relay_16[0].pin) | (1 << relay_16[1].pin));
    _delay_ms(2000);
    DDRC |= (1 << relay_16[3].pin) | (1 << relay_16[4].pin);

    enable_beep();
    enable_blink();
    start_timer1();                                                 // need timer1 for blink detection
    start_adc();
    select_adc3();

    uint8_t color_temp[3] = {0};
    uint8_t led_blink = 0;
    uint8_t edge_direction = 0;                                     // default direction is rising
    uint8_t led_edge_direction = 0;                                 // default direction is rising
    uint32_t t_1 = 0;
    uint32_t t_l_1 = 0;
    uint32_t delta_t = 0;
    uint8_t rms_count = 0;
    uint16_t x = 0;
    uint8_t number_beep = 0;
    uint8_t test_result[3] = {0};
    bool rms_pass = false;

    uint32_t t_0 = tick;                                            // record start time
    color_value[0]=0;color_value[1]=0;color_value[2]=0;
    for (;;) {
        if (color_data_ready) {
            /*  blink detection */
            if (abs(color_value[0] - color_temp[0]) > 75) {
                // sprintf(buffer, "%u\r\n", tick - t_0);
                // uart0_puts(buffer);
                switch (led_edge_direction) {                       // a FSM detects rising/falling edges of LED brightness
                    case 0:
                        if (color_value[0] > color_temp[0]) {
                            t_l_1 = tick;
                            led_edge_direction = 1;
                        }
                        break;
                    case 1:
                        delta_t = tick - t_l_1;
                        if (delta_t < 420 && delta_t > 380) {       // "ON" width = 400 ms
                            led_blink++;
                            led_edge_direction = 0;
                            // uart0_transmit('*');
                        }
                        break;
                    default:
                        break;
                }
            }

            color_temp[0] = color_value[0];
            color_temp[1] = color_value[1];
            color_temp[2] = color_value[2];

            rms_count++;
            color_data_ready = false;
        }

        if (rms_count == 10) {                                      // calculate RMS at 10Hz
            // x = sqrt(rms_sum/rms_num);
            // sprintf(buffer, "rms: %u\r\n", x);
            // uart0_puts(buffer);

            rms_sum = 0;
            rms_num = 0;
            rms_count = 0;
        }

        /* beep detection */
        if (beep_flag) {
            beep_flag = false;

            // sprintf(buffer, "beep: %u\r\n", tick-t_0);
            // uart0_puts(buffer);

            switch (edge_direction)
            {
            case 0:                                                 // rising edge
                t_1 = tick;
                edge_direction = 1;
                break;
            case 1:                                                 // falling edge
                if (tick - t_1 < 899 && tick - t_1 > 415) {         // beep ON width: 655 ms +/- 5% (some boards beep ON width=707ms)
                    number_beep++;
                    // uart0_transmit('~');
                }
                edge_direction = 0;
                break;
            default:
                break;
            }
        }

        /* timeout */
        if (tick - t_0 > 4000) {                                    // middle of gap between to beep groups
            break;
        }
    }

    stop_timer1();
    disable_blink();
    disable_beep();
    stop_adc();

    PORTC |= (1 << relay_16[2].pin) | (1 << relay_16[0].pin) | (1 << relay_16[1].pin) | (1 << relay_16[3].pin) | (1 << relay_16[4].pin);

    (led_blink == 3) ? (test_result[0] = 0) : (test_result[0] = 1);
    (number_beep == 3) ? (test_result[1] = 0) : (test_result[1] = 1);
    x = sqrt(rms_sum/rms_num);
    (x < 17 ) ? (test_result[2] = 0) : (test_result[2] = 1);        // L1_OUT & L2_OUT is NOT zero! but ~90 VAC


    for (uint8_t i=0;i<3;i++) {
        if (test_result[i]) {
            uart0_transmit(0x31 + i);
            uart0_transmit('F');
            return;
        }
    }
    uart0_transmit(0x31 + 2);
    uart0_transmit('P');

    return;
}

void test_12() {
    RT67_ON
    _delay_ms(100);
    // PORTA &= ~((1 << PA6) | (1 << PA7));
    // _delay_ms(1000);

    // UCSR2B |= (1 << RXCIE2); 

    uint8_t cmd[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};

    for (uint8_t i=0;i<16;i++) {
        uart2_transmit(cmd[i]);
    }

    RT67_OFF
    // _delay_ms(1000);
    // PORTA |= ((1 << PA6) | (1 << PA7));

    // sprintf(buffer, "UART2 RX: %u\r\n", s);
    // uart0_puts(buffer);

    /* send test results at end of test */
    // struct message_packet_t message_packet;
    
    // message_packet.number_beep = 0x3030;
    // message_packet.number_blink = PS;
    // message_packet.output_volt = x;
    // message_packet.TP1 = 0x3030;
    // message_packet.TP19 = 0x3030;
    // message_packet.TP21 = volt_tp21;
    // message_packet.TP22 = volt_tp22;
    // message_packet.TP23 = volt_tp25;
    // message_packet.TP26 = volt_tp26;
    // message_packet.TP35 = volt_tp35;
    // message_packet.TP38 = 0x3030;

    // char* p = (char*)&message_packet;

    // for (uint8_t i=0;i<sizeof(message_packet);i++) {
    //     uart0_transmit(*p++);
    // }
}

void _abort() {
    // uart0_puts("test aborted!\r\n");
    
    wdt_enable(WDTO_15MS);
    for (;;);
}

void switch_color_channel(uint8_t color) {
    /* s2 pin on tcs3200 */
    if (color&0x02) {
        PORTA |= (1 << PA2);
    }
    else {
        PORTA &= ~(1 << PA2);
    }
    /* s3 pin on tcs3200 */
    if (color&0x01) {
        PORTA |= (1 << PA3);
    }
    else {
        PORTA &= ~(1 << PA3);
    }
}

static void relay_call(uint8_t pin) {
    (pin & 0x80) ? (PORTE |= (1 << PE4)) : (PORTE &= ~(1 << PE4));
    (pin & 0x40) ? (PORTE |= (1 << PE5)) : (PORTE &= ~(1 << PE5));
    (pin & 0x20) ? (PORTG |= (1 << PG5)) : (PORTG &= ~(1 << PG5));
    (pin & 0x10) ? (PORTE |= (1 << PE3)) : (PORTE &= ~(1 << PE3));

    if ((pin & 0x08) >> 3) {
        PORTH |= (1 << PH3);
    }
    else {
        PORTH &= ~(1 << PH3);
    }

    if ((pin & 0x04) >> 2) {
        PORTH |= (1 << PH4);
    }
    else {
        PORTH &= ~(1 << PH4);
    }

    if ((pin & 0x02) >> 1) {
        PORTH |= (1 << PH5);
    }
    else {
        PORTH &= ~(1 << PH5);
    }

    if (pin & 0x01) {
        PORTH |= (1 << PH6);
    }
    else {
        PORTH &= ~(1 << PH6);
    }
}

/* ISR */

ISR(TIMER0_COMPA_vect) {
    tick++;
}

ISR(TIMER1_COMPA_vect) {
    if (color_channel == 1) {
        color_value[0] = color_pulse_count;                               // update red value
        switch_color_channel(g_channel);
        color_channel = 2;
    }
    else if (color_channel == 2) {
        color_value[1] = color_pulse_count;
        switch_color_channel(b_channel);
        color_channel = 3;
    }
    else if (color_channel == 3) {
        color_value[2] = color_pulse_count;
        switch_color_channel(r_channel);
        color_channel = 1;

        color_data_ready = true;
    }
    else {
        color_channel = 1;
        switch_color_channel(r_channel);
    }

    color_pulse_count = 0;
}

ISR(TIMER3_COMPA_vect) {
    timer3_flag = true;
}

ISR(INT0_vect) {
    switch (s)
    {
    case 1:
        t_1 = TCNT2;
        EICRA |= (1 << ISC00);
        s = 2;
        break;

    case 2:
        t_2 = TCNT2;
        TCNT2 = 0;
        EICRA &= ~(1 << ISC00);
        s = 1;

        pulse[i][0] = t_2 - t_1;
        pulse[i][1] = t_2;

        if (i++>99) {
            pwm_pulse_ready = true;
            EIMSK &= ~(1 << INT0); 
        }
        break;

    default:
        TCNT2 = 0;
        s = 1;
        EICRA &= ~(1 << ISC00);
        break;
    } 
}

ISR(INT2_vect) {
    beep_flag = true;
    EICRA ^= (1 << ISC20);                                          // change direction of edge detection
}

ISR(INT3_vect) {
    color_pulse_count++;
}

ISR(USART0_RX_vect) {
    message = UDR0;
    message_ready = true;
}

ISR(USART2_RX_vect) {
    // message = UDR2;
    // message_ready = true;
    // s++;
}

ISR(ADC_vect) {
    adc_temp = abs(ADCH - 127);                                     // waveform centres around 2.5V, thus 127
    rms_sum += adc_temp * adc_temp;
    rms_num++;
}