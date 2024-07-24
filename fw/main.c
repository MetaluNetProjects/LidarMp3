// Lidar Mp3 firmware

#define BOARD pico
#include "fraise.h"
#include "hardware/uart.h"
#include "string.h"
#include "fraise_eeprom.h"


int distance_high = 2000;
int distance_low = 800;
int bg_substract = 100;
float bg_compress = 0.9;
int bg_min_width = 2;
int snap_smooth = 2;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
int ledPeriod = 250;

const uint BUTTON_PIN = 8;

const uint LIDAR_TX_PIN = 0;
const uint LIDAR_RX_PIN = 1;
uart_inst_t *LIDAR_UART = uart0;

const uint MP3_TX_PIN = 4;
const uint MP3_RX_PIN = 5;
uart_inst_t *MP3_UART = uart1;

#define CLIP(x, min, max) MAX(min, MIN(max, x))

bool button, button_last;
int button_count;

void mp3_play(uint index) {
	uart_putc_raw(MP3_UART, 0x7E);
	uart_putc_raw(MP3_UART, 0xFF);
	uart_putc_raw(MP3_UART, 0x06);
	uart_putc_raw(MP3_UART, 0x08);
	uart_putc_raw(MP3_UART, 0x00);
	uart_putc_raw(MP3_UART, index >> 8);
	uart_putc_raw(MP3_UART, index & 0xFF);
	uart_putc_raw(MP3_UART, 0xEF);
}

void mp3_volume(uint volume) {
	if(volume > 30) volume = 30;
	uart_putc_raw(MP3_UART, 0x7E);
	uart_putc_raw(MP3_UART, 0xFF);
	uart_putc_raw(MP3_UART, 0x06);
	uart_putc_raw(MP3_UART, 0x06);
	uart_putc_raw(MP3_UART, 0x00);
	uart_putc_raw(MP3_UART, 0x00);
	uart_putc_raw(MP3_UART, volume);
	uart_putc_raw(MP3_UART, 0xEF);
}

void setup_mp3() {
	uart_init(MP3_UART, 9600);
	gpio_set_function(MP3_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(MP3_RX_PIN, GPIO_FUNC_UART);
}

enum {STOP, START, RUN, SNAP_PRE, SNAP, SNAP_POST} lidar_state = STOP;
bool lidar_maxing = false;
bool lidar_header_ok;
uint16_t lidar_distance[360];
uint16_t lidar_background[360];
uint16_t lidar_distance_masked[360];
uint8_t lidar_written[360];
uint8_t lidar_buffer[10];
uint8_t lidar_bufcount;
const uint8_t lidar_header_ref[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

// irq handler that stores the distance map
void lidar_irq() {
	while(uart_is_readable(LIDAR_UART)) {
		uint8_t b = uart_getc(LIDAR_UART);
		if(lidar_bufcount < 10) lidar_buffer[lidar_bufcount++] = b;
		if(! lidar_header_ok) {
			if(
				(lidar_bufcount == 7) &&
				(memcmp(lidar_buffer, lidar_header_ref, sizeof(lidar_header_ref)) == 0)
			) {
				lidar_header_ok = true;
				lidar_bufcount = 0;
				printf("l lidar scan started\n");
			}
		} else {
			if(lidar_bufcount == 5) {
				int a = ((lidar_buffer[1] + lidar_buffer[2] * 128) / 64) % 360;
				int d = (lidar_buffer[3] + lidar_buffer[4] * 256) / 4;
				if(d) {
					if(lidar_maxing) lidar_distance[a] = MIN(lidar_distance[a], d);
					else lidar_distance[a] = d;
					lidar_written[a] = 1;
				}
				lidar_bufcount = 0;
			}
		}
	}
}

void lidar_reset() {
	uart_putc_raw(LIDAR_UART, 0xA5);
	uart_putc_raw(LIDAR_UART, 0x40);
}

void lidar_start() {
	uart_putc_raw(LIDAR_UART, 0xA5);
	uart_putc_raw(LIDAR_UART, 0x20);
	lidar_header_ok = false;
	lidar_bufcount = 0;
}

void lidar_stop() {
	uart_putc_raw(LIDAR_UART, 0xA5);
	uart_putc_raw(LIDAR_UART, 0x25);
}

void lidar_background_snap() {
	for(int a = 0; a < 360; a++) {
		int d = 12000;
		for(int o = -snap_smooth; o <= snap_smooth ; o++) {
			int ao = (a + o + 360) % 360;
			if(lidar_distance[ao] < d) d = lidar_distance[ao];
		}
		if(d > bg_substract) 
			d = (d - bg_substract) * bg_compress;
		else 
			lidar_background[a] = 0;
		lidar_background[a] = d;
	}
}

void setup_lidar() {
	uart_init(LIDAR_UART, 460800);
	gpio_set_function(LIDAR_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);

	// Set up a RX interrupt
	// We need to set up the handler first
	// Select correct interrupt for the UART we are using
	int UART_IRQ = LIDAR_UART == uart0 ? UART0_IRQ : UART1_IRQ;

	// And set up and enable the interrupt handlers
	irq_set_exclusive_handler(UART_IRQ, lidar_irq);
	irq_set_enabled(UART_IRQ, true);

	// Now enable the UART to send interrupts - RX only
	uart_set_irq_enables(LIDAR_UART, true, false);
}

void lidar_update() {
	static absolute_time_t update_time;
	static int volume;
	if(!time_reached(update_time)) return;
	update_time = make_timeout_time_ms(100);
	switch(lidar_state) {
		case STOP: break;
		case START: 
			lidar_start();
			update_time = make_timeout_time_ms(1000);
			lidar_state = RUN;
			break;
		case RUN:
			{
				int min_distance = 12000;
				int dist;
				// mask the distance map with the background map
				for(int a = 0; a < 360; a++) {
					if(lidar_written[a] && lidar_distance[a] > 0 && lidar_distance[a] < lidar_background[a]) {
						lidar_distance_masked[a] = lidar_distance[a];
					} else lidar_distance_masked[a] = 12000;
				}
				// discard objects that are too narrow, and find the smaller distance
				for(int a = 0; a < 360; a++) {
					if(lidar_distance_masked[a] < distance_high) {
						for(int o = 0; o < bg_min_width; o++) {
							if(lidar_distance_masked[(a + o) % 360] > distance_high) {
								lidar_distance_masked[a] = 12000;
								break;
							}
						}
					}
					if(lidar_distance_masked[a] < min_distance) min_distance = lidar_distance_masked[a];
				}
				// process and fade volume
				int vol_dest = 0;
				vol_dest = 30 - (30 * (min_distance - distance_low)) / (distance_high - distance_low);
				vol_dest = CLIP(vol_dest, 0, 30);
				if(volume < vol_dest) volume++;
				else if(volume > vol_dest) volume--;
				if(volume < vol_dest) volume++;
				else if(volume > vol_dest) volume--;
				if(volume < vol_dest) volume++;
				else if(volume > vol_dest) volume--;
				if(volume < vol_dest) volume++;
				else if(volume > vol_dest) volume--;
				printf("d %d %d\n", min_distance, volume);
				mp3_volume(volume);
				memset(lidar_written, 0, sizeof(lidar_written));
			}
			break;
		case SNAP_PRE:
			printf("l snap init\n");
			update_time = make_timeout_time_ms(4000);
			lidar_state = SNAP;
			break;
		case SNAP:
			printf("l snap start\n");
			update_time = make_timeout_time_ms(1000);
			lidar_maxing = true;
			lidar_state = SNAP_POST;
			break;
		case SNAP_POST:
			lidar_background_snap();
			lidar_maxing = false;
			lidar_stop();
			sleep_ms(500);
			eeprom_save();
			lidar_start();
			lidar_state = RUN;
			printf("l snap done\n");
			break;
		default: ;
	}
}

void setup() {
	gpio_init(BUTTON_PIN);
	gpio_set_dir(BUTTON_PIN, GPIO_IN);
	gpio_pull_up(BUTTON_PIN);

	eeprom_load();
	setup_mp3();
	sleep_ms(100);
	mp3_volume(0);
	sleep_ms(100);
	mp3_play(1);
	setup_lidar();
	lidar_state = START;
}

void loop(){
	static absolute_time_t nextLed;
	static bool led = false;

	if(time_reached(nextLed)) {
		gpio_put(LED_PIN, led = !led);
		nextLed = make_timeout_time_ms(ledPeriod);
	}

	if(gpio_get(BUTTON_PIN)) {
		if(button_count > 0) button_count--;
		else button = false;
	} else {
		if(button_count < 1000) button_count++;
		else button = true;
	}
	if(button_last != button) {
		button_last = button;
		printf("b %d\n", button);
		if(button) {
			lidar_state = SNAP_PRE;
		}
	}

	lidar_update();
}

void fraise_receivebytes(const char *data, uint8_t len){
	uint8_t command = fraise_get_uint8();

	if(command == 1) ledPeriod = (int)data[1] * 10;
	else if(command == 2) mp3_play(data[1]);
	else if(command == 3) mp3_volume(data[1]);
	else if(command == 4) {
		switch(data[1]) {
			case 0: lidar_reset(); break;
			case 1: lidar_stop(); break;
			case 2: lidar_start(); break;
			case 3: lidar_background_snap(); break;
		}
	}
	else if(command == 5) {
		for(int a = 0; a < 36 ; a++) {
			fraise_put_init();
			fraise_put_uint8(20);
			fraise_put_uint8(a);
			for(int i = 0; i < 10; i++) fraise_put_uint16(lidar_distance[a * 10 + i]);
			fraise_put_send();
		}
	}
	else if(command == 6) {
		for(int a = 0; a < 36 ; a++) {
			fraise_put_init();
			fraise_put_uint8(21);
			fraise_put_uint8(a);
			for(int i = 0; i < 10; i++) fraise_put_uint16(lidar_background[a * 10 + i]);
			fraise_put_send();
		}
	}
	else if(command == 7) {
		for(int a = 0; a < 36 ; a++) {
			fraise_put_init();
			fraise_put_uint8(22);
			fraise_put_uint8(a);
			for(int i = 0; i < 10; i++) fraise_put_uint16(lidar_distance_masked[a * 10 + i]);
			fraise_put_send();
		}
	}
	else if(command == 100) distance_high = fraise_get_uint16();
	else if(command == 101) distance_low = fraise_get_uint16();
	else if(command == 102) bg_substract = fraise_get_uint16();
	else if(command == 103) bg_min_width = fraise_get_uint16();
	else if(command == 104) snap_smooth = fraise_get_uint16();
	else {
		printf("rcvd ");
		for(int i = 0; i < len; i++) printf("%d ", (uint8_t)data[i]);
		putchar('\n');
	}
}

void fraise_receivechars(const char *data, uint8_t len){
	if(data[0] == 'E') { // Echo
		printf("E%s\n", data + 1);
	}
}

void eeprom_declare_main() {
	eeprom_declare_data((char*)lidar_background, sizeof(lidar_background));
}

