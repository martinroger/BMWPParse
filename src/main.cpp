#include <Arduino.h>
//#include "kwpDaemon.h"
//#include "esp_log.h"
#include "driver/twai.h"

#ifndef CAN_TX
	#define CAN_TX D2
#endif
#ifndef CAN_RX
	#define CAN_RX D1
#endif

#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 0
#define SERIAL_UPDATE_RATE 100 //Serial refresh of the status

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was sent
unsigned long previousDebug = 0;	//Will store last time the alerts were sent over Serial

uint32_t alerts_triggered;
unsigned long currentMillis;

void setup() {
	//Serial Debug
	pinMode(LED_BUILTIN,OUTPUT);
	digitalWrite(LED_BUILTIN,!LOW);

	Serial.begin(115200);
	while(Serial.available()>0) {
		Serial.read();
		digitalWrite(LED_BUILTIN,!HIGH);
	}
	while(Serial.available()==0) {
		delay(10);
		digitalWrite(LED_BUILTIN,!HIGH);
	}
	digitalWrite(LED_BUILTIN,!LOW);

	//Twai configuration
	twai_general_config_t g_config =  {	.mode = TWAI_MODE_NORMAL, 
										.tx_io = (gpio_num_t) CAN_TX, 
										.rx_io = (gpio_num_t) CAN_RX, 
										.clkout_io = TWAI_IO_UNUSED, 
										.bus_off_io = TWAI_IO_UNUSED,      
										.tx_queue_len = 128, 
										.rx_queue_len = 128,       
										.alerts_enabled = TWAI_ALERT_NONE,  
										.clkout_divider = 0,        
										.intr_flags = ESP_INTR_FLAG_LEVEL1};
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  
  	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	//Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    	Serial.println("Driver installed");
  	} else {
		Serial.println("Failed to install driver");
		return;
  	}

	// Start TWAI driver
	if (twai_start() == ESP_OK) {
		Serial.println("Driver started");
	} else {
		Serial.println("Failed to start driver");
		return;
	}

	// Reconfigure alerts to detect TX alerts and Bus-Off errors
	uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
	if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
		Serial.println("CAN Alerts reconfigured");
	} else {
		Serial.println("Failed to reconfigure alerts");
		return;
	}

	// TWAI driver is now successfully installed and started
  	driver_installed = true;

}

static void send_message() {
  // Send message

  // Configure message to transmit
  twai_message_t message;
  message.identifier = 0x0F6;
  message.data_length_code = 8;
  for (int i = 0; i < 4; i++) {
    message.data[i] = 0xAA;
  }

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
    //printf("Message queued for transmission\n");
  } else {
    //printf("Failed to queue message for transmission\n");
  }
}

static void handle_rx_message(twai_message_t &message) {
  // Process received message
  if (message.extd) {
    Serial.println("Message is in Extended Format");
  } else {
    Serial.println("Message is in Standard Format");
  }
  Serial.printf("ID: %lx\nByte:", message.identifier);
  if (!(message.rtr)) {
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }
    Serial.println("");
  }
}

void loop() {
	//Eject if driver is uninstalled
	if (!driver_installed) {
    	// Driver not installed
    	delay(1000);
    	return;
  	}
	
	// Check if alert happened
	currentMillis = millis();
	if (currentMillis - previousDebug >= SERIAL_UPDATE_RATE) {
		previousDebug = currentMillis;
		twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
		twai_status_info_t twaistatus;
		twai_get_status_info(&twaistatus);	
		// Handle alerts
		if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
			Serial.println("Alert: TWAI controller has become error passive.");
		}
		if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
			Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
			Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
		}
		if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
			Serial.println("Alert: The Transmission failed.");
			Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
			Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
			Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
		}
		if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
			Serial.println("Alert: The Transmission was successful.");
			Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
		}
		if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
			Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
			Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
			Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
			Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  		}
		if (alerts_triggered & TWAI_ALERT_RX_DATA) {
			// One or more messages received. Handle all.
			twai_message_t message;
			while (twai_receive(&message, 0) == ESP_OK) {
			 	handle_rx_message(message);
			}
  		}
	}
	
	// Send message
	currentMillis = millis();
	if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
		previousMillis = currentMillis;
		send_message();
	}
}