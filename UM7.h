#ifndef UM7_H
#define UM7_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

// Structure for holding packet information
typedef struct UM7_packet_struct {
	bool has_data() { return (packet_type >> 7) & 0x01; }
	bool is_batch() { return (packet_type >> 6) & 0x01; }
	bool is_hidden() { return (packet_type >> 1) & 0x01; }
	bool command_failed() { return packet_type & 0x01; }
	uint8_t batch_length() { return (packet_type >> 2) & 0x0F; }
	bool checksum_ok() {
		uint16_t checksum  = (checksum1<<8) | checksum0;	// Combine checksum1 and checksum0
		uint16_t computed_checksum = 's' + 'n' + 'p' + packet_type + address;
		for (int i = 0; i < data_length; i++){
			computed_checksum += data[i];
		}
		return checksum == computed_checksum;
	}
	
	uint8_t packet_type;
	uint8_t address;
		
	uint8_t data_length; // 4 * batch length
	uint8_t data[64];
	
	uint8_t checksum1;	// First byte of checksum
	uint8_t checksum0;	// Second byte of checksum
} UM7_packet;


class UM7 {
public:
	UM7(HardwareSerial &serial);
	void zero_gyros();
	void reset_EKF();
	void set_magnetic_reference();
	
	bool receive_data();
	void check_status(HardwareSerial &serial);
	void get_firmware_version();
	
	float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
	float gyro_x, gyro_y, gyro_z;
	
private:
	void save();
	
	UM7_packet recv_packet;

	// helper variables to receive packets
	uint8_t recv_state;
	enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};
	uint8_t recv_data_index;
	
	uint8_t cmd_buffer[7];
	
	HardwareSerial &serial_port;
	uint32_t error;
	char firmware_string[5];
	bool new_fw_string;
};

#endif
