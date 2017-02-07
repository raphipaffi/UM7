#include "UM7.h"

#define GET_FW_REVISION		 0xAA
#define DREG_HEALTH			 0x55
#define DREG_GYRO_PROC_X 	 0x61
#define DREG_GYRO_PROC_Y 	 0x62
#define DREG_GYRO_PROC_Z	 0x63
#define DREG_EULER_PHI_THETA 0x70


UM7::UM7(HardwareSerial &serial) : recv_state(STATE_ZERO), serial_port(serial) {
	roll = pitch = yaw = roll_rate = pitch_rate = yaw_rate = 0.0;
	gyro_x = gyro_y = gyro_z = 0.0;
	for (int i = 0; i < 5; ++i)
		firmware_string[i] = 0;
	new_fw_string = false;
}

void UM7::get_firmware_version() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0; // packet type (must be set to zero for a command operation)
	cmd_buffer[4] = GET_FW_REVISION; // address
	
	uint16_t checksum = 's' + 'n' + 'p' + 0 + GET_FW_REVISION;
	
	cmd_buffer[5] = checksum >> 8;
	cmd_buffer[6] = checksum & 0xFF;
	
	serial_port.write(cmd_buffer, 7);
}

void UM7::zero_gyros() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0; // packet type (must be set to zero for a command operation)
	cmd_buffer[4] = 0xAD; // address
	
	uint16_t checksum = 's' + 'n' + 'p' + 0 + 0xAD;
	
	cmd_buffer[5] = checksum >> 8;
	cmd_buffer[6] = checksum & 0xFF;
	
	serial_port.write(cmd_buffer, 7);
	
	//delay(4000);
}

void UM7::reset_EKF() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0; // packet type (must be set to zero for a command operation)
	cmd_buffer[4] = 0xB3; // address
	
	uint16_t checksum = 's' + 'n' + 'p' + 0 + 0xB3;
	
	cmd_buffer[5] = checksum >> 8;
	cmd_buffer[6] = checksum & 0xFF;
	
	serial_port.write(cmd_buffer, 7);
}

void UM7::set_magnetic_reference() {
	cmd_buffer[0] = 's';
	cmd_buffer[1] = 'n';
	cmd_buffer[2] = 'p';
	cmd_buffer[3] = 0; // packet type (must be set to zero for a command operation)
	cmd_buffer[4] = 0xB0; // address
	
	uint16_t checksum = 's' + 'n' + 'p' + 0 + 0xB0;
	
	cmd_buffer[5] = checksum >> 8;
	cmd_buffer[6] = checksum & 0xFF;
	
	serial_port.write(cmd_buffer, 7);
}

bool UM7::receive_data() {	
	while (true) {
		while (serial_port.available() < 1) delay(1);
		uint8_t c = serial_port.read();
				
		switch(recv_state){
		case STATE_ZERO:
			if (c == 's'){
				recv_state = STATE_S;		// Entering state S from state Zero
			} else {
				recv_state = STATE_ZERO;
			}
			continue;
		case STATE_S:
			if (c == 'n'){
				recv_state = STATE_SN;		// Entering state SN from state S
			} else {
				recv_state = STATE_ZERO;
			}
			continue;
		case STATE_SN:
			if (c == 'p'){
				recv_state = STATE_SNP;		// Entering state SNP from state SN.  Packet header detected.
			} else {
				recv_state = STATE_ZERO;
			}
			continue;
		case STATE_SNP:
			recv_state = STATE_PT;			// Entering state PT from state SNP.  Decode packet type.
			recv_packet.packet_type = c;
			if (recv_packet.has_data()){
				if (recv_packet.is_batch()){
					recv_packet.data_length = 4 * recv_packet.batch_length();	// Each data packet is 4 bytes long
				}
				else {
					recv_packet.data_length = 4;
				}
			} else {
				recv_packet.data_length = 0;
			}  
			continue;
		case STATE_PT:
			recv_state = STATE_DATA;		// Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
			recv_packet.address = c;
			recv_data_index = 0;
			continue;
		case STATE_DATA:			//  Entering state READ_DATA.  Stay in state until all data is read.
			recv_packet.data[recv_data_index] = c;
			recv_data_index++;
			if (recv_data_index >= recv_packet.data_length){
				recv_state = STATE_CHK1;	//  Data read completed.  Next state will be CHK1
			}
			continue;
		case STATE_CHK1:			// Entering state CHK1.  Next state will be CHK0
			recv_state = STATE_CHK0;
			recv_packet.checksum1 = c;
			continue;
		case STATE_CHK0: 				
			recv_state = STATE_ZERO;		// Entering state CHK0.  Next state will be state Zero.
			recv_packet.checksum0 = c;
			if (recv_packet.checksum_ok()) {
				save(); // save data fields to class members
				return true;
			}
			return false;
		}
	}
}

typedef union {
    float val;
    uint8_t bytes[4];
} floatval;

void UM7::save() {
	switch(recv_packet.address) {
	case GET_FW_REVISION:
		for (int i = 0; i < 4; ++i)
			firmware_string[i] = (char)recv_packet.data[i];
		new_fw_string = true;
		break;
		
	case DREG_HEALTH:
		error = (uint32_t)recv_packet.data[0]<<24 | (uint32_t)recv_packet.data[1]<<16 | (uint32_t)recv_packet.data[2]<<8 | (uint32_t)recv_packet.data[3];
		break;
			
	case DREG_EULER_PHI_THETA:
		roll  = (float)((recv_packet.data[0]<<8) | recv_packet.data[1]) / 91.02222;
		pitch = (float)((recv_packet.data[2]<<8) | recv_packet.data[3]) / 91.02222;
		if (recv_packet.data_length > 4)
			yaw   = (float)((recv_packet.data[4]<<8) | recv_packet.data[5]) / 91.02222;
		if (recv_packet.data_length > 8) {
			roll_rate  = (float)((recv_packet.data[8]<<8) | recv_packet.data[9] ) / 91.02222;
			pitch_rate = (float)((recv_packet.data[10]<<8) | recv_packet.data[11]) / 91.02222;
		}
		if (recv_packet.data_length > 12)
			yaw_rate   = (float)((recv_packet.data[12]<<8) | recv_packet.data[13]) / 91.02222;
		break;
		
	case DREG_GYRO_PROC_X:
		floatval f;
		f.bytes[0] = recv_packet.data[3];
		f.bytes[1] = recv_packet.data[2];
		f.bytes[2] = recv_packet.data[1];
		f.bytes[3] = recv_packet.data[0];
		gyro_x = f.val;
		if (recv_packet.data_length > 4) {
			f.bytes[0] = recv_packet.data[7];
			f.bytes[1] = recv_packet.data[6];
			f.bytes[2] = recv_packet.data[5];
			f.bytes[3] = recv_packet.data[4];
			gyro_y = f.val;
		}
		if (recv_packet.data_length > 8) {
			f.bytes[0] = recv_packet.data[11];
			f.bytes[1] = recv_packet.data[10];
			f.bytes[2] = recv_packet.data[9];
			f.bytes[3] = recv_packet.data[8];
			gyro_z = f.val;
		}
		break;
	}
}

void UM7::check_status(HardwareSerial &serial) {
	if (new_fw_string) {
		Serial.println(firmware_string);
		new_fw_string = false;
	}
	if (error & (1<<8)) {
		serial.println("Data overflow on serial port.");
	}
	if (error & (1<<5)) {
		serial.println("Magnetometer signal larger than expected. Orientation estimate not reliable.");
	}
	if (error & (1<<4)) {
		serial.println("Acceleration much larger than 1 G. Orientation estimate not reliable.");
	}
	if (error & (1<<3)) {
		serial.println("Accelerometer failed to initialize on startup.");
	}
	if (error & (1<<2)) {
		serial.println("Gyro failed to initialize on startup.");
	}
	if (error & (1<<1)) {
		serial.println("Magnetometer failed to initialize on startup.");
	}
}


