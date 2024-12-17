#include <stdio.h>
#include <stdlib.h>
#include <libserialport.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#define PORT "/dev/ttyACM0"
#define BAUD 115200

// MSP V2 Commands
#define MSP_GPS         106
#define MSP_RAW_IMU    102
#define MSP_ATTITUDE    108
#define MSP_ALTITUDE    109
#define MSP_ANALOG     110

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x, mag_y, mag_z;
} imu_data_t;

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
    return crc;
}

void send_msp_command(struct sp_port *port, uint16_t cmd) {
    unsigned char request[] = {
        '$', 'X', '<', 0, cmd, 0, 0, 0
    };
    
    uint8_t crc = 0;
    for (int i = 3; i < 7; i++) {
        crc = crc8_dvb_s2(crc, request[i]);
    }
    request[7] = crc;
    
    sp_blocking_write(port, request, sizeof(request), 1000);
}

void print_imu_data(unsigned char *buffer, int length) {
    if (length >= 18) {
        imu_data_t imu;
        memcpy(&imu, buffer + 6, sizeof(imu_data_t));
        
        printf("Accelerometer (raw): X=%d Y=%d Z=%d\n", imu.acc_x, imu.acc_y, imu.acc_z);
        printf("Gyroscope (raw): X=%d Y=%d Z=%d\n", imu.gyro_x, imu.gyro_y, imu.gyro_z);
        printf("Magnetometer (raw): X=%d Y=%d Z=%d\n", imu.mag_x, imu.mag_y, imu.mag_z);
    }
}

int main() {
    struct sp_port *port;
    sp_get_port_by_name(PORT, &port);
    sp_open(port, SP_MODE_READ_WRITE);
    
    sp_set_baudrate(port, BAUD);
    sp_set_bits(port, 8);
    sp_set_parity(port, SP_PARITY_NONE);
    sp_set_stopbits(port, 1);
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);

    unsigned char buffer[256];
    
    while(1) {
        // Request IMU data
        send_msp_command(port, MSP_RAW_IMU);
        int bytes_read = sp_blocking_read(port, buffer, sizeof(buffer), 1000);
        if (bytes_read > 0) {
            print_imu_data(buffer, bytes_read);
        }
        
        // Request altitude
        send_msp_command(port, MSP_ALTITUDE);
        bytes_read = sp_blocking_read(port, buffer, sizeof(buffer), 1000);
        if (bytes_read >= 6) {
            int32_t altitude = *(int32_t*)(buffer + 6);
            printf("Altitude: %d cm\n", altitude);
        }
        
        // Request attitude (angles)
        send_msp_command(port, MSP_ATTITUDE);
        bytes_read = sp_blocking_read(port, buffer, sizeof(buffer), 1000);
        if (bytes_read >= 6) {
            int16_t roll = *(int16_t*)(buffer + 6);
            int16_t pitch = *(int16_t*)(buffer + 8);
            int16_t yaw = *(int16_t*)(buffer + 10);
            printf("Attitude: Roll=%d° Pitch=%d° Yaw=%d°\n", roll/10, pitch/10, yaw);
        }
        
        printf("\n");
        sleep(1);
    }
    
    sp_close(port);
    sp_free_port(port);
    return 0;
}
