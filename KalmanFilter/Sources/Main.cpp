/*
 * main.cpp
 *
 *  Created on: 25 Nov 2020
 *      Author: Arion
 */

#include <thread>
#include <iostream>
#include <cmath>

#include "TinyEKF.h"

#include "CSV.h"

#define ADJUSTED_SEA_LEVEL_PRESSURE 1018.6

void transmit(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float pressure) {
	IMU_data imu;
	imu.acceleration.x = accelX;
	imu.acceleration.y = accelY;
	imu.acceleration.z = accelZ;
	imu.eulerAngles.x = gyroX;
	imu.eulerAngles.y = gyroY;
	imu.eulerAngles.z = gyroZ;

	BARO_data baro;
	baro.pressure = pressure;
	baro.altitude = 44330 * (1.0 - pow(baro.pressure / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));

	kalmanProcessIMU(imu);
	kalmanProcessBaro(baro);
}

int main() {

	std::thread kalman(TK_kalman);

	std::cout << "Loading flight data... ";
	std::vector<std::vector<float>> flight_data = read_csv("Flight.csv");
	std::cout << "done" << std::endl;

	std::cout << "Started emulating." << std::endl;

	std::vector<float>::iterator time = flight_data.at(0).begin();
	std::vector<float>::iterator pressure = flight_data.at(2).begin();
	std::vector<float>::iterator accelX = flight_data.at(4).begin();
	std::vector<float>::iterator accelY = flight_data.at(5).begin();
	std::vector<float>::iterator accelZ = flight_data.at(6).begin();
	std::vector<float>::iterator gyroX = flight_data.at(7).begin();
	std::vector<float>::iterator gyroY = flight_data.at(8).begin();
	std::vector<float>::iterator gyroZ = flight_data.at(9).begin();

	std::cout << *pressure << std::endl;
	std::cout << *accelX << std::endl;
	std::cout << *accelY << std::endl;
	std::cout << *accelZ << std::endl;

	std::cout << "Lift-off in 30s..." << std::endl;
	long init_time = 1820000; // milliseconds (10 seconds before lift-off)
	long start_time = HAL_GetTick();

	while(time != flight_data.at(0).end()) {
		// Sync emulation
		long ltime = (int) (1000 * *time);
		long delta = ltime - (HAL_GetTick() - start_time + init_time);

		if(delta > 0) {
			osDelay(delta);
		}

		time++;

		transmit(*accelX++, *accelY++, *accelZ++, *gyroX++, *gyroY++, *gyroZ++, *pressure++);
	}

	std::cout << "Emulation terminated." << std::endl;

	while(true);

	return 0;
}
