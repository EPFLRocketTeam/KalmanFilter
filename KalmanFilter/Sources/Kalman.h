/*
 * kalman.h
 *
 *  Created on: 25 Nov 2020
 *      Author: Arion
 */

#ifndef SOURCES_KALMAN_H_
#define SOURCES_KALMAN_H_

#include <chrono>
#include <thread>

#define rocket_log printf

typedef struct {
	float x, y, z;
} float3D;

typedef struct {
	float3D acceleration;
	float3D eulerAngles;
} IMU_data;

typedef struct {
	float temperature;
	float pressure;
	float altitude;
	float base_pressure;
	float base_altitude;
} BARO_data;

inline long HAL_GetTick() {
	std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
		std::chrono::system_clock::now().time_since_epoch()
	);

	return ms.count();
};

inline void osDelay(long ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
};

#endif /* SOURCES_KALMAN_H_ */
