/*
 * tc_data_format.h
 *
 *  Created on: Jun 9, 2016
 *      Author: priya
 */

#ifndef TC_NODE_TC_DATA_FORMAT_H_
#define TC_NODE_TC_DATA_FORMAT_H_

#include "json.h"
#include "common.h"

//number of objects in our data structure. increase this if we add more members to struct ttc_data
#define TC_DATA_NUM 14
#define SOLAR_HIFI_DEVICE_MODE_STRLEN 30

typedef struct tc_data {

	unsigned int device_id;
#define JSON_DEVICE_ID "device_id"
	char device_mode[SOLAR_HIFI_DEVICE_MODE_STRLEN];
#define JSON_DEVICE_MODE "device_mode"
	unsigned long timestamp;
#define JSON_TIMESTAMP "timestamp"
	unsigned int ibus; //pnarasim:q whats this?
#define JSON_IBUS "ibus"
	unsigned int energy_fed;
#define JSON_ENERGY_FED "energy_fed"
	unsigned int instant_energy_fed;
#define JSON_INSTANT_ENERGY_FED "instant_energy_fed"
	unsigned int energy_generated;
#define JSON_ENERGY_GENERATED "energy_generated"
	unsigned int instant_energy_generated;
#define JSON_INSTANT_ENERGY_GENERATED "instant_energy_generated"
	unsigned int pv_voltage;
#define JSON_PV_VOLTAGE "pv_voltage"
	unsigned int pv_current;
#define JSON_PV_CURRENT "pv_current"
	unsigned int grid_voltage;
#define JSON_GRID_VOLTAGE "grid_voltage"
	unsigned int grid_current;
#define JSON_GRID_CURRENT "grid_current"
	unsigned int battery_voltage;
#define JSON_BATTERY_VOLTAGE "battery_voltage"
	unsigned int battery_current;
#define JSON_BATTERY_CURRENT "battery_current"

} tc_data;

#define JSON_RESOURCE "resource"

void print_ttc_data(tc_data);
char* create_json(tc_data);

#define ENGEN_CONST 74600
#define ENFED_CONST 73200
#define VAC_CONST   1464
#define VPV_CONST   1492


#define MAX_TIME_DATA_SIZE 500

#endif /* TC_NODE_TC_DATA_FORMAT_H_ */
