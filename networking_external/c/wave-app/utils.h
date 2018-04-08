/*
 * utils.h
 *
 *  Created on: May 13, 2013
 *      Author: Michele Segata <segata@ccs-labs.org>
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>

/**
 * Return distance between two GPS coordinates, in meters
 *
 * \param lat1 latitude of the first position
 * \param lon1 longitude of the first position
 * \param lat2 latitude of the second position
 * \param lon2 longitude of the second position
 * \return the distance in meters
 */
double get_gps_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * Transforms an angle in degrees to radians
 *
 * \param deg angle in degrees
 * \return angle in radians
 */
double deg2rad(double deg);

/**
 * Convert MCS to string
 *
 * \param mcs the modulation and coding scheme
 * \param buffer were to store the converted mcs
 * \return buffer
 */
//char *mcs_to_string(tMK2MCS mcs, char *buffer) {
//
//	switch(mcs) {
//	case MK2MCS_R12BPSK:
//		strcpy(buffer, "MK2MCS_R12BPSK");
//		break;
//	case MK2MCS_R34BPSK:
//		strcpy(buffer, "MK2MCS_R34BPSK");
//		break;
//	case MK2MCS_R12QPSK:
//		strcpy(buffer, "MK2MCS_R12QPSK");
//	case MK2MCS_R34QPSK:
//	strcpy(buffer, "MK2MCS_R34QPSK");
//	break;
//	case MK2MCS_R12QAM16:
//	strcpy(buffer, "MK2MCS_R12QAM16");
//	break;
//	case MK2MCS_R34QAM16;
//	strcpy(buffer, "MK2MCS_R34QAM16");
//	case MK2MCS_R23QAM64;
//	case MK2MCS_R34QAM64;
//	strcpy(buffer, "MK2MCS_R34QAM64", optarg) == 0)
//	}
//
//	return buffer;
//}

#endif /* UTILS_H_ */
