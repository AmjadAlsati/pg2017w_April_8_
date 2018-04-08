/*
 * pwrManagement.h
 *
 *  Created on: 14.04.2012
 *      Author: sid
 */

#ifndef PWRMANAGEMENT_H_
#define PWRMANAGEMENT_H_

#define PM_HD		B,7
#define PM_SN		A,0

#define PM_MINVCC	593	//5.8V

void PM_Init(void);

void PM_Poll(void);

#endif /* PWRMANAGEMENT_H_ */
