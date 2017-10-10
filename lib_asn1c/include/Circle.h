/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_Circle_H_
#define	_Circle_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Position3D.h"
#include "Radius-B12.h"
#include "DistanceUnits.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Circle */
typedef struct Circle {
	Position3D_t	 center;
	Radius_B12_t	 radius;
	DistanceUnits_t	 units;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Circle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Circle;
extern asn_SEQUENCE_specifics_t asn_SPC_Circle_specs_1;
extern asn_TYPE_member_t asn_MBR_Circle_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _Circle_H_ */
#include <asn_internal.h>
