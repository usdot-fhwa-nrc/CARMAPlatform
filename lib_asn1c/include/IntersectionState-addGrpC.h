/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_IntersectionState_addGrpC_H_
#define	_IntersectionState_addGrpC_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PrioritizationResponseList;

/* IntersectionState-addGrpC */
typedef struct IntersectionState_addGrpC {
	struct PrioritizationResponseList	*activePrioritizations	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntersectionState_addGrpC_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntersectionState_addGrpC;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PrioritizationResponseList.h"

#endif	/* _IntersectionState_addGrpC_H_ */
#include <asn_internal.h>
