/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_EventDescription_H_
#define	_EventDescription_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ITIScodes.h"
#include "Priority.h"
#include "HeadingSlice.h"
#include "Extent.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RegionalExtension;

/* EventDescription */
typedef struct EventDescription {
	ITIScodes_t	 typeEvent;
	struct EventDescription__description {
		A_SEQUENCE_OF(ITIScodes_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *description;
	Priority_t	*priority	/* OPTIONAL */;
	HeadingSlice_t	*heading	/* OPTIONAL */;
	Extent_t	*extent	/* OPTIONAL */;
	struct EventDescription__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} EventDescription_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_EventDescription;
extern asn_SEQUENCE_specifics_t asn_SPC_EventDescription_specs_1;
extern asn_TYPE_member_t asn_MBR_EventDescription_1[6];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RegionalExtension.h"

#endif	/* _EventDescription_H_ */
#include <asn_internal.h>
