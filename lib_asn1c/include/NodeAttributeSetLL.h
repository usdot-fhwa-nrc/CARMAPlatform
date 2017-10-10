/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_NodeAttributeSetLL_H_
#define	_NodeAttributeSetLL_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Offset-B10.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct NodeAttributeLLList;
struct SegmentAttributeLLList;
struct LaneDataAttributeList;
struct RegionalExtension;

/* NodeAttributeSetLL */
typedef struct NodeAttributeSetLL {
	struct NodeAttributeLLList	*localNode	/* OPTIONAL */;
	struct SegmentAttributeLLList	*disabled	/* OPTIONAL */;
	struct SegmentAttributeLLList	*enabled	/* OPTIONAL */;
	struct LaneDataAttributeList	*data	/* OPTIONAL */;
	Offset_B10_t	*dWidth	/* OPTIONAL */;
	Offset_B10_t	*dElevation	/* OPTIONAL */;
	struct NodeAttributeSetLL__regional {
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
} NodeAttributeSetLL_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeAttributeSetLL;
extern asn_SEQUENCE_specifics_t asn_SPC_NodeAttributeSetLL_specs_1;
extern asn_TYPE_member_t asn_MBR_NodeAttributeSetLL_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "NodeAttributeLLList.h"
#include "SegmentAttributeLLList.h"
#include "LaneDataAttributeList.h"
#include "RegionalExtension.h"

#endif	/* _NodeAttributeSetLL_H_ */
#include <asn_internal.h>
