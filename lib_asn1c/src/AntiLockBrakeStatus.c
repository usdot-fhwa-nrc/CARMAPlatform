/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "AntiLockBrakeStatus.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
asn_per_constraints_t asn_PER_type_AntiLockBrakeStatus_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_AntiLockBrakeStatus_value2enum_1[] = {
	{ 0,	11,	"unavailable" },
	{ 1,	3,	"off" },
	{ 2,	2,	"on" },
	{ 3,	7,	"engaged" }
};
static const unsigned int asn_MAP_AntiLockBrakeStatus_enum2value_1[] = {
	3,	/* engaged(3) */
	1,	/* off(1) */
	2,	/* on(2) */
	0	/* unavailable(0) */
};
const asn_INTEGER_specifics_t asn_SPC_AntiLockBrakeStatus_specs_1 = {
	asn_MAP_AntiLockBrakeStatus_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_AntiLockBrakeStatus_enum2value_1,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_AntiLockBrakeStatus_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_AntiLockBrakeStatus = {
	"AntiLockBrakeStatus",
	"AntiLockBrakeStatus",
	&asn_OP_NativeEnumerated,
	NativeEnumerated_constraint,
	asn_DEF_AntiLockBrakeStatus_tags_1,
	sizeof(asn_DEF_AntiLockBrakeStatus_tags_1)
		/sizeof(asn_DEF_AntiLockBrakeStatus_tags_1[0]), /* 1 */
	asn_DEF_AntiLockBrakeStatus_tags_1,	/* Same as above */
	sizeof(asn_DEF_AntiLockBrakeStatus_tags_1)
		/sizeof(asn_DEF_AntiLockBrakeStatus_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	&asn_PER_type_AntiLockBrakeStatus_constr_1,
	0, 0,	/* Defined elsewhere */
	&asn_SPC_AntiLockBrakeStatus_specs_1	/* Additional specs */
};

