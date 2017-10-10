/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "NodeAttributeXYList.h"

asn_per_constraints_t asn_PER_type_NodeAttributeXYList_constr_1 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 3,  3,  1,  8 }	/* (SIZE(1..8)) */,
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_NodeAttributeXYList_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (10 << 2)),
		0,
		&asn_DEF_NodeAttributeXY,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		""
		},
};
static const ber_tlv_tag_t asn_DEF_NodeAttributeXYList_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_NodeAttributeXYList_specs_1 = {
	sizeof(struct NodeAttributeXYList),
	offsetof(struct NodeAttributeXYList, _asn_ctx),
	1,	/* XER encoding is XMLValueList */
};
asn_TYPE_descriptor_t asn_DEF_NodeAttributeXYList = {
	"NodeAttributeXYList",
	"NodeAttributeXYList",
	&asn_OP_SEQUENCE_OF,
	SEQUENCE_OF_constraint,
	asn_DEF_NodeAttributeXYList_tags_1,
	sizeof(asn_DEF_NodeAttributeXYList_tags_1)
		/sizeof(asn_DEF_NodeAttributeXYList_tags_1[0]), /* 1 */
	asn_DEF_NodeAttributeXYList_tags_1,	/* Same as above */
	sizeof(asn_DEF_NodeAttributeXYList_tags_1)
		/sizeof(asn_DEF_NodeAttributeXYList_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	&asn_PER_type_NodeAttributeXYList_constr_1,
	asn_MBR_NodeAttributeXYList_1,
	1,	/* Single element */
	&asn_SPC_NodeAttributeXYList_specs_1	/* Additional specs */
};

