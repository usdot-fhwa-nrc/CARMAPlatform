/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "NodeListXY.h"

asn_per_constraints_t asn_PER_type_NodeListXY_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  1,  1,  0,  1 }	/* (0..1,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_NodeListXY_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct NodeListXY, choice.nodes),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NodeSetXY,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"nodes"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct NodeListXY, choice.computed),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ComputedLane,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"computed"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_NodeListXY_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* nodes */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* computed */
};
asn_CHOICE_specifics_t asn_SPC_NodeListXY_specs_1 = {
	sizeof(struct NodeListXY),
	offsetof(struct NodeListXY, _asn_ctx),
	offsetof(struct NodeListXY, present),
	sizeof(((struct NodeListXY *)0)->present),
	asn_MAP_NodeListXY_tag2el_1,
	2,	/* Count of tags in the map */
	0,
	2	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_NodeListXY = {
	"NodeListXY",
	"NodeListXY",
	&asn_OP_CHOICE,
	CHOICE_constraint,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	0,	/* No OER visible constraints */
	&asn_PER_type_NodeListXY_constr_1,
	asn_MBR_NodeListXY_1,
	2,	/* Elements count */
	&asn_SPC_NodeListXY_specs_1	/* Additional specs */
};

