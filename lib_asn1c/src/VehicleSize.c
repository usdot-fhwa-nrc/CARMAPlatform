/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "VehicleSize.h"

asn_TYPE_member_t asn_MBR_VehicleSize_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSize, width),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleWidth,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"width"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSize, length),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleLength,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"length"
		},
};
static const ber_tlv_tag_t asn_DEF_VehicleSize_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleSize_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* width */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* length */
};
asn_SEQUENCE_specifics_t asn_SPC_VehicleSize_specs_1 = {
	sizeof(struct VehicleSize),
	offsetof(struct VehicleSize, _asn_ctx),
	asn_MAP_VehicleSize_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_VehicleSize = {
	"VehicleSize",
	"VehicleSize",
	&asn_OP_SEQUENCE,
	SEQUENCE_constraint,
	asn_DEF_VehicleSize_tags_1,
	sizeof(asn_DEF_VehicleSize_tags_1)
		/sizeof(asn_DEF_VehicleSize_tags_1[0]), /* 1 */
	asn_DEF_VehicleSize_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleSize_tags_1)
		/sizeof(asn_DEF_VehicleSize_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	0,	/* No PER visible constraints */
	asn_MBR_VehicleSize_1,
	2,	/* Elements count */
	&asn_SPC_VehicleSize_specs_1	/* Additional specs */
};

