/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "SpeedandHeadingandThrottleConfidence.h"

asn_TYPE_member_t asn_MBR_SpeedandHeadingandThrottleConfidence_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SpeedandHeadingandThrottleConfidence, heading),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HeadingConfidence,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"heading"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SpeedandHeadingandThrottleConfidence, speed),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SpeedConfidence,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"speed"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SpeedandHeadingandThrottleConfidence, throttle),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ThrottleConfidence,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"throttle"
		},
};
static const ber_tlv_tag_t asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SpeedandHeadingandThrottleConfidence_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* heading */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* speed */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* throttle */
};
asn_SEQUENCE_specifics_t asn_SPC_SpeedandHeadingandThrottleConfidence_specs_1 = {
	sizeof(struct SpeedandHeadingandThrottleConfidence),
	offsetof(struct SpeedandHeadingandThrottleConfidence, _asn_ctx),
	asn_MAP_SpeedandHeadingandThrottleConfidence_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_SpeedandHeadingandThrottleConfidence = {
	"SpeedandHeadingandThrottleConfidence",
	"SpeedandHeadingandThrottleConfidence",
	&asn_OP_SEQUENCE,
	SEQUENCE_constraint,
	asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1,
	sizeof(asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1)
		/sizeof(asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1[0]), /* 1 */
	asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1,	/* Same as above */
	sizeof(asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1)
		/sizeof(asn_DEF_SpeedandHeadingandThrottleConfidence_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	0,	/* No PER visible constraints */
	asn_MBR_SpeedandHeadingandThrottleConfidence_1,
	3,	/* Elements count */
	&asn_SPC_SpeedandHeadingandThrottleConfidence_specs_1	/* Additional specs */
};

