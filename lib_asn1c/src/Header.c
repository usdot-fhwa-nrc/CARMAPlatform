/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "Header.h"

asn_TYPE_member_t asn_MBR_Header_1[] = {
	{ ATF_POINTER, 4, offsetof(struct Header, year),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DYear,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"year"
		},
	{ ATF_POINTER, 3, offsetof(struct Header, timeStamp),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MinuteOfTheYear,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"timeStamp"
		},
	{ ATF_POINTER, 2, offsetof(struct Header, secMark),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSecond,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"secMark"
		},
	{ ATF_POINTER, 1, offsetof(struct Header, msgIssueRevision),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSRC_MsgCount,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"msgIssueRevision"
		},
};
static const int asn_MAP_Header_oms_1[] = { 0, 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_Header_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Header_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* year */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* timeStamp */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* secMark */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* msgIssueRevision */
};
asn_SEQUENCE_specifics_t asn_SPC_Header_specs_1 = {
	sizeof(struct Header),
	offsetof(struct Header, _asn_ctx),
	asn_MAP_Header_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_Header_oms_1,	/* Optional members */
	4, 0,	/* Root/Additions */
	3,	/* Start extensions */
	5	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_Header = {
	"Header",
	"Header",
	&asn_OP_SEQUENCE,
	SEQUENCE_constraint,
	asn_DEF_Header_tags_1,
	sizeof(asn_DEF_Header_tags_1)
		/sizeof(asn_DEF_Header_tags_1[0]), /* 1 */
	asn_DEF_Header_tags_1,	/* Same as above */
	sizeof(asn_DEF_Header_tags_1)
		/sizeof(asn_DEF_Header_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	0,	/* No PER visible constraints */
	asn_MBR_Header_1,
	4,	/* Elements count */
	&asn_SPC_Header_specs_1	/* Additional specs */
};

