/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "IsDolly.h"

/*
 * This type is implemented using BOOLEAN,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_IsDolly_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (1 << 2))
};
asn_TYPE_descriptor_t asn_DEF_IsDolly = {
	"IsDolly",
	"IsDolly",
	&asn_OP_BOOLEAN,
	BOOLEAN_constraint,
	asn_DEF_IsDolly_tags_1,
	sizeof(asn_DEF_IsDolly_tags_1)
		/sizeof(asn_DEF_IsDolly_tags_1[0]), /* 1 */
	asn_DEF_IsDolly_tags_1,	/* Same as above */
	sizeof(asn_DEF_IsDolly_tags_1)
		/sizeof(asn_DEF_IsDolly_tags_1[0]), /* 1 */
	0,	/* No OER visible constraints */
	0,	/* No PER visible constraints */
	0, 0,	/* No members */
	0	/* No specifics */
};

