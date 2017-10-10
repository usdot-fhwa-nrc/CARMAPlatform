/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../test_bsm/J2735_201603.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#include "RestrictionUserType.h"

static int
memb_regional_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return td->check_constraints(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t asn_PER_type_regional_constr_3 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
static asn_per_constraints_t asn_PER_memb_regional_constr_3 GCC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 2,  2,  1,  4 }	/* (SIZE(1..4)) */,
	0, 0	/* No PER value map */
};
asn_per_constraints_t asn_PER_type_RestrictionUserType_constr_1 GCC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  1,  1,  0,  1 }	/* (0..1,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_regional_3[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_RegionalExtension_99P0,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		""
		},
};
static const ber_tlv_tag_t asn_DEF_regional_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_regional_specs_3 = {
	sizeof(struct RestrictionUserType__regional),
	offsetof(struct RestrictionUserType__regional, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_regional_3 = {
	"regional",
	"regional",
	&asn_OP_SEQUENCE_OF,
	SEQUENCE_OF_constraint,
	asn_DEF_regional_tags_3,
	sizeof(asn_DEF_regional_tags_3)
		/sizeof(asn_DEF_regional_tags_3[0]) - 1, /* 1 */
	asn_DEF_regional_tags_3,	/* Same as above */
	sizeof(asn_DEF_regional_tags_3)
		/sizeof(asn_DEF_regional_tags_3[0]), /* 2 */
	0,	/* No OER visible constraints */
	&asn_PER_type_regional_constr_3,
	asn_MBR_regional_3,
	1,	/* Single element */
	&asn_SPC_regional_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_RestrictionUserType_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RestrictionUserType, choice.basicType),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RestrictionAppliesTo,
		0,
		0,	/* Defer constraints checking to the member type */
		0,	/* OER is not compiled, use -gen-OER */
		0,	/* No PER visible constraints */
		0,
		"basicType"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RestrictionUserType, choice.regional),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_regional_3,
		0,
		memb_regional_constraint_1,
		0,	/* OER is not compiled, use -gen-OER */
		&asn_PER_memb_regional_constr_3,
		0,
		"regional"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_RestrictionUserType_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* basicType */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* regional */
};
asn_CHOICE_specifics_t asn_SPC_RestrictionUserType_specs_1 = {
	sizeof(struct RestrictionUserType),
	offsetof(struct RestrictionUserType, _asn_ctx),
	offsetof(struct RestrictionUserType, present),
	sizeof(((struct RestrictionUserType *)0)->present),
	asn_MAP_RestrictionUserType_tag2el_1,
	2,	/* Count of tags in the map */
	0,
	2	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_RestrictionUserType = {
	"RestrictionUserType",
	"RestrictionUserType",
	&asn_OP_CHOICE,
	CHOICE_constraint,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	0,	/* No OER visible constraints */
	&asn_PER_type_RestrictionUserType_constr_1,
	asn_MBR_RestrictionUserType_1,
	2,	/* Elements count */
	&asn_SPC_RestrictionUserType_specs_1	/* Additional specs */
};

