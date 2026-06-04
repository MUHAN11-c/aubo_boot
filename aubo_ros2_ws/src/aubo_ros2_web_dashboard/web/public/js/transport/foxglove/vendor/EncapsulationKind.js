// From <https://www.omg.org/spec/DDS-XTypes/1.3/PDF>
// ============= NOTE DDS-XTypes spec v1.3 ==========
// "7.4.3.4 Functions related to data types and objects"
// ENC_HEADER
// {0x00, 0x00} -- PLAIN_CDR, BIG_ENDIAN,
// {0x00, 0x01} -- PLAIN_CDR, LITTLE_ENDIAN
// {0x00, 0x02} -- PL_CDR, BIG_ENDIAN,
// {0x00, 0x03} -- PL_CDR, LITTLE_ENDIAN,
// {0x00, 0x10} -- PLAIN_CDR2, BIG_ENDIAN,
// {0x00, 0x11} -- PLAIN_CDR2, LITTLE_ENDIAN
// {0x00, 0x12} -- PL_CDR2, BIG_ENDIAN
// {0x00, 0x13} -- PL_CDR2, LITTLE_ENDIAN
// {0x00, 0x14} -- DELIMIT_CDR, BIG_ENDIAN
// {0x00, 0x15} -- DELIMIT_CDR, LITTLE_ENDIAN
//
// 7.6.2.1.2 RTPS encapsulation identifier
// XCDR VERSION 1
// {0x00, 0x00} -- CDR_BE, BIG_ENDIAN, FINAL
// {0x00, 0x01} -- CDR_LE, LITTLE_ENDIAN, FINAL
// {0x00, 0x00} -- CDR_BE, BIG_ENDIAN, APPENDABLE
// {0x00, 0x01} -- CDR_LE, LITTLE_ENDIAN, APPENDABLE
// {0x00, 0x02} -- PL_CDR_BE, BIG_ENDIAN, MUTABLE
// {0x00, 0x03} -- PL_CDR_LE, LITTLE_ENDIAN, MUTABLE
//
// XCDR VERSION 2
// {0x00, 0x06} -- CDR2_BE, BIG_ENDIAN, FINAL
// {0x00, 0x07} -- CDR2_LE, LITTLE_ENDIAN, FINAL
// {0x00, 0x08} -- D_CDR2_BE, BIG_ENDIAN, APPENDABLE
// {0x00, 0x09} -- D_CDR2_LE, LITTLE_ENDIAN, APPENDABLE
// {0x00, 0x0a} -- PL_CDR2_BE, BIG_ENDIAN, MUTABLE
// {0x00, 0x0b} -- PL_CDR2_LE, LITTLE_ENDIAN, MUTABLE
var EncapsulationKind;
(function (EncapsulationKind) {
    // Both RTPS and ENC_HEADER enum values
    /** Plain CDR, big-endian */
    EncapsulationKind[EncapsulationKind["CDR_BE"] = 0] = "CDR_BE";
    /** Plain CDR, little-endian */
    EncapsulationKind[EncapsulationKind["CDR_LE"] = 1] = "CDR_LE";
    /** Parameter List CDR, big-endian */
    EncapsulationKind[EncapsulationKind["PL_CDR_BE"] = 2] = "PL_CDR_BE";
    /** Parameter List CDR, little-endian */
    EncapsulationKind[EncapsulationKind["PL_CDR_LE"] = 3] = "PL_CDR_LE";
    // ENC_HEADER enum values
    /** Plain CDR2, big-endian */
    EncapsulationKind[EncapsulationKind["CDR2_BE"] = 16] = "CDR2_BE";
    /** Plain CDR2, little-endian */
    EncapsulationKind[EncapsulationKind["CDR2_LE"] = 17] = "CDR2_LE";
    /** Parameter List CDR2, big-endian */
    EncapsulationKind[EncapsulationKind["PL_CDR2_BE"] = 18] = "PL_CDR2_BE";
    /** Parameter List CDR2, little-endian */
    EncapsulationKind[EncapsulationKind["PL_CDR2_LE"] = 19] = "PL_CDR2_LE";
    /** Delimited CDR, big-endian */
    EncapsulationKind[EncapsulationKind["DELIMITED_CDR2_BE"] = 20] = "DELIMITED_CDR2_BE";
    /** Delimited CDR, little-endian */
    EncapsulationKind[EncapsulationKind["DELIMITED_CDR2_LE"] = 21] = "DELIMITED_CDR2_LE";
    // RTPS enum values
    /** Plain CDR2, big-endian */
    EncapsulationKind[EncapsulationKind["RTPS_CDR2_BE"] = 6] = "RTPS_CDR2_BE";
    /** Plain CDR2, little-endian */
    EncapsulationKind[EncapsulationKind["RTPS_CDR2_LE"] = 7] = "RTPS_CDR2_LE";
    /** Delimited CDR, big-endian */
    EncapsulationKind[EncapsulationKind["RTPS_DELIMITED_CDR2_BE"] = 8] = "RTPS_DELIMITED_CDR2_BE";
    /** Delimited CDR, little-endian */
    EncapsulationKind[EncapsulationKind["RTPS_DELIMITED_CDR2_LE"] = 9] = "RTPS_DELIMITED_CDR2_LE";
    /** Parameter List CDR2, big-endian */
    EncapsulationKind[EncapsulationKind["RTPS_PL_CDR2_BE"] = 10] = "RTPS_PL_CDR2_BE";
    /** Parameter List CDR2, little-endian */
    EncapsulationKind[EncapsulationKind["RTPS_PL_CDR2_LE"] = 11] = "RTPS_PL_CDR2_LE";
})(EncapsulationKind || (exports.EncapsulationKind = EncapsulationKind = {}));
//# sourceMappingURL=EncapsulationKind.js.map