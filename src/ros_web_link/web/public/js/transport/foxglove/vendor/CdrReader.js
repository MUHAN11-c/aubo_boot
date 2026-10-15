import * as getEncapsulationKindInfo_1 from './getEncapsulationKindInfo.js';
import * as isBigEndian_1 from './isBigEndian.js';
import * as lengthCodes_1 from './lengthCodes.js';
import * as reservedPIDs_1 from './reservedPIDs.js';
const textDecoder = new TextDecoder("utf8");
export class CdrReader {
    get kind() {
        return this.view.getUint8(1);
    }
    get decodedBytes() {
        return this.offset;
    }
    get byteLength() {
        return this.view.byteLength;
    }
    constructor(data) {
        /** Origin offset into stream used for alignment */
        this.origin = 0;
        if (data.byteLength < 4) {
            throw new Error(`Invalid CDR data size ${data.byteLength}, must contain at least a 4-byte header`);
        }
        this.view = new DataView(data.buffer, data.byteOffset, data.byteLength);
        const kind = this.kind;
        const { isCDR2, littleEndian, usesDelimiterHeader, usesMemberHeader } = (0, getEncapsulationKindInfo_1.getEncapsulationKindInfo)(kind);
        this.usesDelimiterHeader = usesDelimiterHeader;
        this.usesMemberHeader = usesMemberHeader;
        this.littleEndian = littleEndian;
        this.hostLittleEndian = !(0, isBigEndian_1.isBigEndian)();
        this.isCDR2 = isCDR2;
        this.eightByteAlignment = isCDR2 ? 4 : 8;
        this.origin = 4;
        this.offset = 4;
    }
    int8() {
        const value = this.view.getInt8(this.offset);
        this.offset += 1;
        return value;
    }
    uint8() {
        const value = this.view.getUint8(this.offset);
        this.offset += 1;
        return value;
    }
    int16() {
        this.align(2);
        const value = this.view.getInt16(this.offset, this.littleEndian);
        this.offset += 2;
        return value;
    }
    uint16() {
        this.align(2);
        const value = this.view.getUint16(this.offset, this.littleEndian);
        this.offset += 2;
        return value;
    }
    int32() {
        this.align(4);
        const value = this.view.getInt32(this.offset, this.littleEndian);
        this.offset += 4;
        return value;
    }
    uint32() {
        this.align(4);
        const value = this.view.getUint32(this.offset, this.littleEndian);
        this.offset += 4;
        return value;
    }
    int64() {
        this.align(this.eightByteAlignment);
        const value = this.view.getBigInt64(this.offset, this.littleEndian);
        this.offset += 8;
        return value;
    }
    uint64() {
        this.align(this.eightByteAlignment);
        const value = this.view.getBigUint64(this.offset, this.littleEndian);
        this.offset += 8;
        return value;
    }
    uint16BE() {
        this.align(2);
        const value = this.view.getUint16(this.offset, false);
        this.offset += 2;
        return value;
    }
    uint32BE() {
        this.align(4);
        const value = this.view.getUint32(this.offset, false);
        this.offset += 4;
        return value;
    }
    uint64BE() {
        this.align(this.eightByteAlignment);
        const value = this.view.getBigUint64(this.offset, false);
        this.offset += 8;
        return value;
    }
    float32() {
        this.align(4);
        const value = this.view.getFloat32(this.offset, this.littleEndian);
        this.offset += 4;
        return value;
    }
    float64() {
        this.align(this.eightByteAlignment);
        const value = this.view.getFloat64(this.offset, this.littleEndian);
        this.offset += 8;
        return value;
    }
    string(prereadLength) {
        const length = prereadLength ?? this.uint32();
        if (length <= 1) {
            this.offset += length;
            return "";
        }
        const data = new Uint8Array(this.view.buffer, this.view.byteOffset + this.offset, length - 1);
        const value = textDecoder.decode(data);
        this.offset += length;
        return value;
    }
    /** Reads the delimiter header which contains and returns the object size */
    dHeader() {
        const header = this.uint32();
        return header;
    }
    /**
     * Reads the member header (EMHEADER) and returns the member ID, mustUnderstand flag, and object size with optional length code
     * The length code is only present in CDR2 and should prompt objectSize to be used in place of sequence length if applicable.
     * See Extensible and Dynamic Topic Types (DDS-XTypes) v1.3 @ `7.4.3.4.2` for more info about CDR2 EMHEADER composition.
     * If a sentinelHeader was read (PL_CDR v1), the readSentinelHeader flag is set to true.
     */
    emHeader() {
        if (this.isCDR2) {
            return this.memberHeaderV2();
        }
        else {
            return this.memberHeaderV1();
        }
    }
    /** XCDR1 PL_CDR encapsulation parameter header*/
    memberHeaderV1() {
        // 4-byte header with two 16-bit fields
        this.align(4);
        const idHeader = this.uint16();
        const mustUnderstandFlag = (idHeader & 0x4000) >> 14 === 1;
        // indicates that the parameter has a implementation-specific interpretation
        const implementationSpecificFlag = (idHeader & 0x8000) >> 15 === 1;
        // Allows the specification of large member ID and/or data length values
        // requires the reading in of two uint32's for ID and size
        const extendedPIDFlag = (idHeader & 0x3fff) === reservedPIDs_1.EXTENDED_PID;
        // Indicates the end of the parameter list structure
        const sentinelPIDFlag = (idHeader & 0x3fff) === reservedPIDs_1.SENTINEL_PID;
        if (sentinelPIDFlag) {
            // Read another uint16 because the sentinel header has another uint16 after writing the flag
            this.uint16();
            // Return that we have read the sentinel header when we expected to read an emHeader.
            // This can happen for absent optional members at the end of a struct.
            return { id: reservedPIDs_1.SENTINEL_PID, objectSize: 0, mustUnderstand: false, readSentinelHeader: true };
        }
        // Indicates that the ID should be ignored
        // const ignorePIDFlag = (idHeader & 0x3fff) === 0x3f03;
        const usesReservedParameterId = (idHeader & 0x3fff) > reservedPIDs_1.SENTINEL_PID;
        // Not trying to support right now if we don't need to
        if (usesReservedParameterId || implementationSpecificFlag) {
            throw new Error(`Unsupported parameter ID header ${idHeader.toString(16)}`);
        }
        if (extendedPIDFlag) {
            // Need to consume last part of header (is just an 8 in this case)
            // Alignment could take care of this, but I want to be explicit
            this.uint16();
        }
        const id = extendedPIDFlag ? this.uint32() : idHeader & 0x3fff;
        const objectSize = extendedPIDFlag ? this.uint32() : this.uint16();
        this.resetOrigin();
        return { id, objectSize, mustUnderstand: mustUnderstandFlag };
    }
    /** Sets the origin to the offset (DDS-XTypes Spec: `PUSH(ORIGIN = 0)`)*/
    resetOrigin() {
        this.origin = this.offset;
    }
    /** Reads boolean flag for optional members in CDR2
     * Will throw an error if called for CDR1.
     */
    isPresentFlag() {
        if (!this.isCDR2) {
            throw new Error("isPresentFlag is only supported for CDR2");
        }
        const isPresent = Boolean(this.uint8());
        return isPresent;
    }
    /** Reads the PID_SENTINEL value if encapsulation kind supports it (PL_CDR version 1)
     * @returns true if the sentinel header was read, false otherwise
     */
    sentinelHeader() {
        if (!this.isCDR2) {
            this.align(4);
            const header = this.uint16();
            // Indicates the end of the parameter list structure
            const sentinelPIDFlag = (header & 0x3fff) === reservedPIDs_1.SENTINEL_PID;
            if (!sentinelPIDFlag) {
                return false;
            }
            this.uint16();
            return true;
        }
        else {
            return false;
        }
    }
    memberHeaderV2() {
        const header = this.uint32();
        // EMHEADER = (M_FLAG<<31) + (LC<<28) + M.id
        // M is the member of a structure
        // M_FLAG is the value of the Must Understand option for the member
        const mustUnderstand = Math.abs((header & 0x80000000) >> 31) === 1;
        // LC is the value of the Length Code for the member.
        const lengthCode = ((header & 0x70000000) >> 28);
        const id = header & 0x0fffffff;
        const objectSize = this.emHeaderObjectSize(lengthCode);
        return { mustUnderstand, id, objectSize, lengthCode };
    }
    /** Uses the length code to derive the member object size in
     * the EMHEADER, sometimes reading NEXTINT (the next uint32
     * following the header) from the buffer */
    emHeaderObjectSize(lengthCode) {
        // 7.4.3.4.2 Member Header (EMHEADER), Length Code (LC) and NEXTINT
        switch (lengthCode) {
            case 0:
            case 1:
            case 2:
            case 3:
                return lengthCodes_1.lengthCodeToObjectSizes[lengthCode];
            // LC > 3 -> NEXTINT exists after header
            case 4:
            case 5:
                // both 4 and 5 just read the next uint32
                return this.uint32();
            case 6:
                return 4 * this.uint32();
            case 7:
                return 8 * this.uint32();
            default:
                throw new Error(
                // eslint-disable-next-line @typescript-eslint/restrict-template-expressions
                `Invalid length code ${lengthCode} in EMHEADER at offset ${this.offset - 4}`);
        }
    }
    sequenceLength() {
        return this.uint32();
    }
    int8Array(count = this.sequenceLength()) {
        const array = new Int8Array(this.view.buffer, this.view.byteOffset + this.offset, count);
        this.offset += count;
        return array;
    }
    uint8Array(count = this.sequenceLength()) {
        const array = new Uint8Array(this.view.buffer, this.view.byteOffset + this.offset, count);
        this.offset += count;
        return array;
    }
    int16Array(count = this.sequenceLength()) {
        return this.typedArray(Int16Array, "getInt16", count);
    }
    uint16Array(count = this.sequenceLength()) {
        return this.typedArray(Uint16Array, "getUint16", count);
    }
    int32Array(count = this.sequenceLength()) {
        return this.typedArray(Int32Array, "getInt32", count);
    }
    uint32Array(count = this.sequenceLength()) {
        return this.typedArray(Uint32Array, "getUint32", count);
    }
    int64Array(count = this.sequenceLength()) {
        return this.typedArray(BigInt64Array, "getBigInt64", count, this.eightByteAlignment);
    }
    uint64Array(count = this.sequenceLength()) {
        return this.typedArray(BigUint64Array, "getBigUint64", count, this.eightByteAlignment);
    }
    float32Array(count = this.sequenceLength()) {
        return this.typedArray(Float32Array, "getFloat32", count);
    }
    float64Array(count = this.sequenceLength()) {
        return this.typedArray(Float64Array, "getFloat64", count, this.eightByteAlignment);
    }
    stringArray(count = this.sequenceLength()) {
        const output = [];
        for (let i = 0; i < count; i++) {
            output.push(this.string());
        }
        return output;
    }
    /**
     * Seek the current read pointer a number of bytes relative to the current position. Note that
     * seeking before the four-byte header is invalid
     * @param relativeOffset A positive or negative number of bytes to seek
     */
    seek(relativeOffset) {
        const newOffset = this.offset + relativeOffset;
        if (newOffset < 4 || newOffset > this.view.byteLength) {
            throw new Error(`seek(${relativeOffset}) failed, ${newOffset} is outside the data range`);
        }
        this.offset = newOffset;
    }
    /**
     * Seek to an absolute byte position in the data. Note that seeking before the four-byte header is
     * invalid
     * @param offset An absolute byte offset in the range of [4-byteLength)
     */
    seekTo(offset) {
        if (offset < 4 || offset > this.view.byteLength) {
            throw new Error(`seekTo(${offset}) failed, value is outside the data range`);
        }
        this.offset = offset;
    }
    /**
     * Duplicate this reader. The underlying buffer is reused and not copied.
     */
    clone() {
        const clone = new CdrReader(this.view);
        clone.offset = this.offset;
        clone.origin = this.origin;
        return clone;
    }
    /**
     * Limit the reader to a given number of bytes.
     * @param length The number of bytes to limit the reader to.
     */
    limit(length) {
        const newByteLength = this.offset + length;
        if (newByteLength <= this.view.byteLength) {
            this.view = new DataView(this.view.buffer, this.view.byteOffset, newByteLength);
        }
        else {
            throw new RangeError(`length ${length} exceeds byte length of view`);
        }
    }
    /**
     * Returns `true` if the reader is at the end of the buffer, or `false` otherwise.
     */
    isAtEnd() {
        return this.offset >= this.view.byteLength;
    }
    align(size) {
        const alignment = (this.offset - this.origin) % size;
        if (alignment > 0) {
            this.offset += size - alignment;
        }
    }
    // Reads a given count of numeric values into a typed array.
    typedArray(TypedArrayConstructor, getter, count, alignment = TypedArrayConstructor.BYTES_PER_ELEMENT) {
        if (count === 0) {
            return new TypedArrayConstructor();
        }
        this.align(alignment);
        const totalOffset = this.view.byteOffset + this.offset;
        if (this.littleEndian !== this.hostLittleEndian) {
            // Slowest path
            return this.typedArraySlow(TypedArrayConstructor, getter, count);
        }
        else if (totalOffset % TypedArrayConstructor.BYTES_PER_ELEMENT === 0) {
            // Fastest path
            const array = new TypedArrayConstructor(this.view.buffer, totalOffset, count);
            this.offset += TypedArrayConstructor.BYTES_PER_ELEMENT * count;
            return array;
        }
        else {
            // Slower path
            return this.typedArrayUnaligned(TypedArrayConstructor, getter, count);
        }
    }
    typedArrayUnaligned(TypedArrayConstructor, getter, count) {
        // Benchmarks indicate for count < ~10 doing each individually is faster than copy
        if (count < 10) {
            return this.typedArraySlow(TypedArrayConstructor, getter, count);
        }
        // If the length is > 10, then doing a copy of the data to align it is faster
        // using _set_ is slightly faster than slice on the array buffer according to today's benchmarks
        const byteLength = TypedArrayConstructor.BYTES_PER_ELEMENT * count;
        const copy = new Uint8Array(byteLength);
        copy.set(new Uint8Array(this.view.buffer, this.view.byteOffset + this.offset, byteLength));
        this.offset += byteLength;
        return new TypedArrayConstructor(copy.buffer, copy.byteOffset, count);
    }
    typedArraySlow(TypedArrayConstructor, getter, count) {
        const array = new TypedArrayConstructor(count);
        let offset = this.offset;
        for (let i = 0; i < count; i++) {
            array[i] = this.view[getter](offset, this.littleEndian);
            offset += TypedArrayConstructor.BYTES_PER_ELEMENT;
        }
        this.offset = offset;
        return array;
    }
}

//# sourceMappingURL=CdrReader.js.map