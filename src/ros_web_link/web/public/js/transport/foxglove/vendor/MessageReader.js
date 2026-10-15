var __classPrivateFieldGet = (this && this.__classPrivateFieldGet) || function (receiver, state, kind, f) {
    if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a getter");
    if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot read private member from an object whose class did not declare it");
    return kind === "m" ? f : kind === "a" ? f.call(receiver) : f ? f.value : state.get(receiver);
};
var __classPrivateFieldSet = (this && this.__classPrivateFieldSet) || function (receiver, state, value, kind, f) {
    if (kind === "m") throw new TypeError("Private method is not writable");
    if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a setter");
    if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot write private member to an object whose class did not declare it");
    return (kind === "a" ? f.call(receiver, value) : f ? f.value = value : state.set(receiver, value)), value;
};
var _MessageReader_instances, _MessageReader_rootDefinition, _MessageReader_definitions, _MessageReader_lastReadByteLength, _MessageReader_lastReadHadTrailingBytes, _MessageReader_useRos1Time, _MessageReader_readComplexType;
import * as cdr_1 from './CdrReader.js';
import * as messageDefinitionHasDataFields_1 from './messageDefinitionHasDataFields.js';
export class MessageReader {
    /**
     * True when the most recent decode finished before reaching the end of the buffer, excluding CDR
     * final padding. CDR ignores trailing bytes by design, so this can signal a schema/payload
     * version mismatch.
     */
    lastReadHadTrailingBytes() {
        return __classPrivateFieldGet(this, _MessageReader_lastReadHadTrailingBytes, "f");
    }
    /**
     * Number of bytes consumed by the most recent decode.
     */
    lastReadByteLength() {
        return __classPrivateFieldGet(this, _MessageReader_lastReadByteLength, "f");
    }
    constructor(definitions, options = {}) {
        _MessageReader_instances.add(this);
        _MessageReader_rootDefinition.set(this, void 0);
        _MessageReader_definitions.set(this, void 0);
        _MessageReader_lastReadByteLength.set(this, 0);
        _MessageReader_lastReadHadTrailingBytes.set(this, false);
        _MessageReader_useRos1Time.set(this, void 0);
        const { timeType = "sec,nanosec" } = options;
        // ros2idl modules could have constant modules before the root struct used to decode message
        const rootDefinition = definitions.find((def) => !isConstantModule(def));
        if (rootDefinition == undefined) {
            throw new Error("MessageReader initialized with no root MessageDefinition");
        }
        __classPrivateFieldSet(this, _MessageReader_rootDefinition, rootDefinition.definitions, "f");
        __classPrivateFieldSet(this, _MessageReader_definitions, new Map(definitions.map((def) => [def.name ?? "", def.definitions])), "f");
        __classPrivateFieldSet(this, _MessageReader_useRos1Time, timeType === "sec,nsec", "f");
    }
    // We template on R here for call site type information if the class type information T is not
    // known or available
    // eslint-disable-next-line @typescript-eslint/no-unnecessary-type-parameters
    readMessage(buffer) {
        const reader = new cdr_1.CdrReader(buffer);
        const value = __classPrivateFieldGet(this, _MessageReader_instances, "m", _MessageReader_readComplexType).call(this, __classPrivateFieldGet(this, _MessageReader_rootDefinition, "f"), reader);
        __classPrivateFieldSet(this, _MessageReader_lastReadByteLength, reader.decodedBytes, "f");
        __classPrivateFieldSet(this, _MessageReader_lastReadHadTrailingBytes, __classPrivateFieldGet(this, _MessageReader_lastReadByteLength, "f") < buffer.byteLength &&
            !isCdrFinalPadding(buffer, __classPrivateFieldGet(this, _MessageReader_lastReadByteLength, "f")), "f");
        return value;
    }
}

_MessageReader_rootDefinition = new WeakMap(), _MessageReader_definitions = new WeakMap(), _MessageReader_lastReadByteLength = new WeakMap(), _MessageReader_lastReadHadTrailingBytes = new WeakMap(), _MessageReader_useRos1Time = new WeakMap(), _MessageReader_instances = new WeakSet(), _MessageReader_readComplexType = function _MessageReader_readComplexType(definition, reader) {
    const msg = {};
    if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
        // In case a message definition definition is empty, ROS 2 adds a
        // `uint8 structure_needs_at_least_one_member` field when converting to IDL,
        // to satisfy the requirement from IDL of not being empty.
        // See also https://design.ros2.org/articles/legacy_interface_definition.html
        reader.uint8();
        return msg;
    }
    for (const field of definition) {
        if (field.isConstant === true) {
            continue;
        }
        if (field.isComplex === true) {
            // Complex type
            const nestedDefinition = __classPrivateFieldGet(this, _MessageReader_definitions, "f").get(field.type);
            if (nestedDefinition == undefined) {
                throw new Error(`Unrecognized complex type ${field.type}`);
            }
            if (field.isArray === true) {
                // For dynamic length arrays we need to read a uint32 prefix
                const arrayLength = field.arrayLength ?? reader.sequenceLength();
                const array = [];
                for (let i = 0; i < arrayLength; i++) {
                    array.push(__classPrivateFieldGet(this, _MessageReader_instances, "m", _MessageReader_readComplexType).call(this, nestedDefinition, reader));
                }
                msg[field.name] = array;
            }
            else {
                msg[field.name] = __classPrivateFieldGet(this, _MessageReader_instances, "m", _MessageReader_readComplexType).call(this, nestedDefinition, reader);
            }
        }
        else {
            // Primitive type
            if (field.isArray === true) {
                const deser = (__classPrivateFieldGet(this, _MessageReader_useRos1Time, "f") ? ros1TypedArrayDeserializers : typedArrayDeserializers).get(field.type);
                if (deser == undefined) {
                    throw new Error(`Unrecognized primitive array type ${field.type}[]`);
                }
                // For dynamic length arrays we need to read a uint32 prefix
                const arrayLength = field.arrayLength ?? reader.sequenceLength();
                msg[field.name] = deser(reader, arrayLength);
            }
            else {
                const deser = (__classPrivateFieldGet(this, _MessageReader_useRos1Time, "f") ? ros1TimeDeserializers : deserializers).get(field.type);
                if (deser == undefined) {
                    throw new Error(`Unrecognized primitive type ${field.type}`);
                }
                msg[field.name] = deser(reader);
            }
        }
    }
    return msg;
};
function isConstantModule(def) {
    // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
    return def.definitions.length > 0 && def.definitions.every((field) => field.isConstant);
}
function isCdrFinalPadding(buffer, decodedBytes) {
    const trailingByteLength = buffer.byteLength - decodedBytes;
    if (trailingByteLength <= 0) {
        return false;
    }
    // The 4-byte branch is load-bearing: standard CDR1 final padding aligns the data section to a
    // 4-byte boundary, and since the encapsulation header is itself 4 bytes, buffer-relative and
    // data-relative alignment agree. The 8-byte branch is a buffer-relative best-effort for
    // publishers that pad to a wider boundary.
    const paddingToFourByteBoundary = (4 - (decodedBytes % 4)) % 4;
    const paddingToEightByteBoundary = (8 - (decodedBytes % 8)) % 8;
    if (trailingByteLength !== paddingToFourByteBoundary &&
        trailingByteLength !== paddingToEightByteBoundary) {
        return false;
    }
    const trailingBytes = new Uint8Array(buffer.buffer, buffer.byteOffset + decodedBytes, trailingByteLength);
    return trailingBytes.every((byte) => byte === 0);
}
const deserializers = new Map([
    ["bool", (reader) => Boolean(reader.int8())],
    ["int8", (reader) => reader.int8()],
    ["uint8", (reader) => reader.uint8()],
    ["int16", (reader) => reader.int16()],
    ["uint16", (reader) => reader.uint16()],
    ["int32", (reader) => reader.int32()],
    ["uint32", (reader) => reader.uint32()],
    ["int64", (reader) => reader.int64()],
    ["uint64", (reader) => reader.uint64()],
    ["float32", (reader) => reader.float32()],
    ["float64", (reader) => reader.float64()],
    ["string", (reader) => reader.string()],
    ["wstring", throwOnWstring],
    ["time", (reader) => ({ sec: reader.int32(), nanosec: reader.uint32() })],
    ["duration", (reader) => ({ sec: reader.int32(), nanosec: reader.uint32() })],
]);
const ros1TimeDeserializers = new Map([
    ...deserializers,
    ["time", (reader) => ({ sec: reader.int32(), nsec: reader.uint32() })],
    ["duration", (reader) => ({ sec: reader.int32(), nsec: reader.uint32() })],
]);
const typedArrayDeserializers = new Map([
    ["bool", readBoolArray],
    ["int8", (reader, count) => reader.int8Array(count)],
    ["uint8", (reader, count) => reader.uint8Array(count)],
    ["int16", (reader, count) => reader.int16Array(count)],
    ["uint16", (reader, count) => reader.uint16Array(count)],
    ["int32", (reader, count) => reader.int32Array(count)],
    ["uint32", (reader, count) => reader.uint32Array(count)],
    ["int64", (reader, count) => reader.int64Array(count)],
    ["uint64", (reader, count) => reader.uint64Array(count)],
    ["float32", (reader, count) => reader.float32Array(count)],
    ["float64", (reader, count) => reader.float64Array(count)],
    ["string", readStringArray],
    ["wstring", throwOnWstring],
    ["time", readTimeArray],
    ["duration", readTimeArray],
]);
const ros1TypedArrayDeserializers = new Map([
    ...typedArrayDeserializers,
    ["time", readRos1TimeArray],
    ["duration", readRos1TimeArray],
]);
function readBoolArray(reader, count) {
    const array = new Array(count);
    for (let i = 0; i < count; i++) {
        array[i] = Boolean(reader.int8());
    }
    return array;
}
function readStringArray(reader, count) {
    const array = new Array(count);
    for (let i = 0; i < count; i++) {
        array[i] = reader.string();
    }
    return array;
}
function readRos1TimeArray(reader, count) {
    const array = new Array(count);
    for (let i = 0; i < count; i++) {
        const sec = reader.int32();
        const nsec = reader.uint32();
        array[i] = { sec, nsec };
    }
    return array;
}
function readTimeArray(reader, count) {
    const array = new Array(count);
    for (let i = 0; i < count; i++) {
        const sec = reader.int32();
        const nanosec = reader.uint32();
        array[i] = { sec, nanosec };
    }
    return array;
}
function throwOnWstring() {
    throw new Error("wstring is implementation-defined and therefore not supported");
}
//# sourceMappingURL=MessageReader.js.map