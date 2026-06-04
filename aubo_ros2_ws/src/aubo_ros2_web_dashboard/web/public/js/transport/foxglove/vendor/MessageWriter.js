var __classPrivateFieldSet = (this && this.__classPrivateFieldSet) || function (receiver, state, value, kind, f) {
    if (kind === "m") throw new TypeError("Private method is not writable");
    if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a setter");
    if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot write private member to an object whose class did not declare it");
    return (kind === "a" ? f.call(receiver, value) : f ? f.value = value : state.set(receiver, value)), value;
};
var __classPrivateFieldGet = (this && this.__classPrivateFieldGet) || function (receiver, state, kind, f) {
    if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a getter");
    if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot read private member from an object whose class did not declare it");
    return kind === "m" ? f : kind === "a" ? f.call(receiver) : f ? f.value : state.get(receiver);
};
var _MessageWriter_instances, _MessageWriter_rootDefinition, _MessageWriter_definitions, _MessageWriter_byteSize, _MessageWriter_write, _MessageWriter_getDefinition, _MessageWriter_getPrimitiveSize, _MessageWriter_getPrimitiveWriter, _MessageWriter_getPrimitiveArrayWriter;
import * as cdr_1 from './CdrReader.js';
import * as messageDefinitionHasDataFields_1 from './messageDefinitionHasDataFields.js';
import * as stringLengthUtf8_1 from './stringLengthUtf8.js';
const PRIMITIVE_SIZES = new Map([
    ["bool", 1],
    ["int8", 1],
    ["uint8", 1],
    ["int16", 2],
    ["uint16", 2],
    ["int32", 4],
    ["uint32", 4],
    ["int64", 8],
    ["uint64", 8],
    ["float32", 4],
    ["float64", 8],
    // ["string", ...], // handled separately
    ["time", 8],
    ["duration", 8],
]);
const PRIMITIVE_WRITERS = new Map([
    ["bool", bool],
    ["int8", int8],
    ["uint8", uint8],
    ["int16", int16],
    ["uint16", uint16],
    ["int32", int32],
    ["uint32", uint32],
    ["int64", int64],
    ["uint64", uint64],
    ["float32", float32],
    ["float64", float64],
    ["string", string],
    ["time", time],
    ["duration", time],
    ["wstring", throwOnWstring],
]);
const PRIMITIVE_ARRAY_WRITERS = new Map([
    ["bool", boolArray],
    ["int8", int8Array],
    ["uint8", uint8Array],
    ["int16", int16Array],
    ["uint16", uint16Array],
    ["int32", int32Array],
    ["uint32", uint32Array],
    ["int64", int64Array],
    ["uint64", uint64Array],
    ["float32", float32Array],
    ["float64", float64Array],
    ["string", stringArray],
    ["time", timeArray],
    ["duration", timeArray],
    ["wstring", throwOnWstring],
]);
function throwOnWstring() {
    throw new Error("wstring is implementation-defined and therefore not supported");
}
/**
 * Takes a parsed message definition and returns a message writer which
 * serializes JavaScript objects to CDR-encoded binary.
 */
export class MessageWriter {
    constructor(definitions) {
        _MessageWriter_instances.add(this);
        _MessageWriter_rootDefinition.set(this, void 0);
        _MessageWriter_definitions.set(this, void 0);
        // ros2idl modules could have constant modules before the root struct used to decode message
        const rootDefinition = definitions.find((def) => !isConstantModule(def));
        if (rootDefinition == undefined) {
            throw new Error("MessageReader initialized with no root MessageDefinition");
        }
        __classPrivateFieldSet(this, _MessageWriter_rootDefinition, rootDefinition.definitions, "f");
        __classPrivateFieldSet(this, _MessageWriter_definitions, new Map(definitions.map((def) => [def.name ?? "", def.definitions])), "f");
    }
    /** Calculates the byte size needed to write this message in bytes. */
    calculateByteSize(message) {
        return __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_byteSize).call(this, __classPrivateFieldGet(this, _MessageWriter_rootDefinition, "f"), message, 4);
    }
    /**
     * Serializes a JavaScript object to CDR-encoded binary according to this
     * writer's message definition. If output is provided, it's byte length must
     * be equal or greater to the result of `calculateByteSize(message)`. If not
     * provided, a new Uint8Array will be allocated.
     */
    writeMessage(message, output) {
        const writer = new cdr_1.CdrWriter({
            buffer: output,
            size: output ? undefined : this.calculateByteSize(message),
        });
        __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write).call(this, __classPrivateFieldGet(this, _MessageWriter_rootDefinition, "f"), message, writer);
        return writer.data;
    }
}

_MessageWriter_rootDefinition = new WeakMap(), _MessageWriter_definitions = new WeakMap(), _MessageWriter_instances = new WeakSet(), _MessageWriter_byteSize = function _MessageWriter_byteSize(definition, message, offset) {
    const messageObj = message;
    let newOffset = offset;
    if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
        // In case a message definition definition is empty, ROS 2 adds a
        // `uint8 structure_needs_at_least_one_member` field when converting to IDL,
        // to satisfy the requirement from IDL of not being empty.
        // See also https://design.ros2.org/articles/legacy_interface_definition.html
        return offset + __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveSize).call(this, "uint8");
    }
    for (const field of definition) {
        if (field.isConstant === true) {
            continue;
        }
        const nestedMessage = messageObj?.[field.name];
        if (field.isArray === true) {
            const arrayLength = field.arrayLength ?? fieldLength(nestedMessage);
            const dataIsArray = Array.isArray(nestedMessage) || ArrayBuffer.isView(nestedMessage);
            const dataArray = (dataIsArray ? nestedMessage : []);
            if (field.arrayLength == undefined) {
                // uint32 array length for dynamic arrays
                newOffset += padding(newOffset, 4);
                newOffset += 4;
            }
            if (field.isComplex === true) {
                // Complex type array
                const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
                for (let i = 0; i < arrayLength; i++) {
                    const entry = (dataArray[i] ?? {});
                    newOffset = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_byteSize).call(this, nestedDefinition, entry, newOffset);
                }
            }
            else if (field.type === "string") {
                // String array
                for (let i = 0; i < arrayLength; i++) {
                    const entry = (dataArray[i] ?? "");
                    newOffset += padding(newOffset, 4);
                    newOffset += 4 + (0, stringLengthUtf8_1.stringLengthUtf8)(entry) + 1; // uint32 length prefix, string, null terminator
                }
            }
            else {
                // Primitive array
                const entrySize = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveSize).call(this, field.type);
                const alignment = field.type === "time" || field.type === "duration" ? 4 : entrySize;
                newOffset += padding(newOffset, alignment);
                newOffset += entrySize * arrayLength;
            }
        }
        else {
            if (field.isComplex === true) {
                // Complex type
                const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
                const entry = (nestedMessage ?? {});
                newOffset = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_byteSize).call(this, nestedDefinition, entry, newOffset);
            }
            else if (field.type === "string") {
                // String
                const entry = typeof nestedMessage === "string" ? nestedMessage : "";
                newOffset += padding(newOffset, 4);
                newOffset += 4 + (0, stringLengthUtf8_1.stringLengthUtf8)(entry) + 1; // uint32 length prefix, string, null terminator
            }
            else {
                // Primitive
                const entrySize = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveSize).call(this, field.type);
                const alignment = field.type === "time" || field.type === "duration" ? 4 : entrySize;
                newOffset += padding(newOffset, alignment);
                newOffset += entrySize;
            }
        }
    }
    return newOffset;
}, _MessageWriter_write = function _MessageWriter_write(definition, message, writer) {
    const messageObj = message;
    if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
        // In case a message definition definition is empty, ROS 2 adds a
        // `uint8 structure_needs_at_least_one_member` field when converting to IDL,
        // to satisfy the requirement from IDL of not being empty.
        // See also https://design.ros2.org/articles/legacy_interface_definition.html
        uint8(0, 0, writer);
        return;
    }
    for (const field of definition) {
        if (field.isConstant === true) {
            continue;
        }
        const nestedMessage = messageObj?.[field.name];
        if (field.isArray === true) {
            const arrayLength = field.arrayLength ?? fieldLength(nestedMessage);
            const dataIsArray = Array.isArray(nestedMessage) || ArrayBuffer.isView(nestedMessage);
            const dataArray = (dataIsArray ? nestedMessage : []);
            if (field.arrayLength == undefined) {
                // uint32 array length for dynamic arrays
                writer.sequenceLength(arrayLength);
            }
            if (field.arrayLength != undefined && nestedMessage != undefined) {
                const givenFieldLength = fieldLength(nestedMessage);
                if (givenFieldLength !== field.arrayLength) {
                    throw new Error(`Expected ${field.arrayLength} items for fixed-length array field ${field.name} but received ${givenFieldLength}`);
                }
            }
            if (field.isComplex === true) {
                // Complex type array
                const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
                for (let i = 0; i < arrayLength; i++) {
                    const entry = dataArray[i] ?? {};
                    __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write).call(this, nestedDefinition, entry, writer);
                }
            }
            else {
                // Primitive array
                const arrayWriter = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveArrayWriter).call(this, field.type);
                arrayWriter(nestedMessage, field.defaultValue, writer, field.arrayLength);
            }
        }
        else {
            if (field.isComplex === true) {
                // Complex type
                const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
                const entry = nestedMessage ?? {};
                __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write).call(this, nestedDefinition, entry, writer);
            }
            else {
                // Primitive
                const primitiveWriter = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveWriter).call(this, field.type);
                primitiveWriter(nestedMessage, field.defaultValue, writer);
            }
        }
    }
}, _MessageWriter_getDefinition = function _MessageWriter_getDefinition(datatype) {
    const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_definitions, "f").get(datatype);
    if (nestedDefinition == undefined) {
        throw new Error(`Unrecognized complex type ${datatype}`);
    }
    return nestedDefinition;
}, _MessageWriter_getPrimitiveSize = function _MessageWriter_getPrimitiveSize(primitiveType) {
    const size = PRIMITIVE_SIZES.get(primitiveType);
    if (size == undefined) {
        if (primitiveType === "wstring") {
            throwOnWstring();
        }
        throw new Error(`Unrecognized primitive type ${primitiveType}`);
    }
    return size;
}, _MessageWriter_getPrimitiveWriter = function _MessageWriter_getPrimitiveWriter(primitiveType) {
    const writer = PRIMITIVE_WRITERS.get(primitiveType);
    if (writer == undefined) {
        throw new Error(`Unrecognized primitive type ${primitiveType}`);
    }
    return writer;
}, _MessageWriter_getPrimitiveArrayWriter = function _MessageWriter_getPrimitiveArrayWriter(primitiveType) {
    const writer = PRIMITIVE_ARRAY_WRITERS.get(primitiveType);
    if (writer == undefined) {
        throw new Error(`Unrecognized primitive type ${primitiveType}[]`);
    }
    return writer;
};
function isConstantModule(def) {
    // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
    return def.definitions.length > 0 && def.definitions.every((field) => field.isConstant);
}
function fieldLength(value) {
    const length = value?.length;
    return typeof length === "number" ? length : 0;
}
function bool(value, defaultValue, writer) {
    const boolValue = typeof value === "boolean" ? value : (defaultValue ?? false);
    writer.int8(boolValue ? 1 : 0);
}
function int8(value, defaultValue, writer) {
    writer.int8(typeof value === "number" ? value : (defaultValue ?? 0));
}
function uint8(value, defaultValue, writer) {
    writer.uint8(typeof value === "number" ? value : (defaultValue ?? 0));
}
function int16(value, defaultValue, writer) {
    writer.int16(typeof value === "number" ? value : (defaultValue ?? 0));
}
function uint16(value, defaultValue, writer) {
    writer.uint16(typeof value === "number" ? value : (defaultValue ?? 0));
}
function int32(value, defaultValue, writer) {
    writer.int32(typeof value === "number" ? value : (defaultValue ?? 0));
}
function uint32(value, defaultValue, writer) {
    writer.uint32(typeof value === "number" ? value : (defaultValue ?? 0));
}
function int64(value, defaultValue, writer) {
    if (typeof value === "bigint") {
        writer.int64(value);
    }
    else if (typeof value === "number") {
        writer.int64(BigInt(value));
    }
    else {
        writer.int64((defaultValue ?? 0n));
    }
}
function uint64(value, defaultValue, writer) {
    if (typeof value === "bigint") {
        writer.uint64(value);
    }
    else if (typeof value === "number") {
        writer.uint64(BigInt(value));
    }
    else {
        writer.uint64((defaultValue ?? 0n));
    }
}
function float32(value, defaultValue, writer) {
    writer.float32(typeof value === "number" ? value : (defaultValue ?? 0));
}
function float64(value, defaultValue, writer) {
    writer.float64(typeof value === "number" ? value : (defaultValue ?? 0));
}
function string(value, defaultValue, writer) {
    writer.string(typeof value === "string" ? value : (defaultValue ?? ""));
}
function time(value, _defaultValue, writer) {
    if (value == undefined) {
        writer.int32(0);
        writer.uint32(0);
        return;
    }
    const timeObj = value;
    writer.int32(timeObj.sec ?? 0);
    writer.uint32(timeObj.nsec ?? timeObj.nanosec ?? 0);
}
function boolArray(value, defaultValue, writer, arrayLength) {
    if (Array.isArray(value)) {
        const array = new Int8Array(value);
        writer.int8Array(array);
    }
    else {
        writer.int8Array((defaultValue ?? new Int8Array(arrayLength ?? 0).fill(0)));
    }
}
function int8Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Int8Array) {
        writer.int8Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Int8Array(value);
        writer.int8Array(array);
    }
    else {
        writer.int8Array((defaultValue ?? new Int8Array(arrayLength ?? 0).fill(0)));
    }
}
function uint8Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Uint8Array) {
        writer.uint8Array(value);
    }
    else if (value instanceof Uint8ClampedArray) {
        writer.uint8Array(new Uint8Array(value));
    }
    else if (Array.isArray(value)) {
        const array = new Uint8Array(value);
        writer.uint8Array(array);
    }
    else {
        writer.uint8Array((defaultValue ?? new Uint8Array(arrayLength ?? 0).fill(0)));
    }
}
function int16Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Int16Array) {
        writer.int16Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Int16Array(value);
        writer.int16Array(array);
    }
    else {
        writer.int16Array((defaultValue ?? new Int16Array(arrayLength ?? 0).fill(0)));
    }
}
function uint16Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Uint16Array) {
        writer.uint16Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Uint16Array(value);
        writer.uint16Array(array);
    }
    else {
        writer.uint16Array((defaultValue ?? new Uint16Array(arrayLength ?? 0).fill(0)));
    }
}
function int32Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Int32Array) {
        writer.int32Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Int32Array(value);
        writer.int32Array(array);
    }
    else {
        writer.int32Array((defaultValue ?? new Int32Array(arrayLength ?? 0).fill(0)));
    }
}
function uint32Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Uint32Array) {
        writer.uint32Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Uint32Array(value);
        writer.uint32Array(array);
    }
    else {
        writer.uint32Array((defaultValue ?? new Uint32Array(arrayLength ?? 0).fill(0)));
    }
}
function int64Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof BigInt64Array) {
        writer.int64Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new BigInt64Array(value);
        writer.int64Array(array);
    }
    else {
        writer.int64Array((defaultValue ?? new BigInt64Array(arrayLength ?? 0).fill(0n)));
    }
}
function uint64Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof BigUint64Array) {
        writer.uint64Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new BigUint64Array(value);
        writer.uint64Array(array);
    }
    else {
        writer.uint64Array((defaultValue ?? new BigUint64Array(arrayLength ?? 0).fill(0n)));
    }
}
function float32Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Float32Array) {
        writer.float32Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Float32Array(value);
        writer.float32Array(array);
    }
    else {
        writer.float32Array((defaultValue ?? new Float32Array(arrayLength ?? 0).fill(0)));
    }
}
function float64Array(value, defaultValue, writer, arrayLength) {
    if (value instanceof Float64Array) {
        writer.float64Array(value);
    }
    else if (Array.isArray(value)) {
        const array = new Float64Array(value);
        writer.float64Array(array);
    }
    else {
        writer.float64Array((defaultValue ?? new Float64Array(arrayLength ?? 0).fill(0)));
    }
}
function stringArray(value, defaultValue, writer, arrayLength) {
    if (Array.isArray(value)) {
        for (const item of value) {
            writer.string(typeof item === "string" ? item : "");
        }
    }
    else {
        const array = (defaultValue ?? new Array(arrayLength ?? 0).fill(""));
        for (const item of array) {
            writer.string(item);
        }
    }
}
function timeArray(value, _defaultValue, writer, arrayLength) {
    if (Array.isArray(value)) {
        for (const item of value) {
            time(item, undefined, writer);
        }
    }
    else {
        const array = new Array(arrayLength).fill(undefined);
        for (const item of array) {
            time(item, undefined, writer);
        }
    }
}
function padding(offset, byteWidth) {
    // The four byte header is not considered for alignment
    const alignment = (offset - 4) % byteWidth;
    return alignment > 0 ? byteWidth - alignment : 0;
}
//# sourceMappingURL=MessageWriter.js.map