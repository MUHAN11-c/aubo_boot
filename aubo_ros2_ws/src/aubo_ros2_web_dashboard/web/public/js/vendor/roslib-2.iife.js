(() => {
  var __create = Object.create;
  var __defProp = Object.defineProperty;
  var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
  var __getOwnPropNames = Object.getOwnPropertyNames;
  var __getProtoOf = Object.getPrototypeOf;
  var __hasOwnProp = Object.prototype.hasOwnProperty;
  var __esm = (fn, res) => function __init() {
    return fn && (res = (0, fn[__getOwnPropNames(fn)[0]])(fn = 0)), res;
  };
  var __commonJS = (cb, mod) => function __require() {
    return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
  };
  var __export = (target, all) => {
    for (var name in all)
      __defProp(target, name, { get: all[name], enumerable: true });
  };
  var __copyProps = (to, from, except, desc) => {
    if (from && typeof from === "object" || typeof from === "function") {
      for (let key of __getOwnPropNames(from))
        if (!__hasOwnProp.call(to, key) && key !== except)
          __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
    }
    return to;
  };
  var __toESM = (mod, isNodeMode, target) => (target = mod != null ? __create(__getProtoOf(mod)) : {}, __copyProps(
    // If the importer is in node compatibility mode or this is not an ESM
    // file that has been converted to a CommonJS file using a Babel-
    // compatible transform (i.e. "__esModule" has not been set), then set
    // "default" to the CommonJS "module.exports" for node compatibility.
    isNodeMode || !mod || !mod.__esModule ? __defProp(target, "default", { value: mod, enumerable: true }) : target,
    mod
  ));

  // node_modules/eventemitter3/index.js
  var require_eventemitter3 = __commonJS({
    "node_modules/eventemitter3/index.js"(exports, module) {
      "use strict";
      var has = Object.prototype.hasOwnProperty;
      var prefix = "~";
      function Events() {
      }
      if (Object.create) {
        Events.prototype = /* @__PURE__ */ Object.create(null);
        if (!new Events().__proto__) prefix = false;
      }
      function EE(fn, context, once) {
        this.fn = fn;
        this.context = context;
        this.once = once || false;
      }
      function addListener(emitter, event, fn, context, once) {
        if (typeof fn !== "function") {
          throw new TypeError("The listener must be a function");
        }
        var listener = new EE(fn, context || emitter, once), evt = prefix ? prefix + event : event;
        if (!emitter._events[evt]) emitter._events[evt] = listener, emitter._eventsCount++;
        else if (!emitter._events[evt].fn) emitter._events[evt].push(listener);
        else emitter._events[evt] = [emitter._events[evt], listener];
        return emitter;
      }
      function clearEvent(emitter, evt) {
        if (--emitter._eventsCount === 0) emitter._events = new Events();
        else delete emitter._events[evt];
      }
      function EventEmitter2() {
        this._events = new Events();
        this._eventsCount = 0;
      }
      EventEmitter2.prototype.eventNames = function eventNames() {
        var names = [], events, name;
        if (this._eventsCount === 0) return names;
        for (name in events = this._events) {
          if (has.call(events, name)) names.push(prefix ? name.slice(1) : name);
        }
        if (Object.getOwnPropertySymbols) {
          return names.concat(Object.getOwnPropertySymbols(events));
        }
        return names;
      };
      EventEmitter2.prototype.listeners = function listeners(event) {
        var evt = prefix ? prefix + event : event, handlers = this._events[evt];
        if (!handlers) return [];
        if (handlers.fn) return [handlers.fn];
        for (var i2 = 0, l6 = handlers.length, ee = new Array(l6); i2 < l6; i2++) {
          ee[i2] = handlers[i2].fn;
        }
        return ee;
      };
      EventEmitter2.prototype.listenerCount = function listenerCount(event) {
        var evt = prefix ? prefix + event : event, listeners = this._events[evt];
        if (!listeners) return 0;
        if (listeners.fn) return 1;
        return listeners.length;
      };
      EventEmitter2.prototype.emit = function emit(event, a1, a22, a32, a4, a5) {
        var evt = prefix ? prefix + event : event;
        if (!this._events[evt]) return false;
        var listeners = this._events[evt], len = arguments.length, args, i2;
        if (listeners.fn) {
          if (listeners.once) this.removeListener(event, listeners.fn, void 0, true);
          switch (len) {
            case 1:
              return listeners.fn.call(listeners.context), true;
            case 2:
              return listeners.fn.call(listeners.context, a1), true;
            case 3:
              return listeners.fn.call(listeners.context, a1, a22), true;
            case 4:
              return listeners.fn.call(listeners.context, a1, a22, a32), true;
            case 5:
              return listeners.fn.call(listeners.context, a1, a22, a32, a4), true;
            case 6:
              return listeners.fn.call(listeners.context, a1, a22, a32, a4, a5), true;
          }
          for (i2 = 1, args = new Array(len - 1); i2 < len; i2++) {
            args[i2 - 1] = arguments[i2];
          }
          listeners.fn.apply(listeners.context, args);
        } else {
          var length = listeners.length, j2;
          for (i2 = 0; i2 < length; i2++) {
            if (listeners[i2].once) this.removeListener(event, listeners[i2].fn, void 0, true);
            switch (len) {
              case 1:
                listeners[i2].fn.call(listeners[i2].context);
                break;
              case 2:
                listeners[i2].fn.call(listeners[i2].context, a1);
                break;
              case 3:
                listeners[i2].fn.call(listeners[i2].context, a1, a22);
                break;
              case 4:
                listeners[i2].fn.call(listeners[i2].context, a1, a22, a32);
                break;
              default:
                if (!args) for (j2 = 1, args = new Array(len - 1); j2 < len; j2++) {
                  args[j2 - 1] = arguments[j2];
                }
                listeners[i2].fn.apply(listeners[i2].context, args);
            }
          }
        }
        return true;
      };
      EventEmitter2.prototype.on = function on(event, fn, context) {
        return addListener(this, event, fn, context, false);
      };
      EventEmitter2.prototype.once = function once(event, fn, context) {
        return addListener(this, event, fn, context, true);
      };
      EventEmitter2.prototype.removeListener = function removeListener(event, fn, context, once) {
        var evt = prefix ? prefix + event : event;
        if (!this._events[evt]) return this;
        if (!fn) {
          clearEvent(this, evt);
          return this;
        }
        var listeners = this._events[evt];
        if (listeners.fn) {
          if (listeners.fn === fn && (!once || listeners.once) && (!context || listeners.context === context)) {
            clearEvent(this, evt);
          }
        } else {
          for (var i2 = 0, events = [], length = listeners.length; i2 < length; i2++) {
            if (listeners[i2].fn !== fn || once && !listeners[i2].once || context && listeners[i2].context !== context) {
              events.push(listeners[i2]);
            }
          }
          if (events.length) this._events[evt] = events.length === 1 ? events[0] : events;
          else clearEvent(this, evt);
        }
        return this;
      };
      EventEmitter2.prototype.removeAllListeners = function removeAllListeners(event) {
        var evt;
        if (event) {
          evt = prefix ? prefix + event : event;
          if (this._events[evt]) clearEvent(this, evt);
        } else {
          this._events = new Events();
          this._eventsCount = 0;
        }
        return this;
      };
      EventEmitter2.prototype.off = EventEmitter2.prototype.removeListener;
      EventEmitter2.prototype.addListener = EventEmitter2.prototype.on;
      EventEmitter2.prefixed = prefix;
      EventEmitter2.EventEmitter = EventEmitter2;
      if ("undefined" !== typeof module) {
        module.exports = EventEmitter2;
      }
    }
  });

  // node_modules/eventemitter3/index.mjs
  var import_index;
  var init_eventemitter3 = __esm({
    "node_modules/eventemitter3/index.mjs"() {
      import_index = __toESM(require_eventemitter3(), 1);
    }
  });

  // node_modules/uuid/dist/stringify.js
  function unsafeStringify(arr, offset = 0) {
    return (byteToHex[arr[offset + 0]] + byteToHex[arr[offset + 1]] + byteToHex[arr[offset + 2]] + byteToHex[arr[offset + 3]] + "-" + byteToHex[arr[offset + 4]] + byteToHex[arr[offset + 5]] + "-" + byteToHex[arr[offset + 6]] + byteToHex[arr[offset + 7]] + "-" + byteToHex[arr[offset + 8]] + byteToHex[arr[offset + 9]] + "-" + byteToHex[arr[offset + 10]] + byteToHex[arr[offset + 11]] + byteToHex[arr[offset + 12]] + byteToHex[arr[offset + 13]] + byteToHex[arr[offset + 14]] + byteToHex[arr[offset + 15]]).toLowerCase();
  }
  var byteToHex;
  var init_stringify = __esm({
    "node_modules/uuid/dist/stringify.js"() {
      byteToHex = [];
      for (let i2 = 0; i2 < 256; ++i2) {
        byteToHex.push((i2 + 256).toString(16).slice(1));
      }
    }
  });

  // node_modules/uuid/dist/rng.js
  function rng() {
    if (!getRandomValues) {
      if (typeof crypto === "undefined" || !crypto.getRandomValues) {
        throw new Error("crypto.getRandomValues() not supported. See https://github.com/uuidjs/uuid#getrandomvalues-not-supported");
      }
      getRandomValues = crypto.getRandomValues.bind(crypto);
    }
    return getRandomValues(rnds8);
  }
  var getRandomValues, rnds8;
  var init_rng = __esm({
    "node_modules/uuid/dist/rng.js"() {
      rnds8 = new Uint8Array(16);
    }
  });

  // node_modules/uuid/dist/native.js
  var randomUUID, native_default;
  var init_native = __esm({
    "node_modules/uuid/dist/native.js"() {
      randomUUID = typeof crypto !== "undefined" && crypto.randomUUID && crypto.randomUUID.bind(crypto);
      native_default = { randomUUID };
    }
  });

  // node_modules/uuid/dist/v4.js
  function _v4(options, buf, offset) {
    options = options || {};
    const rnds = options.random ?? options.rng?.() ?? rng();
    if (rnds.length < 16) {
      throw new Error("Random bytes length must be >= 16");
    }
    rnds[6] = rnds[6] & 15 | 64;
    rnds[8] = rnds[8] & 63 | 128;
    if (buf) {
      offset = offset || 0;
      if (offset < 0 || offset + 16 > buf.length) {
        throw new RangeError(`UUID byte range ${offset}:${offset + 15} is out of buffer bounds`);
      }
      for (let i2 = 0; i2 < 16; ++i2) {
        buf[offset + i2] = rnds[i2];
      }
      return buf;
    }
    return unsafeStringify(rnds);
  }
  function v4(options, buf, offset) {
    if (native_default.randomUUID && !buf && !options) {
      return native_default.randomUUID();
    }
    return _v4(options, buf, offset);
  }
  var v4_default;
  var init_v4 = __esm({
    "node_modules/uuid/dist/v4.js"() {
      init_native();
      init_rng();
      init_stringify();
      v4_default = v4;
    }
  });

  // node_modules/uuid/dist/index.js
  var init_dist = __esm({
    "node_modules/uuid/dist/index.js"() {
      init_v4();
    }
  });

  // node_modules/bson/lib/bson.mjs
  function isUint8Array(value) {
    return TypedArrayPrototypeGetSymbolToStringTag(value) === "Uint8Array";
  }
  function isAnyArrayBuffer(value) {
    return typeof value === "object" && value != null && Symbol.toStringTag in value && (value[Symbol.toStringTag] === "ArrayBuffer" || value[Symbol.toStringTag] === "SharedArrayBuffer");
  }
  function isRegExp(regexp) {
    return regexp instanceof RegExp || Object.prototype.toString.call(regexp) === "[object RegExp]";
  }
  function isMap(value) {
    return typeof value === "object" && value != null && Symbol.toStringTag in value && value[Symbol.toStringTag] === "Map";
  }
  function isDate(date) {
    return date instanceof Date || Object.prototype.toString.call(date) === "[object Date]";
  }
  function defaultInspect(x5, _options) {
    return JSON.stringify(x5, (k4, v3) => {
      if (typeof v3 === "bigint") {
        return { $numberLong: `${v3}` };
      } else if (isMap(v3)) {
        return Object.fromEntries(v3);
      }
      return v3;
    });
  }
  function getStylizeFunction(options) {
    const stylizeExists = options != null && typeof options === "object" && "stylize" in options && typeof options.stylize === "function";
    if (stylizeExists) {
      return options.stylize;
    }
  }
  function parseUtf8(buffer2, start, end, fatal) {
    if (fatal) {
      TextDecoderFatal ??= new TextDecoder("utf8", { fatal: true });
      try {
        return TextDecoderFatal.decode(buffer2.subarray(start, end));
      } catch (cause) {
        throw new BSONError("Invalid UTF-8 string in BSON document", { cause });
      }
    }
    TextDecoderNonFatal ??= new TextDecoder("utf8", { fatal: false });
    return TextDecoderNonFatal.decode(buffer2.subarray(start, end));
  }
  function tryReadBasicLatin(uint8array, start, end) {
    if (uint8array.length === 0) {
      return "";
    }
    const stringByteLength = end - start;
    if (stringByteLength === 0) {
      return "";
    }
    if (stringByteLength > 20) {
      return null;
    }
    if (stringByteLength === 1 && uint8array[start] < 128) {
      return String.fromCharCode(uint8array[start]);
    }
    if (stringByteLength === 2 && uint8array[start] < 128 && uint8array[start + 1] < 128) {
      return String.fromCharCode(uint8array[start]) + String.fromCharCode(uint8array[start + 1]);
    }
    if (stringByteLength === 3 && uint8array[start] < 128 && uint8array[start + 1] < 128 && uint8array[start + 2] < 128) {
      return String.fromCharCode(uint8array[start]) + String.fromCharCode(uint8array[start + 1]) + String.fromCharCode(uint8array[start + 2]);
    }
    const latinBytes = [];
    for (let i2 = start; i2 < end; i2++) {
      const byte = uint8array[i2];
      if (byte > 127) {
        return null;
      }
      latinBytes.push(byte);
    }
    return String.fromCharCode(...latinBytes);
  }
  function tryWriteBasicLatin(destination, source, offset) {
    if (source.length === 0)
      return 0;
    if (source.length > 25)
      return null;
    if (destination.length - offset < source.length)
      return null;
    for (let charOffset = 0, destinationOffset = offset; charOffset < source.length; charOffset++, destinationOffset++) {
      const char = source.charCodeAt(charOffset);
      if (char > 127)
        return null;
      destination[destinationOffset] = char;
    }
    return source.length;
  }
  function nodejsMathRandomBytes(byteLength) {
    return nodeJsByteUtils.fromNumberArray(Array.from({ length: byteLength }, () => Math.floor(Math.random() * 256)));
  }
  function nodejsSecureRandomBytes(byteLength) {
    return crypto.getRandomValues(nodeJsByteUtils.allocate(byteLength));
  }
  function isReactNative() {
    const { navigator } = globalThis;
    return typeof navigator === "object" && navigator.product === "ReactNative";
  }
  function webMathRandomBytes(byteLength) {
    if (byteLength < 0) {
      throw new RangeError(`The argument 'byteLength' is invalid. Received ${byteLength}`);
    }
    return webByteUtils.fromNumberArray(Array.from({ length: byteLength }, () => Math.floor(Math.random() * 256)));
  }
  function validateBinaryVector(vector) {
    if (vector.sub_type !== Binary.SUBTYPE_VECTOR)
      return;
    const size = vector.position;
    const datatype = vector.buffer[0];
    const padding = vector.buffer[1];
    if ((datatype === Binary.VECTOR_TYPE.Float32 || datatype === Binary.VECTOR_TYPE.Int8) && padding !== 0) {
      throw new BSONError("Invalid Vector: padding must be zero for int8 and float32 vectors");
    }
    if (datatype === Binary.VECTOR_TYPE.Float32) {
      if (size !== 0 && size - 2 !== 0 && (size - 2) % 4 !== 0) {
        throw new BSONError("Invalid Vector: Float32 vector must contain a multiple of 4 bytes");
      }
    }
    if (datatype === Binary.VECTOR_TYPE.PackedBit && padding !== 0 && size === 2) {
      throw new BSONError("Invalid Vector: padding must be zero for packed bit vectors that are empty");
    }
    if (datatype === Binary.VECTOR_TYPE.PackedBit && padding > 7) {
      throw new BSONError(`Invalid Vector: padding must be a value between 0 and 7. found: ${padding}`);
    }
  }
  function isDBRefLike(value) {
    return value != null && typeof value === "object" && "$id" in value && value.$id != null && "$ref" in value && typeof value.$ref === "string" && (!("$db" in value) || "$db" in value && typeof value.$db === "string");
  }
  function removeLeadingZerosAndExplicitPlus(str) {
    if (str === "") {
      return str;
    }
    let startIndex = 0;
    const isNegative = str[startIndex] === "-";
    const isExplicitlyPositive = str[startIndex] === "+";
    if (isExplicitlyPositive || isNegative) {
      startIndex += 1;
    }
    let foundInsignificantZero = false;
    for (; startIndex < str.length && str[startIndex] === "0"; ++startIndex) {
      foundInsignificantZero = true;
    }
    if (!foundInsignificantZero) {
      return isExplicitlyPositive ? str.slice(1) : str;
    }
    return `${isNegative ? "-" : ""}${str.length === startIndex ? "0" : str.slice(startIndex)}`;
  }
  function validateStringCharacters(str, radix) {
    radix = radix ?? 10;
    const validCharacters = "0123456789abcdefghijklmnopqrstuvwxyz".slice(0, radix);
    const regex = new RegExp(`[^-+${validCharacters}]`, "i");
    return regex.test(str) ? false : str;
  }
  function isDigit(value) {
    return !isNaN(parseInt(value, 10));
  }
  function divideu128(value) {
    const DIVISOR = Long.fromNumber(1e3 * 1e3 * 1e3);
    let _rem = Long.fromNumber(0);
    if (!value.parts[0] && !value.parts[1] && !value.parts[2] && !value.parts[3]) {
      return { quotient: value, rem: _rem };
    }
    for (let i2 = 0; i2 <= 3; i2++) {
      _rem = _rem.shiftLeft(32);
      _rem = _rem.add(new Long(value.parts[i2], 0));
      value.parts[i2] = _rem.div(DIVISOR).low;
      _rem = _rem.modulo(DIVISOR);
    }
    return { quotient: value, rem: _rem };
  }
  function multiply64x2(left, right) {
    if (!left && !right) {
      return { high: Long.fromNumber(0), low: Long.fromNumber(0) };
    }
    const leftHigh = left.shiftRightUnsigned(32);
    const leftLow = new Long(left.getLowBits(), 0);
    const rightHigh = right.shiftRightUnsigned(32);
    const rightLow = new Long(right.getLowBits(), 0);
    let productHigh = leftHigh.multiply(rightHigh);
    let productMid = leftHigh.multiply(rightLow);
    const productMid2 = leftLow.multiply(rightHigh);
    let productLow = leftLow.multiply(rightLow);
    productHigh = productHigh.add(productMid.shiftRightUnsigned(32));
    productMid = new Long(productMid.getLowBits(), 0).add(productMid2).add(productLow.shiftRightUnsigned(32));
    productHigh = productHigh.add(productMid.shiftRightUnsigned(32));
    productLow = productMid.shiftLeft(32).add(new Long(productLow.getLowBits(), 0));
    return { high: productHigh, low: productLow };
  }
  function lessThan(left, right) {
    const uhleft = left.high >>> 0;
    const uhright = right.high >>> 0;
    if (uhleft < uhright) {
      return true;
    } else if (uhleft === uhright) {
      const ulleft = left.low >>> 0;
      const ulright = right.low >>> 0;
      if (ulleft < ulright)
        return true;
    }
    return false;
  }
  function invalidErr(string, message) {
    throw new BSONError(`"${string}" is not a valid Decimal128 string - ${message}`);
  }
  function alphabetize(str) {
    return str.split("").sort().join("");
  }
  function internalDeserialize(buffer2, options, isArray) {
    options = options == null ? {} : options;
    const index = options && options.index ? options.index : 0;
    const size = NumberUtils.getInt32LE(buffer2, index);
    if (size < 5) {
      throw new BSONError(`bson size must be >= 5, is ${size}`);
    }
    if (options.allowObjectSmallerThanBufferSize && buffer2.length < size) {
      throw new BSONError(`buffer length ${buffer2.length} must be >= bson size ${size}`);
    }
    if (!options.allowObjectSmallerThanBufferSize && buffer2.length !== size) {
      throw new BSONError(`buffer length ${buffer2.length} must === bson size ${size}`);
    }
    if (size + index > buffer2.byteLength) {
      throw new BSONError(`(bson size ${size} + options.index ${index} must be <= buffer length ${buffer2.byteLength})`);
    }
    if (buffer2[index + size - 1] !== 0) {
      throw new BSONError("One object, sized correctly, with a spot for an EOO, but the EOO isn't 0x00");
    }
    return deserializeObject(buffer2, index, options, isArray);
  }
  function deserializeObject(buffer2, index, options, isArray = false) {
    const fieldsAsRaw = options["fieldsAsRaw"] == null ? null : options["fieldsAsRaw"];
    const raw = options["raw"] == null ? false : options["raw"];
    const bsonRegExp = typeof options["bsonRegExp"] === "boolean" ? options["bsonRegExp"] : false;
    const promoteBuffers = options.promoteBuffers ?? false;
    const promoteLongs = options.promoteLongs ?? true;
    const promoteValues = options.promoteValues ?? true;
    const useBigInt64 = options.useBigInt64 ?? false;
    if (useBigInt64 && !promoteValues) {
      throw new BSONError("Must either request bigint or Long for int64 deserialization");
    }
    if (useBigInt64 && !promoteLongs) {
      throw new BSONError("Must either request bigint or Long for int64 deserialization");
    }
    const validation = options.validation == null ? { utf8: true } : options.validation;
    let globalUTFValidation = true;
    let validationSetting;
    let utf8KeysSet;
    const utf8ValidatedKeys = validation.utf8;
    if (typeof utf8ValidatedKeys === "boolean") {
      validationSetting = utf8ValidatedKeys;
    } else {
      globalUTFValidation = false;
      const utf8ValidationValues = Object.keys(utf8ValidatedKeys).map(function(key) {
        return utf8ValidatedKeys[key];
      });
      if (utf8ValidationValues.length === 0) {
        throw new BSONError("UTF-8 validation setting cannot be empty");
      }
      if (typeof utf8ValidationValues[0] !== "boolean") {
        throw new BSONError("Invalid UTF-8 validation option, must specify boolean values");
      }
      validationSetting = utf8ValidationValues[0];
      if (!utf8ValidationValues.every((item) => item === validationSetting)) {
        throw new BSONError("Invalid UTF-8 validation option - keys must be all true or all false");
      }
    }
    if (!globalUTFValidation) {
      utf8KeysSet = /* @__PURE__ */ new Set();
      for (const key of Object.keys(utf8ValidatedKeys)) {
        utf8KeysSet.add(key);
      }
    }
    const startIndex = index;
    if (buffer2.length < 5)
      throw new BSONError("corrupt bson message < 5 bytes long");
    const size = NumberUtils.getInt32LE(buffer2, index);
    index += 4;
    if (size < 5 || size > buffer2.length)
      throw new BSONError("corrupt bson message");
    const object = isArray ? [] : {};
    let arrayIndex = 0;
    let isPossibleDBRef = isArray ? false : null;
    while (true) {
      const elementType = buffer2[index++];
      if (elementType === 0)
        break;
      let i2 = index;
      while (buffer2[i2] !== 0 && i2 < buffer2.length) {
        i2++;
      }
      if (i2 >= buffer2.byteLength)
        throw new BSONError("Bad BSON Document: illegal CString");
      const name = isArray ? arrayIndex++ : ByteUtils.toUTF8(buffer2, index, i2, false);
      let shouldValidateKey = true;
      if (globalUTFValidation || utf8KeysSet?.has(name)) {
        shouldValidateKey = validationSetting;
      } else {
        shouldValidateKey = !validationSetting;
      }
      if (isPossibleDBRef !== false && name[0] === "$") {
        isPossibleDBRef = allowedDBRefKeys.test(name);
      }
      let value;
      index = i2 + 1;
      if (elementType === BSON_DATA_STRING) {
        const stringSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (stringSize <= 0 || stringSize > buffer2.length - index || buffer2[index + stringSize - 1] !== 0) {
          throw new BSONError("bad string length in bson");
        }
        value = ByteUtils.toUTF8(buffer2, index, index + stringSize - 1, shouldValidateKey);
        index = index + stringSize;
      } else if (elementType === BSON_DATA_OID) {
        const oid = ByteUtils.allocateUnsafe(12);
        for (let i3 = 0; i3 < 12; i3++)
          oid[i3] = buffer2[index + i3];
        value = new ObjectId(oid);
        index = index + 12;
      } else if (elementType === BSON_DATA_INT && promoteValues === false) {
        value = new Int32(NumberUtils.getInt32LE(buffer2, index));
        index += 4;
      } else if (elementType === BSON_DATA_INT) {
        value = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
      } else if (elementType === BSON_DATA_NUMBER) {
        value = NumberUtils.getFloat64LE(buffer2, index);
        index += 8;
        if (promoteValues === false)
          value = new Double(value);
      } else if (elementType === BSON_DATA_DATE) {
        const lowBits = NumberUtils.getInt32LE(buffer2, index);
        const highBits = NumberUtils.getInt32LE(buffer2, index + 4);
        index += 8;
        value = new Date(new Long(lowBits, highBits).toNumber());
      } else if (elementType === BSON_DATA_BOOLEAN) {
        if (buffer2[index] !== 0 && buffer2[index] !== 1)
          throw new BSONError("illegal boolean type value");
        value = buffer2[index++] === 1;
      } else if (elementType === BSON_DATA_OBJECT) {
        const _index = index;
        const objectSize = NumberUtils.getInt32LE(buffer2, index);
        if (objectSize <= 0 || objectSize > buffer2.length - index)
          throw new BSONError("bad embedded document length in bson");
        if (raw) {
          value = buffer2.subarray(index, index + objectSize);
        } else {
          let objectOptions = options;
          if (!globalUTFValidation) {
            objectOptions = { ...options, validation: { utf8: shouldValidateKey } };
          }
          value = deserializeObject(buffer2, _index, objectOptions, false);
        }
        index = index + objectSize;
      } else if (elementType === BSON_DATA_ARRAY) {
        const _index = index;
        const objectSize = NumberUtils.getInt32LE(buffer2, index);
        let arrayOptions = options;
        const stopIndex = index + objectSize;
        if (fieldsAsRaw && fieldsAsRaw[name]) {
          arrayOptions = { ...options, raw: true };
        }
        if (!globalUTFValidation) {
          arrayOptions = { ...arrayOptions, validation: { utf8: shouldValidateKey } };
        }
        value = deserializeObject(buffer2, _index, arrayOptions, true);
        index = index + objectSize;
        if (buffer2[index - 1] !== 0)
          throw new BSONError("invalid array terminator byte");
        if (index !== stopIndex)
          throw new BSONError("corrupted array bson");
      } else if (elementType === BSON_DATA_UNDEFINED) {
        value = void 0;
      } else if (elementType === BSON_DATA_NULL) {
        value = null;
      } else if (elementType === BSON_DATA_LONG) {
        if (useBigInt64) {
          value = NumberUtils.getBigInt64LE(buffer2, index);
          index += 8;
        } else {
          const lowBits = NumberUtils.getInt32LE(buffer2, index);
          const highBits = NumberUtils.getInt32LE(buffer2, index + 4);
          index += 8;
          const long = new Long(lowBits, highBits);
          if (promoteLongs && promoteValues === true) {
            value = long.lessThanOrEqual(JS_INT_MAX_LONG) && long.greaterThanOrEqual(JS_INT_MIN_LONG) ? long.toNumber() : long;
          } else {
            value = long;
          }
        }
      } else if (elementType === BSON_DATA_DECIMAL128) {
        const bytes = ByteUtils.allocateUnsafe(16);
        for (let i3 = 0; i3 < 16; i3++)
          bytes[i3] = buffer2[index + i3];
        index = index + 16;
        value = new Decimal128(bytes);
      } else if (elementType === BSON_DATA_BINARY) {
        let binarySize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        const totalBinarySize = binarySize;
        const subType = buffer2[index++];
        if (binarySize < 0)
          throw new BSONError("Negative binary type element size found");
        if (binarySize > buffer2.byteLength)
          throw new BSONError("Binary type size larger than document size");
        if (subType === Binary.SUBTYPE_BYTE_ARRAY) {
          binarySize = NumberUtils.getInt32LE(buffer2, index);
          index += 4;
          if (binarySize < 0)
            throw new BSONError("Negative binary type element size found for subtype 0x02");
          if (binarySize > totalBinarySize - 4)
            throw new BSONError("Binary type with subtype 0x02 contains too long binary size");
          if (binarySize < totalBinarySize - 4)
            throw new BSONError("Binary type with subtype 0x02 contains too short binary size");
        }
        if (promoteBuffers && promoteValues) {
          value = ByteUtils.toLocalBufferType(buffer2.subarray(index, index + binarySize));
        } else {
          value = new Binary(buffer2.subarray(index, index + binarySize), subType);
          if (subType === BSON_BINARY_SUBTYPE_UUID_NEW && UUID.isValid(value)) {
            value = value.toUUID();
          }
        }
        index = index + binarySize;
      } else if (elementType === BSON_DATA_REGEXP && bsonRegExp === false) {
        i2 = index;
        while (buffer2[i2] !== 0 && i2 < buffer2.length) {
          i2++;
        }
        if (i2 >= buffer2.length)
          throw new BSONError("Bad BSON Document: illegal CString");
        const source = ByteUtils.toUTF8(buffer2, index, i2, false);
        index = i2 + 1;
        i2 = index;
        while (buffer2[i2] !== 0 && i2 < buffer2.length) {
          i2++;
        }
        if (i2 >= buffer2.length)
          throw new BSONError("Bad BSON Document: illegal CString");
        const regExpOptions = ByteUtils.toUTF8(buffer2, index, i2, false);
        index = i2 + 1;
        const optionsArray = new Array(regExpOptions.length);
        for (i2 = 0; i2 < regExpOptions.length; i2++) {
          switch (regExpOptions[i2]) {
            case "m":
              optionsArray[i2] = "m";
              break;
            case "s":
              optionsArray[i2] = "g";
              break;
            case "i":
              optionsArray[i2] = "i";
              break;
          }
        }
        value = new RegExp(source, optionsArray.join(""));
      } else if (elementType === BSON_DATA_REGEXP && bsonRegExp === true) {
        i2 = index;
        while (buffer2[i2] !== 0 && i2 < buffer2.length) {
          i2++;
        }
        if (i2 >= buffer2.length)
          throw new BSONError("Bad BSON Document: illegal CString");
        const source = ByteUtils.toUTF8(buffer2, index, i2, false);
        index = i2 + 1;
        i2 = index;
        while (buffer2[i2] !== 0 && i2 < buffer2.length) {
          i2++;
        }
        if (i2 >= buffer2.length)
          throw new BSONError("Bad BSON Document: illegal CString");
        const regExpOptions = ByteUtils.toUTF8(buffer2, index, i2, false);
        index = i2 + 1;
        value = new BSONRegExp(source, regExpOptions);
      } else if (elementType === BSON_DATA_SYMBOL) {
        const stringSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (stringSize <= 0 || stringSize > buffer2.length - index || buffer2[index + stringSize - 1] !== 0) {
          throw new BSONError("bad string length in bson");
        }
        const symbol = ByteUtils.toUTF8(buffer2, index, index + stringSize - 1, shouldValidateKey);
        value = promoteValues ? symbol : new BSONSymbol(symbol);
        index = index + stringSize;
      } else if (elementType === BSON_DATA_TIMESTAMP) {
        value = new Timestamp({
          i: NumberUtils.getUint32LE(buffer2, index),
          t: NumberUtils.getUint32LE(buffer2, index + 4)
        });
        index += 8;
      } else if (elementType === BSON_DATA_MIN_KEY) {
        value = new MinKey();
      } else if (elementType === BSON_DATA_MAX_KEY) {
        value = new MaxKey();
      } else if (elementType === BSON_DATA_CODE) {
        const stringSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (stringSize <= 0 || stringSize > buffer2.length - index || buffer2[index + stringSize - 1] !== 0) {
          throw new BSONError("bad string length in bson");
        }
        const functionString = ByteUtils.toUTF8(buffer2, index, index + stringSize - 1, shouldValidateKey);
        value = new Code(functionString);
        index = index + stringSize;
      } else if (elementType === BSON_DATA_CODE_W_SCOPE) {
        const totalSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (totalSize < 4 + 4 + 4 + 1) {
          throw new BSONError("code_w_scope total size shorter minimum expected length");
        }
        const stringSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (stringSize <= 0 || stringSize > buffer2.length - index || buffer2[index + stringSize - 1] !== 0) {
          throw new BSONError("bad string length in bson");
        }
        const functionString = ByteUtils.toUTF8(buffer2, index, index + stringSize - 1, shouldValidateKey);
        index = index + stringSize;
        const _index = index;
        const objectSize = NumberUtils.getInt32LE(buffer2, index);
        const scopeObject = deserializeObject(buffer2, _index, options, false);
        index = index + objectSize;
        if (totalSize < 4 + 4 + objectSize + stringSize) {
          throw new BSONError("code_w_scope total size is too short, truncating scope");
        }
        if (totalSize > 4 + 4 + objectSize + stringSize) {
          throw new BSONError("code_w_scope total size is too long, clips outer document");
        }
        value = new Code(functionString, scopeObject);
      } else if (elementType === BSON_DATA_DBPOINTER) {
        const stringSize = NumberUtils.getInt32LE(buffer2, index);
        index += 4;
        if (stringSize <= 0 || stringSize > buffer2.length - index || buffer2[index + stringSize - 1] !== 0)
          throw new BSONError("bad string length in bson");
        const namespace = ByteUtils.toUTF8(buffer2, index, index + stringSize - 1, shouldValidateKey);
        index = index + stringSize;
        const oidBuffer = ByteUtils.allocateUnsafe(12);
        for (let i3 = 0; i3 < 12; i3++)
          oidBuffer[i3] = buffer2[index + i3];
        const oid = new ObjectId(oidBuffer);
        index = index + 12;
        value = new DBRef(namespace, oid);
      } else {
        throw new BSONError(`Detected unknown BSON type ${elementType.toString(16)} for fieldname "${name}"`);
      }
      if (name === "__proto__") {
        Object.defineProperty(object, name, {
          value,
          writable: true,
          enumerable: true,
          configurable: true
        });
      } else {
        object[name] = value;
      }
    }
    if (size !== index - startIndex) {
      if (isArray)
        throw new BSONError("corrupt array bson");
      throw new BSONError("corrupt object bson");
    }
    if (!isPossibleDBRef)
      return object;
    if (isDBRefLike(object)) {
      const copy = Object.assign({}, object);
      delete copy.$ref;
      delete copy.$id;
      delete copy.$db;
      return new DBRef(object.$ref, object.$id, object.$db, copy);
    }
    return object;
  }
  function isBSONType(value) {
    return value != null && typeof value === "object" && "_bsontype" in value && typeof value._bsontype === "string";
  }
  function deserializeValue(value, options = {}) {
    if (typeof value === "number") {
      const in32BitRange = value <= BSON_INT32_MAX && value >= BSON_INT32_MIN;
      const in64BitRange = value <= BSON_INT64_MAX && value >= BSON_INT64_MIN;
      if (options.relaxed || options.legacy) {
        return value;
      }
      if (Number.isInteger(value) && !Object.is(value, -0)) {
        if (in32BitRange) {
          return new Int32(value);
        }
        if (in64BitRange) {
          if (options.useBigInt64) {
            return BigInt(value);
          }
          return Long.fromNumber(value);
        }
      }
      return new Double(value);
    }
    if (value == null || typeof value !== "object")
      return value;
    if (value.$undefined)
      return null;
    const keys = Object.keys(value).filter((k4) => k4.startsWith("$") && value[k4] != null);
    for (let i2 = 0; i2 < keys.length; i2++) {
      const c5 = keysToCodecs[keys[i2]];
      if (c5)
        return c5.fromExtendedJSON(value, options);
    }
    if (value.$date != null) {
      const d6 = value.$date;
      const date = /* @__PURE__ */ new Date();
      if (options.legacy) {
        if (typeof d6 === "number")
          date.setTime(d6);
        else if (typeof d6 === "string")
          date.setTime(Date.parse(d6));
        else if (typeof d6 === "bigint")
          date.setTime(Number(d6));
        else
          throw new BSONRuntimeError(`Unrecognized type for EJSON date: ${typeof d6}`);
      } else {
        if (typeof d6 === "string")
          date.setTime(Date.parse(d6));
        else if (Long.isLong(d6))
          date.setTime(d6.toNumber());
        else if (typeof d6 === "number" && options.relaxed)
          date.setTime(d6);
        else if (typeof d6 === "bigint")
          date.setTime(Number(d6));
        else
          throw new BSONRuntimeError(`Unrecognized type for EJSON date: ${typeof d6}`);
      }
      return date;
    }
    if (value.$code != null) {
      const copy = Object.assign({}, value);
      if (value.$scope) {
        copy.$scope = deserializeValue(value.$scope);
      }
      return Code.fromExtendedJSON(value);
    }
    if (isDBRefLike(value) || value.$dbPointer) {
      const v3 = value.$ref ? value : value.$dbPointer;
      if (v3 instanceof DBRef)
        return v3;
      const dollarKeys = Object.keys(v3).filter((k4) => k4.startsWith("$"));
      let valid = true;
      dollarKeys.forEach((k4) => {
        if (["$ref", "$id", "$db"].indexOf(k4) === -1)
          valid = false;
      });
      if (valid)
        return DBRef.fromExtendedJSON(v3);
    }
    return value;
  }
  function serializeArray(array, options) {
    return array.map((v3, index) => {
      options.seenObjects.push({ propertyName: `index ${index}`, obj: null });
      try {
        return serializeValue(v3, options);
      } finally {
        options.seenObjects.pop();
      }
    });
  }
  function getISOString(date) {
    const isoStr = date.toISOString();
    return date.getUTCMilliseconds() !== 0 ? isoStr : isoStr.slice(0, -5) + "Z";
  }
  function serializeValue(value, options) {
    if (value instanceof Map || isMap(value)) {
      const obj = /* @__PURE__ */ Object.create(null);
      for (const [k4, v3] of value) {
        if (typeof k4 !== "string") {
          throw new BSONError("Can only serialize maps with string keys");
        }
        obj[k4] = v3;
      }
      return serializeValue(obj, options);
    }
    if ((typeof value === "object" || typeof value === "function") && value !== null) {
      const index = options.seenObjects.findIndex((entry) => entry.obj === value);
      if (index !== -1) {
        const props = options.seenObjects.map((entry) => entry.propertyName);
        const leadingPart = props.slice(0, index).map((prop) => `${prop} -> `).join("");
        const alreadySeen = props[index];
        const circularPart = " -> " + props.slice(index + 1, props.length - 1).map((prop) => `${prop} -> `).join("");
        const current = props[props.length - 1];
        const leadingSpace = " ".repeat(leadingPart.length + alreadySeen.length / 2);
        const dashes = "-".repeat(circularPart.length + (alreadySeen.length + current.length) / 2 - 1);
        throw new BSONError(`Converting circular structure to EJSON:
    ${leadingPart}${alreadySeen}${circularPart}${current}
    ${leadingSpace}\\${dashes}/`);
      }
      options.seenObjects[options.seenObjects.length - 1].obj = value;
    }
    if (Array.isArray(value))
      return serializeArray(value, options);
    if (value === void 0)
      return options.ignoreUndefined ? void 0 : null;
    if (value instanceof Date || isDate(value)) {
      const dateNum = value.getTime(), inRange = dateNum > -1 && dateNum < 2534023188e5;
      if (options.legacy) {
        return options.relaxed && inRange ? { $date: value.getTime() } : { $date: getISOString(value) };
      }
      return options.relaxed && inRange ? { $date: getISOString(value) } : { $date: { $numberLong: value.getTime().toString() } };
    }
    if (typeof value === "number" && (!options.relaxed || !isFinite(value))) {
      if (Number.isInteger(value) && !Object.is(value, -0)) {
        if (value >= BSON_INT32_MIN && value <= BSON_INT32_MAX) {
          return { $numberInt: value.toString() };
        }
        if (value >= BSON_INT64_MIN && value <= BSON_INT64_MAX) {
          return { $numberLong: value.toString() };
        }
      }
      return { $numberDouble: Object.is(value, -0) ? "-0.0" : value.toString() };
    }
    if (typeof value === "bigint") {
      if (!options.relaxed) {
        return { $numberLong: BigInt.asIntN(64, value).toString() };
      }
      return Number(BigInt.asIntN(64, value));
    }
    if (value instanceof RegExp || isRegExp(value)) {
      let flags = value.flags;
      if (flags === void 0) {
        const match = value.toString().match(/[gimuy]*$/);
        if (match) {
          flags = match[0];
        }
      }
      const rx = new BSONRegExp(value.source, flags);
      return rx.toExtendedJSON(options);
    }
    if (value != null && typeof value === "object")
      return serializeDocument(value, options);
    return value;
  }
  function serializeDocument(doc, options) {
    if (doc == null || typeof doc !== "object")
      throw new BSONError("not an object instance");
    const bsontype = doc._bsontype;
    if (typeof bsontype === "undefined") {
      const _doc = {};
      for (const name of Object.keys(doc)) {
        options.seenObjects.push({ propertyName: name, obj: null });
        try {
          const value = serializeValue(doc[name], options);
          if (name === "__proto__") {
            Object.defineProperty(_doc, name, {
              value,
              writable: true,
              enumerable: true,
              configurable: true
            });
          } else {
            _doc[name] = value;
          }
        } finally {
          options.seenObjects.pop();
        }
      }
      return _doc;
    } else if (doc != null && typeof doc === "object" && typeof doc._bsontype === "string" && doc[BSON_VERSION_SYMBOL] !== BSON_MAJOR_VERSION) {
      throw new BSONVersionError();
    } else if (isBSONType(doc)) {
      let outDoc = doc;
      if (typeof outDoc.toExtendedJSON !== "function") {
        const mapper = BSON_TYPE_MAPPINGS[doc._bsontype];
        if (!mapper) {
          throw new BSONError("Unrecognized or invalid _bsontype: " + doc._bsontype);
        }
        outDoc = mapper(outDoc);
      }
      if (bsontype === "Code" && outDoc.scope) {
        outDoc = new Code(outDoc.code, serializeValue(outDoc.scope, options));
      } else if (bsontype === "DBRef" && outDoc.oid) {
        outDoc = new DBRef(serializeValue(outDoc.collection, options), serializeValue(outDoc.oid, options), serializeValue(outDoc.db, options), serializeValue(outDoc.fields, options));
      }
      return outDoc.toExtendedJSON(options);
    } else {
      throw new BSONError("_bsontype must be a string, but was: " + typeof bsontype);
    }
  }
  function parse(text, options) {
    const ejsonOptions = {
      useBigInt64: options?.useBigInt64 ?? false,
      relaxed: options?.relaxed ?? true,
      legacy: options?.legacy ?? false
    };
    return JSON.parse(text, (key, value) => {
      if (key.indexOf("\0") !== -1) {
        throw new BSONError(`BSON Document field names cannot contain null bytes, found: ${JSON.stringify(key)}`);
      }
      return deserializeValue(value, ejsonOptions);
    });
  }
  function stringify(value, replacer, space, options) {
    if (space != null && typeof space === "object") {
      options = space;
      space = 0;
    }
    if (replacer != null && typeof replacer === "object" && !Array.isArray(replacer)) {
      options = replacer;
      replacer = void 0;
      space = 0;
    }
    const serializeOptions = Object.assign({ relaxed: true, legacy: false }, options, {
      seenObjects: [{ propertyName: "(root)", obj: null }]
    });
    const doc = serializeValue(value, serializeOptions);
    return JSON.stringify(doc, replacer, space);
  }
  function EJSONserialize(value, options) {
    options = options || {};
    return JSON.parse(stringify(value, options));
  }
  function EJSONdeserialize(ejson, options) {
    options = options || {};
    return parse(JSON.stringify(ejson), options);
  }
  function getSize(source, offset) {
    try {
      return NumberUtils.getNonnegativeInt32LE(source, offset);
    } catch (cause) {
      throw new BSONOffsetError("BSON size cannot be negative", offset, { cause });
    }
  }
  function findNull(bytes, offset) {
    let nullTerminatorOffset = offset;
    for (; bytes[nullTerminatorOffset] !== 0; nullTerminatorOffset++)
      ;
    if (nullTerminatorOffset === bytes.length - 1) {
      throw new BSONOffsetError("Null terminator not found", offset);
    }
    return nullTerminatorOffset;
  }
  function parseToElements(bytes, startOffset = 0) {
    startOffset ??= 0;
    if (bytes.length < 5) {
      throw new BSONOffsetError(`Input must be at least 5 bytes, got ${bytes.length} bytes`, startOffset);
    }
    const documentSize = getSize(bytes, startOffset);
    if (documentSize > bytes.length - startOffset) {
      throw new BSONOffsetError(`Parsed documentSize (${documentSize} bytes) does not match input length (${bytes.length} bytes)`, startOffset);
    }
    if (bytes[startOffset + documentSize - 1] !== 0) {
      throw new BSONOffsetError("BSON documents must end in 0x00", startOffset + documentSize);
    }
    const elements = [];
    let offset = startOffset + 4;
    while (offset <= documentSize + startOffset) {
      const type = bytes[offset];
      offset += 1;
      if (type === 0) {
        if (offset - startOffset !== documentSize) {
          throw new BSONOffsetError(`Invalid 0x00 type byte`, offset);
        }
        break;
      }
      const nameOffset = offset;
      const nameLength = findNull(bytes, offset) - nameOffset;
      offset += nameLength + 1;
      let length;
      if (type === BSONElementType.double || type === BSONElementType.long || type === BSONElementType.date || type === BSONElementType.timestamp) {
        length = 8;
      } else if (type === BSONElementType.int) {
        length = 4;
      } else if (type === BSONElementType.objectId) {
        length = 12;
      } else if (type === BSONElementType.decimal) {
        length = 16;
      } else if (type === BSONElementType.bool) {
        length = 1;
      } else if (type === BSONElementType.null || type === BSONElementType.undefined || type === BSONElementType.maxKey || type === BSONElementType.minKey) {
        length = 0;
      } else if (type === BSONElementType.regex) {
        length = findNull(bytes, findNull(bytes, offset) + 1) + 1 - offset;
      } else if (type === BSONElementType.object || type === BSONElementType.array || type === BSONElementType.javascriptWithScope) {
        length = getSize(bytes, offset);
      } else if (type === BSONElementType.string || type === BSONElementType.binData || type === BSONElementType.dbPointer || type === BSONElementType.javascript || type === BSONElementType.symbol) {
        length = getSize(bytes, offset) + 4;
        if (type === BSONElementType.binData) {
          length += 1;
        }
        if (type === BSONElementType.dbPointer) {
          length += 12;
        }
      } else {
        throw new BSONOffsetError(`Invalid 0x${type.toString(16).padStart(2, "0")} type byte`, offset);
      }
      if (length > documentSize) {
        throw new BSONOffsetError("value reports length larger than document", offset);
      }
      elements.push([type, nameOffset, nameLength, offset, length]);
      offset += length;
    }
    return elements;
  }
  function deserialize(buffer2, options = {}) {
    return internalDeserialize(ByteUtils.toLocalBufferType(buffer2), options);
  }
  var TypedArrayPrototypeGetSymbolToStringTag, BSON_MAJOR_VERSION, BSON_VERSION_SYMBOL, BSON_INT32_MAX, BSON_INT32_MIN, BSON_INT64_MAX, BSON_INT64_MIN, JS_INT_MAX, JS_INT_MIN, BSON_DATA_NUMBER, BSON_DATA_STRING, BSON_DATA_OBJECT, BSON_DATA_ARRAY, BSON_DATA_BINARY, BSON_DATA_UNDEFINED, BSON_DATA_OID, BSON_DATA_BOOLEAN, BSON_DATA_DATE, BSON_DATA_NULL, BSON_DATA_REGEXP, BSON_DATA_DBPOINTER, BSON_DATA_CODE, BSON_DATA_SYMBOL, BSON_DATA_CODE_W_SCOPE, BSON_DATA_INT, BSON_DATA_TIMESTAMP, BSON_DATA_LONG, BSON_DATA_DECIMAL128, BSON_DATA_MIN_KEY, BSON_DATA_MAX_KEY, BSON_BINARY_SUBTYPE_UUID_NEW, BSONType, BSONError, BSONVersionError, BSONRuntimeError, BSONOffsetError, TextDecoderFatal, TextDecoderNonFatal, nodejsRandomBytes, nodeJsByteUtils, webRandomBytes, HEX_DIGIT, webByteUtils, hasGlobalBuffer, ByteUtils, bsonType, BSONValue, FLOAT, FLOAT_BYTES, isBigEndian, NumberUtils, Binary, UUID_BYTE_LENGTH, UUID_WITHOUT_DASHES, UUID_WITH_DASHES, UUID, Code, DBRef, wasm, TWO_PWR_16_DBL, TWO_PWR_24_DBL, TWO_PWR_32_DBL, TWO_PWR_64_DBL, TWO_PWR_63_DBL, INT_CACHE, UINT_CACHE, MAX_INT64_STRING_LENGTH, DECIMAL_REG_EX, Long, PARSE_STRING_REGEXP, PARSE_INF_REGEXP, PARSE_NAN_REGEXP, EXPONENT_MAX, EXPONENT_MIN, EXPONENT_BIAS, MAX_DIGITS, NAN_BUFFER, INF_NEGATIVE_BUFFER, INF_POSITIVE_BUFFER, EXPONENT_REGEX, COMBINATION_MASK, EXPONENT_MASK, COMBINATION_INFINITY, COMBINATION_NAN, Decimal128, Double, Int32, MaxKey, MinKey, PROCESS_UNIQUE, __idCache, ObjectId, BSONRegExp, BSONSymbol, LongWithoutOverridesClass, Timestamp, JS_INT_MAX_LONG, JS_INT_MIN_LONG, allowedDBRefKeys, keysToCodecs, BSON_TYPE_MAPPINGS, EJSON, BSONElementType, onDemand, MAXSIZE, buffer;
  var init_bson = __esm({
    "node_modules/bson/lib/bson.mjs"() {
      TypedArrayPrototypeGetSymbolToStringTag = (() => {
        const g5 = Object.getOwnPropertyDescriptor(Object.getPrototypeOf(Uint8Array.prototype), Symbol.toStringTag).get;
        return (value) => g5.call(value);
      })();
      BSON_MAJOR_VERSION = 7;
      BSON_VERSION_SYMBOL = Symbol.for("@@mdb.bson.version");
      BSON_INT32_MAX = 2147483647;
      BSON_INT32_MIN = -2147483648;
      BSON_INT64_MAX = Math.pow(2, 63) - 1;
      BSON_INT64_MIN = -Math.pow(2, 63);
      JS_INT_MAX = Math.pow(2, 53);
      JS_INT_MIN = -Math.pow(2, 53);
      BSON_DATA_NUMBER = 1;
      BSON_DATA_STRING = 2;
      BSON_DATA_OBJECT = 3;
      BSON_DATA_ARRAY = 4;
      BSON_DATA_BINARY = 5;
      BSON_DATA_UNDEFINED = 6;
      BSON_DATA_OID = 7;
      BSON_DATA_BOOLEAN = 8;
      BSON_DATA_DATE = 9;
      BSON_DATA_NULL = 10;
      BSON_DATA_REGEXP = 11;
      BSON_DATA_DBPOINTER = 12;
      BSON_DATA_CODE = 13;
      BSON_DATA_SYMBOL = 14;
      BSON_DATA_CODE_W_SCOPE = 15;
      BSON_DATA_INT = 16;
      BSON_DATA_TIMESTAMP = 17;
      BSON_DATA_LONG = 18;
      BSON_DATA_DECIMAL128 = 19;
      BSON_DATA_MIN_KEY = 255;
      BSON_DATA_MAX_KEY = 127;
      BSON_BINARY_SUBTYPE_UUID_NEW = 4;
      BSONType = Object.freeze({
        double: 1,
        string: 2,
        object: 3,
        array: 4,
        binData: 5,
        undefined: 6,
        objectId: 7,
        bool: 8,
        date: 9,
        null: 10,
        regex: 11,
        dbPointer: 12,
        javascript: 13,
        symbol: 14,
        javascriptWithScope: 15,
        int: 16,
        timestamp: 17,
        long: 18,
        decimal: 19,
        minKey: -1,
        maxKey: 127
      });
      BSONError = class extends Error {
        get bsonError() {
          return true;
        }
        get name() {
          return "BSONError";
        }
        constructor(message, options) {
          super(message, options);
        }
        static isBSONError(value) {
          return value != null && typeof value === "object" && "bsonError" in value && value.bsonError === true && "name" in value && "message" in value && "stack" in value;
        }
      };
      BSONVersionError = class extends BSONError {
        get name() {
          return "BSONVersionError";
        }
        constructor() {
          super(`Unsupported BSON version, bson types must be from bson ${BSON_MAJOR_VERSION}.x.x`);
        }
      };
      BSONRuntimeError = class extends BSONError {
        get name() {
          return "BSONRuntimeError";
        }
        constructor(message) {
          super(message);
        }
      };
      BSONOffsetError = class extends BSONError {
        get name() {
          return "BSONOffsetError";
        }
        offset;
        constructor(message, offset, options) {
          super(`${message}. offset: ${offset}`, options);
          this.offset = offset;
        }
      };
      nodejsRandomBytes = (() => {
        const { crypto: crypto2 } = globalThis;
        if (crypto2 != null && typeof crypto2.getRandomValues === "function") {
          return nodejsSecureRandomBytes;
        } else {
          return nodejsMathRandomBytes;
        }
      })();
      nodeJsByteUtils = {
        isUint8Array,
        toLocalBufferType(potentialBuffer) {
          if (Buffer.isBuffer(potentialBuffer)) {
            return potentialBuffer;
          }
          if (ArrayBuffer.isView(potentialBuffer)) {
            return Buffer.from(potentialBuffer.buffer, potentialBuffer.byteOffset, potentialBuffer.byteLength);
          }
          const stringTag = potentialBuffer?.[Symbol.toStringTag] ?? Object.prototype.toString.call(potentialBuffer);
          if (stringTag === "ArrayBuffer" || stringTag === "SharedArrayBuffer" || stringTag === "[object ArrayBuffer]" || stringTag === "[object SharedArrayBuffer]") {
            return Buffer.from(potentialBuffer);
          }
          throw new BSONError(`Cannot create Buffer from the passed potentialBuffer.`);
        },
        allocate(size) {
          return Buffer.alloc(size);
        },
        allocateUnsafe(size) {
          return Buffer.allocUnsafe(size);
        },
        compare(a4, b5) {
          return nodeJsByteUtils.toLocalBufferType(a4).compare(b5);
        },
        concat(list) {
          return Buffer.concat(list);
        },
        copy(source, target, targetStart, sourceStart, sourceEnd) {
          return nodeJsByteUtils.toLocalBufferType(source).copy(target, targetStart ?? 0, sourceStart ?? 0, sourceEnd ?? source.length);
        },
        equals(a4, b5) {
          return nodeJsByteUtils.toLocalBufferType(a4).equals(b5);
        },
        fromNumberArray(array) {
          return Buffer.from(array);
        },
        fromBase64(base64) {
          return Buffer.from(base64, "base64");
        },
        fromUTF8(utf8) {
          return Buffer.from(utf8, "utf8");
        },
        toBase64(buffer2) {
          return nodeJsByteUtils.toLocalBufferType(buffer2).toString("base64");
        },
        fromISO88591(codePoints) {
          return Buffer.from(codePoints, "binary");
        },
        toISO88591(buffer2) {
          return nodeJsByteUtils.toLocalBufferType(buffer2).toString("binary");
        },
        fromHex(hex) {
          return Buffer.from(hex, "hex");
        },
        toHex(buffer2) {
          return nodeJsByteUtils.toLocalBufferType(buffer2).toString("hex");
        },
        toUTF8(buffer2, start, end, fatal) {
          const basicLatin = end - start <= 20 ? tryReadBasicLatin(buffer2, start, end) : null;
          if (basicLatin != null) {
            return basicLatin;
          }
          const string = nodeJsByteUtils.toLocalBufferType(buffer2).toString("utf8", start, end);
          if (fatal) {
            for (let i2 = 0; i2 < string.length; i2++) {
              if (string.charCodeAt(i2) === 65533) {
                parseUtf8(buffer2, start, end, true);
                break;
              }
            }
          }
          return string;
        },
        utf8ByteLength(input) {
          return Buffer.byteLength(input, "utf8");
        },
        encodeUTF8Into(buffer2, source, byteOffset) {
          const latinBytesWritten = tryWriteBasicLatin(buffer2, source, byteOffset);
          if (latinBytesWritten != null) {
            return latinBytesWritten;
          }
          return nodeJsByteUtils.toLocalBufferType(buffer2).write(source, byteOffset, void 0, "utf8");
        },
        randomBytes: nodejsRandomBytes,
        swap32(buffer2) {
          return nodeJsByteUtils.toLocalBufferType(buffer2).swap32();
        }
      };
      webRandomBytes = (() => {
        const { crypto: crypto2 } = globalThis;
        if (crypto2 != null && typeof crypto2.getRandomValues === "function") {
          return (byteLength) => {
            return crypto2.getRandomValues(webByteUtils.allocate(byteLength));
          };
        } else {
          if (isReactNative()) {
            const { console: console2 } = globalThis;
            console2?.warn?.("BSON: For React Native please polyfill crypto.getRandomValues, e.g. using: https://www.npmjs.com/package/react-native-get-random-values.");
          }
          return webMathRandomBytes;
        }
      })();
      HEX_DIGIT = /(\d|[a-f])/i;
      webByteUtils = {
        isUint8Array,
        toLocalBufferType(potentialUint8array) {
          const stringTag = potentialUint8array?.[Symbol.toStringTag] ?? Object.prototype.toString.call(potentialUint8array);
          if (stringTag === "Uint8Array") {
            return potentialUint8array;
          }
          if (ArrayBuffer.isView(potentialUint8array)) {
            return new Uint8Array(potentialUint8array.buffer.slice(potentialUint8array.byteOffset, potentialUint8array.byteOffset + potentialUint8array.byteLength));
          }
          if (stringTag === "ArrayBuffer" || stringTag === "SharedArrayBuffer" || stringTag === "[object ArrayBuffer]" || stringTag === "[object SharedArrayBuffer]") {
            return new Uint8Array(potentialUint8array);
          }
          throw new BSONError(`Cannot make a Uint8Array from passed potentialBuffer.`);
        },
        allocate(size) {
          if (typeof size !== "number") {
            throw new TypeError(`The "size" argument must be of type number. Received ${String(size)}`);
          }
          return new Uint8Array(size);
        },
        allocateUnsafe(size) {
          return webByteUtils.allocate(size);
        },
        compare(uint8Array, otherUint8Array) {
          if (uint8Array === otherUint8Array)
            return 0;
          const len = Math.min(uint8Array.length, otherUint8Array.length);
          for (let i2 = 0; i2 < len; i2++) {
            if (uint8Array[i2] < otherUint8Array[i2])
              return -1;
            if (uint8Array[i2] > otherUint8Array[i2])
              return 1;
          }
          if (uint8Array.length < otherUint8Array.length)
            return -1;
          if (uint8Array.length > otherUint8Array.length)
            return 1;
          return 0;
        },
        concat(uint8Arrays) {
          if (uint8Arrays.length === 0)
            return webByteUtils.allocate(0);
          let totalLength = 0;
          for (const uint8Array of uint8Arrays) {
            totalLength += uint8Array.length;
          }
          const result = webByteUtils.allocate(totalLength);
          let offset = 0;
          for (const uint8Array of uint8Arrays) {
            result.set(uint8Array, offset);
            offset += uint8Array.length;
          }
          return result;
        },
        copy(source, target, targetStart, sourceStart, sourceEnd) {
          if (sourceEnd !== void 0 && sourceEnd < 0) {
            throw new RangeError(`The value of "sourceEnd" is out of range. It must be >= 0. Received ${sourceEnd}`);
          }
          sourceEnd = sourceEnd ?? source.length;
          if (sourceStart !== void 0 && (sourceStart < 0 || sourceStart > sourceEnd)) {
            throw new RangeError(`The value of "sourceStart" is out of range. It must be >= 0 and <= ${sourceEnd}. Received ${sourceStart}`);
          }
          sourceStart = sourceStart ?? 0;
          if (targetStart !== void 0 && targetStart < 0) {
            throw new RangeError(`The value of "targetStart" is out of range. It must be >= 0. Received ${targetStart}`);
          }
          targetStart = targetStart ?? 0;
          const srcSlice = source.subarray(sourceStart, sourceEnd);
          const maxLen = Math.min(srcSlice.length, target.length - targetStart);
          if (maxLen <= 0) {
            return 0;
          }
          target.set(srcSlice.subarray(0, maxLen), targetStart);
          return maxLen;
        },
        equals(uint8Array, otherUint8Array) {
          if (uint8Array.byteLength !== otherUint8Array.byteLength) {
            return false;
          }
          for (let i2 = 0; i2 < uint8Array.byteLength; i2++) {
            if (uint8Array[i2] !== otherUint8Array[i2]) {
              return false;
            }
          }
          return true;
        },
        fromNumberArray(array) {
          return Uint8Array.from(array);
        },
        fromBase64(base64) {
          return Uint8Array.from(atob(base64), (c5) => c5.charCodeAt(0));
        },
        fromUTF8(utf8) {
          return new TextEncoder().encode(utf8);
        },
        toBase64(uint8array) {
          return btoa(webByteUtils.toISO88591(uint8array));
        },
        fromISO88591(codePoints) {
          return Uint8Array.from(codePoints, (c5) => c5.charCodeAt(0) & 255);
        },
        toISO88591(uint8array) {
          return Array.from(Uint16Array.from(uint8array), (b5) => String.fromCharCode(b5)).join("");
        },
        fromHex(hex) {
          const evenLengthHex = hex.length % 2 === 0 ? hex : hex.slice(0, hex.length - 1);
          const buffer2 = [];
          for (let i2 = 0; i2 < evenLengthHex.length; i2 += 2) {
            const firstDigit = evenLengthHex[i2];
            const secondDigit = evenLengthHex[i2 + 1];
            if (!HEX_DIGIT.test(firstDigit)) {
              break;
            }
            if (!HEX_DIGIT.test(secondDigit)) {
              break;
            }
            const hexDigit = Number.parseInt(`${firstDigit}${secondDigit}`, 16);
            buffer2.push(hexDigit);
          }
          return Uint8Array.from(buffer2);
        },
        toHex(uint8array) {
          return Array.from(uint8array, (byte) => byte.toString(16).padStart(2, "0")).join("");
        },
        toUTF8(uint8array, start, end, fatal) {
          const basicLatin = end - start <= 20 ? tryReadBasicLatin(uint8array, start, end) : null;
          if (basicLatin != null) {
            return basicLatin;
          }
          return parseUtf8(uint8array, start, end, fatal);
        },
        utf8ByteLength(input) {
          return new TextEncoder().encode(input).byteLength;
        },
        encodeUTF8Into(uint8array, source, byteOffset) {
          const bytes = new TextEncoder().encode(source);
          uint8array.set(bytes, byteOffset);
          return bytes.byteLength;
        },
        randomBytes: webRandomBytes,
        swap32(buffer2) {
          if (buffer2.length % 4 !== 0) {
            throw new RangeError("Buffer size must be a multiple of 32-bits");
          }
          for (let i2 = 0; i2 < buffer2.length; i2 += 4) {
            const byte0 = buffer2[i2];
            const byte1 = buffer2[i2 + 1];
            const byte2 = buffer2[i2 + 2];
            const byte3 = buffer2[i2 + 3];
            buffer2[i2] = byte3;
            buffer2[i2 + 1] = byte2;
            buffer2[i2 + 2] = byte1;
            buffer2[i2 + 3] = byte0;
          }
          return buffer2;
        }
      };
      hasGlobalBuffer = typeof Buffer === "function" && Buffer.prototype?._isBuffer !== true;
      ByteUtils = hasGlobalBuffer ? nodeJsByteUtils : webByteUtils;
      bsonType = Symbol.for("@@mdb.bson.type");
      BSONValue = class {
        get [bsonType]() {
          return this._bsontype;
        }
        get [BSON_VERSION_SYMBOL]() {
          return BSON_MAJOR_VERSION;
        }
        [Symbol.for("nodejs.util.inspect.custom")](depth, options, inspect) {
          return this.inspect(depth, options, inspect);
        }
      };
      FLOAT = new Float64Array(1);
      FLOAT_BYTES = new Uint8Array(FLOAT.buffer, 0, 8);
      FLOAT[0] = -1;
      isBigEndian = FLOAT_BYTES[7] === 0;
      NumberUtils = {
        isBigEndian,
        getNonnegativeInt32LE(source, offset) {
          if (source[offset + 3] > 127) {
            throw new RangeError(`Size cannot be negative at offset: ${offset}`);
          }
          return source[offset] | source[offset + 1] << 8 | source[offset + 2] << 16 | source[offset + 3] << 24;
        },
        getInt32LE(source, offset) {
          return source[offset] | source[offset + 1] << 8 | source[offset + 2] << 16 | source[offset + 3] << 24;
        },
        getUint32LE(source, offset) {
          return source[offset] + source[offset + 1] * 256 + source[offset + 2] * 65536 + source[offset + 3] * 16777216;
        },
        getUint32BE(source, offset) {
          return source[offset + 3] + source[offset + 2] * 256 + source[offset + 1] * 65536 + source[offset] * 16777216;
        },
        getBigInt64LE(source, offset) {
          const hi = BigInt(source[offset + 4] + source[offset + 5] * 256 + source[offset + 6] * 65536 + (source[offset + 7] << 24));
          const lo = BigInt(source[offset] + source[offset + 1] * 256 + source[offset + 2] * 65536 + source[offset + 3] * 16777216);
          return (hi << 32n) + lo;
        },
        getFloat64LE: isBigEndian ? (source, offset) => {
          FLOAT_BYTES[7] = source[offset];
          FLOAT_BYTES[6] = source[offset + 1];
          FLOAT_BYTES[5] = source[offset + 2];
          FLOAT_BYTES[4] = source[offset + 3];
          FLOAT_BYTES[3] = source[offset + 4];
          FLOAT_BYTES[2] = source[offset + 5];
          FLOAT_BYTES[1] = source[offset + 6];
          FLOAT_BYTES[0] = source[offset + 7];
          return FLOAT[0];
        } : (source, offset) => {
          FLOAT_BYTES[0] = source[offset];
          FLOAT_BYTES[1] = source[offset + 1];
          FLOAT_BYTES[2] = source[offset + 2];
          FLOAT_BYTES[3] = source[offset + 3];
          FLOAT_BYTES[4] = source[offset + 4];
          FLOAT_BYTES[5] = source[offset + 5];
          FLOAT_BYTES[6] = source[offset + 6];
          FLOAT_BYTES[7] = source[offset + 7];
          return FLOAT[0];
        },
        setInt32BE(destination, offset, value) {
          destination[offset + 3] = value;
          value >>>= 8;
          destination[offset + 2] = value;
          value >>>= 8;
          destination[offset + 1] = value;
          value >>>= 8;
          destination[offset] = value;
          return 4;
        },
        setInt32LE(destination, offset, value) {
          destination[offset] = value;
          value >>>= 8;
          destination[offset + 1] = value;
          value >>>= 8;
          destination[offset + 2] = value;
          value >>>= 8;
          destination[offset + 3] = value;
          return 4;
        },
        setBigInt64LE(destination, offset, value) {
          const mask32bits = 0xffffffffn;
          let lo = Number(value & mask32bits);
          destination[offset] = lo;
          lo >>= 8;
          destination[offset + 1] = lo;
          lo >>= 8;
          destination[offset + 2] = lo;
          lo >>= 8;
          destination[offset + 3] = lo;
          let hi = Number(value >> 32n & mask32bits);
          destination[offset + 4] = hi;
          hi >>= 8;
          destination[offset + 5] = hi;
          hi >>= 8;
          destination[offset + 6] = hi;
          hi >>= 8;
          destination[offset + 7] = hi;
          return 8;
        },
        setFloat64LE: isBigEndian ? (destination, offset, value) => {
          FLOAT[0] = value;
          destination[offset] = FLOAT_BYTES[7];
          destination[offset + 1] = FLOAT_BYTES[6];
          destination[offset + 2] = FLOAT_BYTES[5];
          destination[offset + 3] = FLOAT_BYTES[4];
          destination[offset + 4] = FLOAT_BYTES[3];
          destination[offset + 5] = FLOAT_BYTES[2];
          destination[offset + 6] = FLOAT_BYTES[1];
          destination[offset + 7] = FLOAT_BYTES[0];
          return 8;
        } : (destination, offset, value) => {
          FLOAT[0] = value;
          destination[offset] = FLOAT_BYTES[0];
          destination[offset + 1] = FLOAT_BYTES[1];
          destination[offset + 2] = FLOAT_BYTES[2];
          destination[offset + 3] = FLOAT_BYTES[3];
          destination[offset + 4] = FLOAT_BYTES[4];
          destination[offset + 5] = FLOAT_BYTES[5];
          destination[offset + 6] = FLOAT_BYTES[6];
          destination[offset + 7] = FLOAT_BYTES[7];
          return 8;
        }
      };
      Binary = class _Binary extends BSONValue {
        get _bsontype() {
          return "Binary";
        }
        static BSON_BINARY_SUBTYPE_DEFAULT = 0;
        static BUFFER_SIZE = 256;
        static SUBTYPE_DEFAULT = 0;
        static SUBTYPE_FUNCTION = 1;
        static SUBTYPE_BYTE_ARRAY = 2;
        static SUBTYPE_UUID_OLD = 3;
        static SUBTYPE_UUID = 4;
        static SUBTYPE_MD5 = 5;
        static SUBTYPE_ENCRYPTED = 6;
        static SUBTYPE_COLUMN = 7;
        static SUBTYPE_SENSITIVE = 8;
        static SUBTYPE_VECTOR = 9;
        static SUBTYPE_USER_DEFINED = 128;
        static VECTOR_TYPE = Object.freeze({
          Int8: 3,
          Float32: 39,
          PackedBit: 16
        });
        buffer;
        sub_type;
        position;
        constructor(buffer2, subType) {
          super();
          if (!(buffer2 == null) && typeof buffer2 === "string" && !ArrayBuffer.isView(buffer2) && !isAnyArrayBuffer(buffer2) && !Array.isArray(buffer2)) {
            throw new BSONError("Binary can only be constructed from Uint8Array or number[]");
          }
          this.sub_type = subType ?? _Binary.BSON_BINARY_SUBTYPE_DEFAULT;
          if (buffer2 == null) {
            this.buffer = ByteUtils.allocate(_Binary.BUFFER_SIZE);
            this.position = 0;
          } else {
            this.buffer = Array.isArray(buffer2) ? ByteUtils.fromNumberArray(buffer2) : ByteUtils.toLocalBufferType(buffer2);
            this.position = this.buffer.byteLength;
          }
        }
        put(byteValue) {
          if (typeof byteValue === "string" && byteValue.length !== 1) {
            throw new BSONError("only accepts single character String");
          } else if (typeof byteValue !== "number" && byteValue.length !== 1)
            throw new BSONError("only accepts single character Uint8Array or Array");
          let decodedByte;
          if (typeof byteValue === "string") {
            decodedByte = byteValue.charCodeAt(0);
          } else if (typeof byteValue === "number") {
            decodedByte = byteValue;
          } else {
            decodedByte = byteValue[0];
          }
          if (decodedByte < 0 || decodedByte > 255) {
            throw new BSONError("only accepts number in a valid unsigned byte range 0-255");
          }
          if (this.buffer.byteLength > this.position) {
            this.buffer[this.position++] = decodedByte;
          } else {
            const newSpace = ByteUtils.allocate(_Binary.BUFFER_SIZE + this.buffer.length);
            newSpace.set(this.buffer, 0);
            this.buffer = newSpace;
            this.buffer[this.position++] = decodedByte;
          }
        }
        write(sequence, offset) {
          offset = typeof offset === "number" ? offset : this.position;
          if (this.buffer.byteLength < offset + sequence.length) {
            const newSpace = ByteUtils.allocate(this.buffer.byteLength + sequence.length);
            newSpace.set(this.buffer, 0);
            this.buffer = newSpace;
          }
          if (ArrayBuffer.isView(sequence)) {
            this.buffer.set(ByteUtils.toLocalBufferType(sequence), offset);
            this.position = offset + sequence.byteLength > this.position ? offset + sequence.length : this.position;
          } else if (typeof sequence === "string") {
            throw new BSONError("input cannot be string");
          }
        }
        read(position, length) {
          length = length && length > 0 ? length : this.position;
          const end = position + length;
          return this.buffer.subarray(position, end > this.position ? this.position : end);
        }
        value() {
          return this.buffer.length === this.position ? this.buffer : this.buffer.subarray(0, this.position);
        }
        length() {
          return this.position;
        }
        toJSON() {
          return ByteUtils.toBase64(this.buffer.subarray(0, this.position));
        }
        toString(encoding) {
          if (encoding === "hex")
            return ByteUtils.toHex(this.buffer.subarray(0, this.position));
          if (encoding === "base64")
            return ByteUtils.toBase64(this.buffer.subarray(0, this.position));
          if (encoding === "utf8" || encoding === "utf-8")
            return ByteUtils.toUTF8(this.buffer, 0, this.position, false);
          return ByteUtils.toUTF8(this.buffer, 0, this.position, false);
        }
        toExtendedJSON(options) {
          options = options || {};
          if (this.sub_type === _Binary.SUBTYPE_VECTOR) {
            validateBinaryVector(this);
          }
          const base64String = ByteUtils.toBase64(this.buffer);
          const subType = Number(this.sub_type).toString(16);
          if (options.legacy) {
            return {
              $binary: base64String,
              $type: subType.length === 1 ? "0" + subType : subType
            };
          }
          return {
            $binary: {
              base64: base64String,
              subType: subType.length === 1 ? "0" + subType : subType
            }
          };
        }
        toUUID() {
          if (this.sub_type === _Binary.SUBTYPE_UUID) {
            return new UUID(this.buffer.subarray(0, this.position));
          }
          throw new BSONError(`Binary sub_type "${this.sub_type}" is not supported for converting to UUID. Only "${_Binary.SUBTYPE_UUID}" is currently supported.`);
        }
        static createFromHexString(hex, subType) {
          return new _Binary(ByteUtils.fromHex(hex), subType);
        }
        static createFromBase64(base64, subType) {
          return new _Binary(ByteUtils.fromBase64(base64), subType);
        }
        static fromExtendedJSON(doc, options) {
          options = options || {};
          let data;
          let type;
          if ("$binary" in doc) {
            if (options.legacy && typeof doc.$binary === "string" && "$type" in doc) {
              type = doc.$type ? parseInt(doc.$type, 16) : 0;
              data = ByteUtils.fromBase64(doc.$binary);
            } else {
              if (typeof doc.$binary !== "string") {
                type = doc.$binary.subType ? parseInt(doc.$binary.subType, 16) : 0;
                data = ByteUtils.fromBase64(doc.$binary.base64);
              }
            }
          } else if ("$uuid" in doc) {
            type = 4;
            data = UUID.bytesFromString(doc.$uuid);
          }
          if (!data) {
            throw new BSONError(`Unexpected Binary Extended JSON format ${JSON.stringify(doc)}`);
          }
          return type === BSON_BINARY_SUBTYPE_UUID_NEW ? new UUID(data) : new _Binary(data, type);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          const base64 = ByteUtils.toBase64(this.buffer.subarray(0, this.position));
          const base64Arg = inspect(base64, options);
          const subTypeArg = inspect(this.sub_type, options);
          return `Binary.createFromBase64(${base64Arg}, ${subTypeArg})`;
        }
        toInt8Array() {
          if (this.sub_type !== _Binary.SUBTYPE_VECTOR) {
            throw new BSONError("Binary sub_type is not Vector");
          }
          if (this.buffer[0] !== _Binary.VECTOR_TYPE.Int8) {
            throw new BSONError("Binary datatype field is not Int8");
          }
          validateBinaryVector(this);
          return new Int8Array(this.buffer.buffer.slice(this.buffer.byteOffset + 2, this.buffer.byteOffset + this.position));
        }
        toFloat32Array() {
          if (this.sub_type !== _Binary.SUBTYPE_VECTOR) {
            throw new BSONError("Binary sub_type is not Vector");
          }
          if (this.buffer[0] !== _Binary.VECTOR_TYPE.Float32) {
            throw new BSONError("Binary datatype field is not Float32");
          }
          validateBinaryVector(this);
          const floatBytes = new Uint8Array(this.buffer.buffer.slice(this.buffer.byteOffset + 2, this.buffer.byteOffset + this.position));
          if (NumberUtils.isBigEndian)
            ByteUtils.swap32(floatBytes);
          return new Float32Array(floatBytes.buffer);
        }
        toPackedBits() {
          if (this.sub_type !== _Binary.SUBTYPE_VECTOR) {
            throw new BSONError("Binary sub_type is not Vector");
          }
          if (this.buffer[0] !== _Binary.VECTOR_TYPE.PackedBit) {
            throw new BSONError("Binary datatype field is not packed bit");
          }
          validateBinaryVector(this);
          return new Uint8Array(this.buffer.buffer.slice(this.buffer.byteOffset + 2, this.buffer.byteOffset + this.position));
        }
        toBits() {
          if (this.sub_type !== _Binary.SUBTYPE_VECTOR) {
            throw new BSONError("Binary sub_type is not Vector");
          }
          if (this.buffer[0] !== _Binary.VECTOR_TYPE.PackedBit) {
            throw new BSONError("Binary datatype field is not packed bit");
          }
          validateBinaryVector(this);
          const byteCount = this.length() - 2;
          const bitCount = byteCount * 8 - this.buffer[1];
          const bits2 = new Int8Array(bitCount);
          for (let bitOffset = 0; bitOffset < bits2.length; bitOffset++) {
            const byteOffset = bitOffset / 8 | 0;
            const byte = this.buffer[byteOffset + 2];
            const shift = 7 - bitOffset % 8;
            const bit = byte >> shift & 1;
            bits2[bitOffset] = bit;
          }
          return bits2;
        }
        static fromInt8Array(array) {
          const buffer2 = ByteUtils.allocate(array.byteLength + 2);
          buffer2[0] = _Binary.VECTOR_TYPE.Int8;
          buffer2[1] = 0;
          const intBytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
          buffer2.set(intBytes, 2);
          const bin = new this(buffer2, this.SUBTYPE_VECTOR);
          validateBinaryVector(bin);
          return bin;
        }
        static fromFloat32Array(array) {
          const binaryBytes = ByteUtils.allocate(array.byteLength + 2);
          binaryBytes[0] = _Binary.VECTOR_TYPE.Float32;
          binaryBytes[1] = 0;
          const floatBytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
          binaryBytes.set(floatBytes, 2);
          if (NumberUtils.isBigEndian)
            ByteUtils.swap32(new Uint8Array(binaryBytes.buffer, 2));
          const bin = new this(binaryBytes, this.SUBTYPE_VECTOR);
          validateBinaryVector(bin);
          return bin;
        }
        static fromPackedBits(array, padding = 0) {
          const buffer2 = ByteUtils.allocate(array.byteLength + 2);
          buffer2[0] = _Binary.VECTOR_TYPE.PackedBit;
          buffer2[1] = padding;
          buffer2.set(array, 2);
          const bin = new this(buffer2, this.SUBTYPE_VECTOR);
          validateBinaryVector(bin);
          return bin;
        }
        static fromBits(bits2) {
          const byteLength = bits2.length + 7 >>> 3;
          const bytes = new Uint8Array(byteLength + 2);
          bytes[0] = _Binary.VECTOR_TYPE.PackedBit;
          const remainder = bits2.length % 8;
          bytes[1] = remainder === 0 ? 0 : 8 - remainder;
          for (let bitOffset = 0; bitOffset < bits2.length; bitOffset++) {
            const byteOffset = bitOffset >>> 3;
            const bit = bits2[bitOffset];
            if (bit !== 0 && bit !== 1) {
              throw new BSONError(`Invalid bit value at ${bitOffset}: must be 0 or 1, found ${bits2[bitOffset]}`);
            }
            if (bit === 0)
              continue;
            const shift = 7 - bitOffset % 8;
            bytes[byteOffset + 2] |= bit << shift;
          }
          return new this(bytes, _Binary.SUBTYPE_VECTOR);
        }
      };
      UUID_BYTE_LENGTH = 16;
      UUID_WITHOUT_DASHES = /^[0-9A-F]{32}$/i;
      UUID_WITH_DASHES = /^[0-9A-F]{8}-[0-9A-F]{4}-[0-9A-F]{4}-[0-9A-F]{4}-[0-9A-F]{12}$/i;
      UUID = class _UUID extends Binary {
        constructor(input) {
          let bytes;
          if (input == null) {
            bytes = _UUID.generate();
          } else if (input instanceof _UUID) {
            bytes = ByteUtils.toLocalBufferType(new Uint8Array(input.buffer));
          } else if (ArrayBuffer.isView(input) && input.byteLength === UUID_BYTE_LENGTH) {
            bytes = ByteUtils.toLocalBufferType(input);
          } else if (typeof input === "string") {
            bytes = _UUID.bytesFromString(input);
          } else {
            throw new BSONError("Argument passed in UUID constructor must be a UUID, a 16 byte Buffer or a 32/36 character hex string (dashes excluded/included, format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx).");
          }
          super(bytes, BSON_BINARY_SUBTYPE_UUID_NEW);
        }
        get id() {
          return this.buffer;
        }
        set id(value) {
          this.buffer = value;
        }
        toHexString(includeDashes = true) {
          if (includeDashes) {
            return [
              ByteUtils.toHex(this.buffer.subarray(0, 4)),
              ByteUtils.toHex(this.buffer.subarray(4, 6)),
              ByteUtils.toHex(this.buffer.subarray(6, 8)),
              ByteUtils.toHex(this.buffer.subarray(8, 10)),
              ByteUtils.toHex(this.buffer.subarray(10, 16))
            ].join("-");
          }
          return ByteUtils.toHex(this.buffer);
        }
        toString(encoding) {
          if (encoding === "hex")
            return ByteUtils.toHex(this.id);
          if (encoding === "base64")
            return ByteUtils.toBase64(this.id);
          return this.toHexString();
        }
        toJSON() {
          return this.toHexString();
        }
        equals(otherId) {
          if (!otherId) {
            return false;
          }
          if (otherId instanceof _UUID) {
            return ByteUtils.equals(otherId.id, this.id);
          }
          try {
            return ByteUtils.equals(new _UUID(otherId).id, this.id);
          } catch {
            return false;
          }
        }
        toBinary() {
          return new Binary(this.id, Binary.SUBTYPE_UUID);
        }
        static generate() {
          const bytes = ByteUtils.randomBytes(UUID_BYTE_LENGTH);
          bytes[6] = bytes[6] & 15 | 64;
          bytes[8] = bytes[8] & 63 | 128;
          return bytes;
        }
        static isValid(input) {
          if (!input) {
            return false;
          }
          if (typeof input === "string") {
            return _UUID.isValidUUIDString(input);
          }
          if (isUint8Array(input)) {
            return input.byteLength === UUID_BYTE_LENGTH;
          }
          return input._bsontype === "Binary" && input.sub_type === this.SUBTYPE_UUID && input.buffer.byteLength === 16;
        }
        static createFromHexString(hexString) {
          const buffer2 = _UUID.bytesFromString(hexString);
          return new _UUID(buffer2);
        }
        static createFromBase64(base64) {
          return new _UUID(ByteUtils.fromBase64(base64));
        }
        static bytesFromString(representation) {
          if (!_UUID.isValidUUIDString(representation)) {
            throw new BSONError("UUID string representation must be 32 hex digits or canonical hyphenated representation");
          }
          return ByteUtils.fromHex(representation.replace(/-/g, ""));
        }
        static isValidUUIDString(representation) {
          return UUID_WITHOUT_DASHES.test(representation) || UUID_WITH_DASHES.test(representation);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          return `new UUID(${inspect(this.toHexString(), options)})`;
        }
      };
      Code = class _Code extends BSONValue {
        get _bsontype() {
          return "Code";
        }
        code;
        scope;
        constructor(code, scope) {
          super();
          this.code = code.toString();
          this.scope = scope ?? null;
        }
        toJSON() {
          if (this.scope != null) {
            return { code: this.code, scope: this.scope };
          }
          return { code: this.code };
        }
        toExtendedJSON() {
          if (this.scope) {
            return { $code: this.code, $scope: this.scope };
          }
          return { $code: this.code };
        }
        static fromExtendedJSON(doc) {
          return new _Code(doc.$code, doc.$scope);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          let parametersString = inspect(this.code, options);
          const multiLineFn = parametersString.includes("\n");
          if (this.scope != null) {
            parametersString += `,${multiLineFn ? "\n" : " "}${inspect(this.scope, options)}`;
          }
          const endingNewline = multiLineFn && this.scope === null;
          return `new Code(${multiLineFn ? "\n" : ""}${parametersString}${endingNewline ? "\n" : ""})`;
        }
      };
      DBRef = class _DBRef extends BSONValue {
        get _bsontype() {
          return "DBRef";
        }
        collection;
        oid;
        db;
        fields;
        constructor(collection, oid, db, fields) {
          super();
          const parts = collection.split(".");
          if (parts.length === 2) {
            db = parts.shift();
            collection = parts.shift();
          }
          this.collection = collection;
          this.oid = oid;
          this.db = db;
          this.fields = fields || {};
        }
        get namespace() {
          return this.collection;
        }
        set namespace(value) {
          this.collection = value;
        }
        toJSON() {
          const o5 = Object.assign({
            $ref: this.collection,
            $id: this.oid
          }, this.fields);
          if (this.db != null)
            o5.$db = this.db;
          return o5;
        }
        toExtendedJSON(options) {
          options = options || {};
          let o5 = {
            $ref: this.collection,
            $id: this.oid
          };
          if (options.legacy) {
            return o5;
          }
          if (this.db)
            o5.$db = this.db;
          o5 = Object.assign(o5, this.fields);
          return o5;
        }
        static fromExtendedJSON(doc) {
          const copy = Object.assign({}, doc);
          delete copy.$ref;
          delete copy.$id;
          delete copy.$db;
          return new _DBRef(doc.$ref, doc.$id, doc.$db, copy);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          const args = [
            inspect(this.namespace, options),
            inspect(this.oid, options),
            ...this.db ? [inspect(this.db, options)] : [],
            ...Object.keys(this.fields).length > 0 ? [inspect(this.fields, options)] : []
          ];
          args[1] = inspect === defaultInspect ? `new ObjectId(${args[1]})` : args[1];
          return `new DBRef(${args.join(", ")})`;
        }
      };
      wasm = void 0;
      try {
        wasm = new WebAssembly.Instance(new WebAssembly.Module(new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 1, 13, 2, 96, 0, 1, 127, 96, 4, 127, 127, 127, 127, 1, 127, 3, 7, 6, 0, 1, 1, 1, 1, 1, 6, 6, 1, 127, 1, 65, 0, 11, 7, 50, 6, 3, 109, 117, 108, 0, 1, 5, 100, 105, 118, 95, 115, 0, 2, 5, 100, 105, 118, 95, 117, 0, 3, 5, 114, 101, 109, 95, 115, 0, 4, 5, 114, 101, 109, 95, 117, 0, 5, 8, 103, 101, 116, 95, 104, 105, 103, 104, 0, 0, 10, 191, 1, 6, 4, 0, 35, 0, 11, 36, 1, 1, 126, 32, 0, 173, 32, 1, 173, 66, 32, 134, 132, 32, 2, 173, 32, 3, 173, 66, 32, 134, 132, 126, 34, 4, 66, 32, 135, 167, 36, 0, 32, 4, 167, 11, 36, 1, 1, 126, 32, 0, 173, 32, 1, 173, 66, 32, 134, 132, 32, 2, 173, 32, 3, 173, 66, 32, 134, 132, 127, 34, 4, 66, 32, 135, 167, 36, 0, 32, 4, 167, 11, 36, 1, 1, 126, 32, 0, 173, 32, 1, 173, 66, 32, 134, 132, 32, 2, 173, 32, 3, 173, 66, 32, 134, 132, 128, 34, 4, 66, 32, 135, 167, 36, 0, 32, 4, 167, 11, 36, 1, 1, 126, 32, 0, 173, 32, 1, 173, 66, 32, 134, 132, 32, 2, 173, 32, 3, 173, 66, 32, 134, 132, 129, 34, 4, 66, 32, 135, 167, 36, 0, 32, 4, 167, 11, 36, 1, 1, 126, 32, 0, 173, 32, 1, 173, 66, 32, 134, 132, 32, 2, 173, 32, 3, 173, 66, 32, 134, 132, 130, 34, 4, 66, 32, 135, 167, 36, 0, 32, 4, 167, 11])), {}).exports;
      } catch {
      }
      TWO_PWR_16_DBL = 1 << 16;
      TWO_PWR_24_DBL = 1 << 24;
      TWO_PWR_32_DBL = TWO_PWR_16_DBL * TWO_PWR_16_DBL;
      TWO_PWR_64_DBL = TWO_PWR_32_DBL * TWO_PWR_32_DBL;
      TWO_PWR_63_DBL = TWO_PWR_64_DBL / 2;
      INT_CACHE = {};
      UINT_CACHE = {};
      MAX_INT64_STRING_LENGTH = 20;
      DECIMAL_REG_EX = /^(\+?0|(\+|-)?[1-9][0-9]*)$/;
      Long = class _Long extends BSONValue {
        get _bsontype() {
          return "Long";
        }
        get __isLong__() {
          return true;
        }
        high;
        low;
        unsigned;
        constructor(lowOrValue = 0, highOrUnsigned, unsigned) {
          super();
          const unsignedBool = typeof highOrUnsigned === "boolean" ? highOrUnsigned : Boolean(unsigned);
          const high = typeof highOrUnsigned === "number" ? highOrUnsigned : 0;
          const res = typeof lowOrValue === "string" ? _Long.fromString(lowOrValue, unsignedBool) : typeof lowOrValue === "bigint" ? _Long.fromBigInt(lowOrValue, unsignedBool) : { low: lowOrValue | 0, high: high | 0, unsigned: unsignedBool };
          this.low = res.low;
          this.high = res.high;
          this.unsigned = res.unsigned;
        }
        static TWO_PWR_24 = _Long.fromInt(TWO_PWR_24_DBL);
        static MAX_UNSIGNED_VALUE = _Long.fromBits(4294967295 | 0, 4294967295 | 0, true);
        static ZERO = _Long.fromInt(0);
        static UZERO = _Long.fromInt(0, true);
        static ONE = _Long.fromInt(1);
        static UONE = _Long.fromInt(1, true);
        static NEG_ONE = _Long.fromInt(-1);
        static MAX_VALUE = _Long.fromBits(4294967295 | 0, 2147483647 | 0, false);
        static MIN_VALUE = _Long.fromBits(0, 2147483648 | 0, false);
        static fromBits(lowBits, highBits, unsigned) {
          return new _Long(lowBits, highBits, unsigned);
        }
        static fromInt(value, unsigned) {
          let obj, cachedObj, cache;
          if (unsigned) {
            value >>>= 0;
            if (cache = 0 <= value && value < 256) {
              cachedObj = UINT_CACHE[value];
              if (cachedObj)
                return cachedObj;
            }
            obj = _Long.fromBits(value, (value | 0) < 0 ? -1 : 0, true);
            if (cache)
              UINT_CACHE[value] = obj;
            return obj;
          } else {
            value |= 0;
            if (cache = -128 <= value && value < 128) {
              cachedObj = INT_CACHE[value];
              if (cachedObj)
                return cachedObj;
            }
            obj = _Long.fromBits(value, value < 0 ? -1 : 0, false);
            if (cache)
              INT_CACHE[value] = obj;
            return obj;
          }
        }
        static fromNumber(value, unsigned) {
          if (isNaN(value))
            return unsigned ? _Long.UZERO : _Long.ZERO;
          if (unsigned) {
            if (value < 0)
              return _Long.UZERO;
            if (value >= TWO_PWR_64_DBL)
              return _Long.MAX_UNSIGNED_VALUE;
          } else {
            if (value <= -TWO_PWR_63_DBL)
              return _Long.MIN_VALUE;
            if (value + 1 >= TWO_PWR_63_DBL)
              return _Long.MAX_VALUE;
          }
          if (value < 0)
            return _Long.fromNumber(-value, unsigned).neg();
          return _Long.fromBits(value % TWO_PWR_32_DBL | 0, value / TWO_PWR_32_DBL | 0, unsigned);
        }
        static fromBigInt(value, unsigned) {
          const FROM_BIGINT_BIT_MASK = 0xffffffffn;
          const FROM_BIGINT_BIT_SHIFT = 32n;
          return new _Long(Number(value & FROM_BIGINT_BIT_MASK), Number(value >> FROM_BIGINT_BIT_SHIFT & FROM_BIGINT_BIT_MASK), unsigned);
        }
        static _fromString(str, unsigned, radix) {
          if (str.length === 0)
            throw new BSONError("empty string");
          if (radix < 2 || 36 < radix)
            throw new BSONError("radix");
          let p5;
          if ((p5 = str.indexOf("-")) > 0)
            throw new BSONError("interior hyphen");
          else if (p5 === 0) {
            return _Long._fromString(str.substring(1), unsigned, radix).neg();
          }
          const radixToPower = _Long.fromNumber(Math.pow(radix, 8));
          let result = _Long.ZERO;
          for (let i2 = 0; i2 < str.length; i2 += 8) {
            const size = Math.min(8, str.length - i2), value = parseInt(str.substring(i2, i2 + size), radix);
            if (size < 8) {
              const power = _Long.fromNumber(Math.pow(radix, size));
              result = result.mul(power).add(_Long.fromNumber(value));
            } else {
              result = result.mul(radixToPower);
              result = result.add(_Long.fromNumber(value));
            }
          }
          result.unsigned = unsigned;
          return result;
        }
        static fromStringStrict(str, unsignedOrRadix, radix) {
          let unsigned = false;
          if (typeof unsignedOrRadix === "number") {
            radix = unsignedOrRadix, unsignedOrRadix = false;
          } else {
            unsigned = !!unsignedOrRadix;
          }
          radix ??= 10;
          if (str.trim() !== str) {
            throw new BSONError(`Input: '${str}' contains leading and/or trailing whitespace`);
          }
          if (!validateStringCharacters(str, radix)) {
            throw new BSONError(`Input: '${str}' contains invalid characters for radix: ${radix}`);
          }
          const cleanedStr = removeLeadingZerosAndExplicitPlus(str);
          const result = _Long._fromString(cleanedStr, unsigned, radix);
          if (result.toString(radix).toLowerCase() !== cleanedStr.toLowerCase()) {
            throw new BSONError(`Input: ${str} is not representable as ${result.unsigned ? "an unsigned" : "a signed"} 64-bit Long ${radix != null ? `with radix: ${radix}` : ""}`);
          }
          return result;
        }
        static fromString(str, unsignedOrRadix, radix) {
          let unsigned = false;
          if (typeof unsignedOrRadix === "number") {
            radix = unsignedOrRadix, unsignedOrRadix = false;
          } else {
            unsigned = !!unsignedOrRadix;
          }
          radix ??= 10;
          if (str === "NaN" && radix < 24) {
            return _Long.ZERO;
          } else if ((str === "Infinity" || str === "+Infinity" || str === "-Infinity") && radix < 35) {
            return _Long.ZERO;
          }
          return _Long._fromString(str, unsigned, radix);
        }
        static fromBytes(bytes, unsigned, le) {
          return le ? _Long.fromBytesLE(bytes, unsigned) : _Long.fromBytesBE(bytes, unsigned);
        }
        static fromBytesLE(bytes, unsigned) {
          return new _Long(bytes[0] | bytes[1] << 8 | bytes[2] << 16 | bytes[3] << 24, bytes[4] | bytes[5] << 8 | bytes[6] << 16 | bytes[7] << 24, unsigned);
        }
        static fromBytesBE(bytes, unsigned) {
          return new _Long(bytes[4] << 24 | bytes[5] << 16 | bytes[6] << 8 | bytes[7], bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3], unsigned);
        }
        static isLong(value) {
          return value != null && typeof value === "object" && "__isLong__" in value && value.__isLong__ === true;
        }
        static fromValue(val, unsigned) {
          if (typeof val === "number")
            return _Long.fromNumber(val, unsigned);
          if (typeof val === "string")
            return _Long.fromString(val, unsigned);
          return _Long.fromBits(val.low, val.high, typeof unsigned === "boolean" ? unsigned : val.unsigned);
        }
        add(addend) {
          if (!_Long.isLong(addend))
            addend = _Long.fromValue(addend);
          const a48 = this.high >>> 16;
          const a32 = this.high & 65535;
          const a16 = this.low >>> 16;
          const a00 = this.low & 65535;
          const b48 = addend.high >>> 16;
          const b32 = addend.high & 65535;
          const b16 = addend.low >>> 16;
          const b00 = addend.low & 65535;
          let c48 = 0, c32 = 0, c16 = 0, c00 = 0;
          c00 += a00 + b00;
          c16 += c00 >>> 16;
          c00 &= 65535;
          c16 += a16 + b16;
          c32 += c16 >>> 16;
          c16 &= 65535;
          c32 += a32 + b32;
          c48 += c32 >>> 16;
          c32 &= 65535;
          c48 += a48 + b48;
          c48 &= 65535;
          return _Long.fromBits(c16 << 16 | c00, c48 << 16 | c32, this.unsigned);
        }
        and(other) {
          if (!_Long.isLong(other))
            other = _Long.fromValue(other);
          return _Long.fromBits(this.low & other.low, this.high & other.high, this.unsigned);
        }
        compare(other) {
          if (!_Long.isLong(other))
            other = _Long.fromValue(other);
          if (this.eq(other))
            return 0;
          const thisNeg = this.isNegative(), otherNeg = other.isNegative();
          if (thisNeg && !otherNeg)
            return -1;
          if (!thisNeg && otherNeg)
            return 1;
          if (!this.unsigned)
            return this.sub(other).isNegative() ? -1 : 1;
          return other.high >>> 0 > this.high >>> 0 || other.high === this.high && other.low >>> 0 > this.low >>> 0 ? -1 : 1;
        }
        comp(other) {
          return this.compare(other);
        }
        divide(divisor) {
          if (!_Long.isLong(divisor))
            divisor = _Long.fromValue(divisor);
          if (divisor.isZero())
            throw new BSONError("division by zero");
          if (wasm) {
            if (!this.unsigned && this.high === -2147483648 && divisor.low === -1 && divisor.high === -1) {
              return this;
            }
            const low = (this.unsigned ? wasm.div_u : wasm.div_s)(this.low, this.high, divisor.low, divisor.high);
            return _Long.fromBits(low, wasm.get_high(), this.unsigned);
          }
          if (this.isZero())
            return this.unsigned ? _Long.UZERO : _Long.ZERO;
          let approx, rem, res;
          if (!this.unsigned) {
            if (this.eq(_Long.MIN_VALUE)) {
              if (divisor.eq(_Long.ONE) || divisor.eq(_Long.NEG_ONE))
                return _Long.MIN_VALUE;
              else if (divisor.eq(_Long.MIN_VALUE))
                return _Long.ONE;
              else {
                const halfThis = this.shr(1);
                approx = halfThis.div(divisor).shl(1);
                if (approx.eq(_Long.ZERO)) {
                  return divisor.isNegative() ? _Long.ONE : _Long.NEG_ONE;
                } else {
                  rem = this.sub(divisor.mul(approx));
                  res = approx.add(rem.div(divisor));
                  return res;
                }
              }
            } else if (divisor.eq(_Long.MIN_VALUE))
              return this.unsigned ? _Long.UZERO : _Long.ZERO;
            if (this.isNegative()) {
              if (divisor.isNegative())
                return this.neg().div(divisor.neg());
              return this.neg().div(divisor).neg();
            } else if (divisor.isNegative())
              return this.div(divisor.neg()).neg();
            res = _Long.ZERO;
          } else {
            if (!divisor.unsigned)
              divisor = divisor.toUnsigned();
            if (divisor.gt(this))
              return _Long.UZERO;
            if (divisor.gt(this.shru(1)))
              return _Long.UONE;
            res = _Long.UZERO;
          }
          rem = this;
          while (rem.gte(divisor)) {
            approx = Math.max(1, Math.floor(rem.toNumber() / divisor.toNumber()));
            const log2 = Math.ceil(Math.log(approx) / Math.LN2);
            const delta = log2 <= 48 ? 1 : Math.pow(2, log2 - 48);
            let approxRes = _Long.fromNumber(approx);
            let approxRem = approxRes.mul(divisor);
            while (approxRem.isNegative() || approxRem.gt(rem)) {
              approx -= delta;
              approxRes = _Long.fromNumber(approx, this.unsigned);
              approxRem = approxRes.mul(divisor);
            }
            if (approxRes.isZero())
              approxRes = _Long.ONE;
            res = res.add(approxRes);
            rem = rem.sub(approxRem);
          }
          return res;
        }
        div(divisor) {
          return this.divide(divisor);
        }
        equals(other) {
          if (!_Long.isLong(other))
            other = _Long.fromValue(other);
          if (this.unsigned !== other.unsigned && this.high >>> 31 === 1 && other.high >>> 31 === 1)
            return false;
          return this.high === other.high && this.low === other.low;
        }
        eq(other) {
          return this.equals(other);
        }
        getHighBits() {
          return this.high;
        }
        getHighBitsUnsigned() {
          return this.high >>> 0;
        }
        getLowBits() {
          return this.low;
        }
        getLowBitsUnsigned() {
          return this.low >>> 0;
        }
        getNumBitsAbs() {
          if (this.isNegative()) {
            return this.eq(_Long.MIN_VALUE) ? 64 : this.neg().getNumBitsAbs();
          }
          const val = this.high !== 0 ? this.high : this.low;
          let bit;
          for (bit = 31; bit > 0; bit--)
            if ((val & 1 << bit) !== 0)
              break;
          return this.high !== 0 ? bit + 33 : bit + 1;
        }
        greaterThan(other) {
          return this.comp(other) > 0;
        }
        gt(other) {
          return this.greaterThan(other);
        }
        greaterThanOrEqual(other) {
          return this.comp(other) >= 0;
        }
        gte(other) {
          return this.greaterThanOrEqual(other);
        }
        ge(other) {
          return this.greaterThanOrEqual(other);
        }
        isEven() {
          return (this.low & 1) === 0;
        }
        isNegative() {
          return !this.unsigned && this.high < 0;
        }
        isOdd() {
          return (this.low & 1) === 1;
        }
        isPositive() {
          return this.unsigned || this.high >= 0;
        }
        isZero() {
          return this.high === 0 && this.low === 0;
        }
        lessThan(other) {
          return this.comp(other) < 0;
        }
        lt(other) {
          return this.lessThan(other);
        }
        lessThanOrEqual(other) {
          return this.comp(other) <= 0;
        }
        lte(other) {
          return this.lessThanOrEqual(other);
        }
        modulo(divisor) {
          if (!_Long.isLong(divisor))
            divisor = _Long.fromValue(divisor);
          if (wasm) {
            const low = (this.unsigned ? wasm.rem_u : wasm.rem_s)(this.low, this.high, divisor.low, divisor.high);
            return _Long.fromBits(low, wasm.get_high(), this.unsigned);
          }
          return this.sub(this.div(divisor).mul(divisor));
        }
        mod(divisor) {
          return this.modulo(divisor);
        }
        rem(divisor) {
          return this.modulo(divisor);
        }
        multiply(multiplier) {
          if (this.isZero())
            return _Long.ZERO;
          if (!_Long.isLong(multiplier))
            multiplier = _Long.fromValue(multiplier);
          if (wasm) {
            const low = wasm.mul(this.low, this.high, multiplier.low, multiplier.high);
            return _Long.fromBits(low, wasm.get_high(), this.unsigned);
          }
          if (multiplier.isZero())
            return _Long.ZERO;
          if (this.eq(_Long.MIN_VALUE))
            return multiplier.isOdd() ? _Long.MIN_VALUE : _Long.ZERO;
          if (multiplier.eq(_Long.MIN_VALUE))
            return this.isOdd() ? _Long.MIN_VALUE : _Long.ZERO;
          if (this.isNegative()) {
            if (multiplier.isNegative())
              return this.neg().mul(multiplier.neg());
            else
              return this.neg().mul(multiplier).neg();
          } else if (multiplier.isNegative())
            return this.mul(multiplier.neg()).neg();
          if (this.lt(_Long.TWO_PWR_24) && multiplier.lt(_Long.TWO_PWR_24))
            return _Long.fromNumber(this.toNumber() * multiplier.toNumber(), this.unsigned);
          const a48 = this.high >>> 16;
          const a32 = this.high & 65535;
          const a16 = this.low >>> 16;
          const a00 = this.low & 65535;
          const b48 = multiplier.high >>> 16;
          const b32 = multiplier.high & 65535;
          const b16 = multiplier.low >>> 16;
          const b00 = multiplier.low & 65535;
          let c48 = 0, c32 = 0, c16 = 0, c00 = 0;
          c00 += a00 * b00;
          c16 += c00 >>> 16;
          c00 &= 65535;
          c16 += a16 * b00;
          c32 += c16 >>> 16;
          c16 &= 65535;
          c16 += a00 * b16;
          c32 += c16 >>> 16;
          c16 &= 65535;
          c32 += a32 * b00;
          c48 += c32 >>> 16;
          c32 &= 65535;
          c32 += a16 * b16;
          c48 += c32 >>> 16;
          c32 &= 65535;
          c32 += a00 * b32;
          c48 += c32 >>> 16;
          c32 &= 65535;
          c48 += a48 * b00 + a32 * b16 + a16 * b32 + a00 * b48;
          c48 &= 65535;
          return _Long.fromBits(c16 << 16 | c00, c48 << 16 | c32, this.unsigned);
        }
        mul(multiplier) {
          return this.multiply(multiplier);
        }
        negate() {
          if (!this.unsigned && this.eq(_Long.MIN_VALUE))
            return _Long.MIN_VALUE;
          return this.not().add(_Long.ONE);
        }
        neg() {
          return this.negate();
        }
        not() {
          return _Long.fromBits(~this.low, ~this.high, this.unsigned);
        }
        notEquals(other) {
          return !this.equals(other);
        }
        neq(other) {
          return this.notEquals(other);
        }
        ne(other) {
          return this.notEquals(other);
        }
        or(other) {
          if (!_Long.isLong(other))
            other = _Long.fromValue(other);
          return _Long.fromBits(this.low | other.low, this.high | other.high, this.unsigned);
        }
        shiftLeft(numBits) {
          if (_Long.isLong(numBits))
            numBits = numBits.toInt();
          if ((numBits &= 63) === 0)
            return this;
          else if (numBits < 32)
            return _Long.fromBits(this.low << numBits, this.high << numBits | this.low >>> 32 - numBits, this.unsigned);
          else
            return _Long.fromBits(0, this.low << numBits - 32, this.unsigned);
        }
        shl(numBits) {
          return this.shiftLeft(numBits);
        }
        shiftRight(numBits) {
          if (_Long.isLong(numBits))
            numBits = numBits.toInt();
          if ((numBits &= 63) === 0)
            return this;
          else if (numBits < 32)
            return _Long.fromBits(this.low >>> numBits | this.high << 32 - numBits, this.high >> numBits, this.unsigned);
          else
            return _Long.fromBits(this.high >> numBits - 32, this.high >= 0 ? 0 : -1, this.unsigned);
        }
        shr(numBits) {
          return this.shiftRight(numBits);
        }
        shiftRightUnsigned(numBits) {
          if (_Long.isLong(numBits))
            numBits = numBits.toInt();
          numBits &= 63;
          if (numBits === 0)
            return this;
          else {
            const high = this.high;
            if (numBits < 32) {
              const low = this.low;
              return _Long.fromBits(low >>> numBits | high << 32 - numBits, high >>> numBits, this.unsigned);
            } else if (numBits === 32)
              return _Long.fromBits(high, 0, this.unsigned);
            else
              return _Long.fromBits(high >>> numBits - 32, 0, this.unsigned);
          }
        }
        shr_u(numBits) {
          return this.shiftRightUnsigned(numBits);
        }
        shru(numBits) {
          return this.shiftRightUnsigned(numBits);
        }
        subtract(subtrahend) {
          if (!_Long.isLong(subtrahend))
            subtrahend = _Long.fromValue(subtrahend);
          return this.add(subtrahend.neg());
        }
        sub(subtrahend) {
          return this.subtract(subtrahend);
        }
        toInt() {
          return this.unsigned ? this.low >>> 0 : this.low;
        }
        toNumber() {
          if (this.unsigned)
            return (this.high >>> 0) * TWO_PWR_32_DBL + (this.low >>> 0);
          return this.high * TWO_PWR_32_DBL + (this.low >>> 0);
        }
        toBigInt() {
          return BigInt(this.toString());
        }
        toBytes(le) {
          return le ? this.toBytesLE() : this.toBytesBE();
        }
        toBytesLE() {
          const hi = this.high, lo = this.low;
          return [
            lo & 255,
            lo >>> 8 & 255,
            lo >>> 16 & 255,
            lo >>> 24,
            hi & 255,
            hi >>> 8 & 255,
            hi >>> 16 & 255,
            hi >>> 24
          ];
        }
        toBytesBE() {
          const hi = this.high, lo = this.low;
          return [
            hi >>> 24,
            hi >>> 16 & 255,
            hi >>> 8 & 255,
            hi & 255,
            lo >>> 24,
            lo >>> 16 & 255,
            lo >>> 8 & 255,
            lo & 255
          ];
        }
        toSigned() {
          if (!this.unsigned)
            return this;
          return _Long.fromBits(this.low, this.high, false);
        }
        toString(radix) {
          radix = radix || 10;
          if (radix < 2 || 36 < radix)
            throw new BSONError("radix");
          if (this.isZero())
            return "0";
          if (this.isNegative()) {
            if (this.eq(_Long.MIN_VALUE)) {
              const radixLong = _Long.fromNumber(radix), div = this.div(radixLong), rem1 = div.mul(radixLong).sub(this);
              return div.toString(radix) + rem1.toInt().toString(radix);
            } else
              return "-" + this.neg().toString(radix);
          }
          const radixToPower = _Long.fromNumber(Math.pow(radix, 6), this.unsigned);
          let rem = this;
          let result = "";
          while (true) {
            const remDiv = rem.div(radixToPower);
            const intval = rem.sub(remDiv.mul(radixToPower)).toInt() >>> 0;
            let digits = intval.toString(radix);
            rem = remDiv;
            if (rem.isZero()) {
              return digits + result;
            } else {
              while (digits.length < 6)
                digits = "0" + digits;
              result = "" + digits + result;
            }
          }
        }
        toUnsigned() {
          if (this.unsigned)
            return this;
          return _Long.fromBits(this.low, this.high, true);
        }
        xor(other) {
          if (!_Long.isLong(other))
            other = _Long.fromValue(other);
          return _Long.fromBits(this.low ^ other.low, this.high ^ other.high, this.unsigned);
        }
        eqz() {
          return this.isZero();
        }
        le(other) {
          return this.lessThanOrEqual(other);
        }
        toExtendedJSON(options) {
          if (options && options.relaxed)
            return this.toNumber();
          return { $numberLong: this.toString() };
        }
        static fromExtendedJSON(doc, options) {
          const { useBigInt64 = false, relaxed = true } = { ...options };
          if (doc.$numberLong.length > MAX_INT64_STRING_LENGTH) {
            throw new BSONError("$numberLong string is too long");
          }
          if (!DECIMAL_REG_EX.test(doc.$numberLong)) {
            throw new BSONError(`$numberLong string "${doc.$numberLong}" is in an invalid format`);
          }
          if (useBigInt64) {
            const bigIntResult = BigInt(doc.$numberLong);
            return BigInt.asIntN(64, bigIntResult);
          }
          const longResult = _Long.fromString(doc.$numberLong);
          if (relaxed) {
            return longResult.toNumber();
          }
          return longResult;
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          const longVal = inspect(this.toString(), options);
          const unsignedVal = this.unsigned ? `, ${inspect(this.unsigned, options)}` : "";
          return `new Long(${longVal}${unsignedVal})`;
        }
      };
      PARSE_STRING_REGEXP = /^(\+|-)?(\d+|(\d*\.\d*))?(E|e)?([-+])?(\d+)?$/;
      PARSE_INF_REGEXP = /^(\+|-)?(Infinity|inf)$/i;
      PARSE_NAN_REGEXP = /^(\+|-)?NaN$/i;
      EXPONENT_MAX = 6111;
      EXPONENT_MIN = -6176;
      EXPONENT_BIAS = 6176;
      MAX_DIGITS = 34;
      NAN_BUFFER = ByteUtils.fromNumberArray([
        124,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      ].reverse());
      INF_NEGATIVE_BUFFER = ByteUtils.fromNumberArray([
        248,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      ].reverse());
      INF_POSITIVE_BUFFER = ByteUtils.fromNumberArray([
        120,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      ].reverse());
      EXPONENT_REGEX = /^([-+])?(\d+)?$/;
      COMBINATION_MASK = 31;
      EXPONENT_MASK = 16383;
      COMBINATION_INFINITY = 30;
      COMBINATION_NAN = 31;
      Decimal128 = class _Decimal128 extends BSONValue {
        get _bsontype() {
          return "Decimal128";
        }
        bytes;
        constructor(bytes) {
          super();
          if (typeof bytes === "string") {
            this.bytes = _Decimal128.fromString(bytes).bytes;
          } else if (bytes instanceof Uint8Array || isUint8Array(bytes)) {
            if (bytes.byteLength !== 16) {
              throw new BSONError("Decimal128 must take a Buffer of 16 bytes");
            }
            this.bytes = bytes;
          } else {
            throw new BSONError("Decimal128 must take a Buffer or string");
          }
        }
        static fromString(representation) {
          return _Decimal128._fromString(representation, { allowRounding: false });
        }
        static fromStringWithRounding(representation) {
          return _Decimal128._fromString(representation, { allowRounding: true });
        }
        static _fromString(representation, options) {
          let isNegative = false;
          let sawSign = false;
          let sawRadix = false;
          let foundNonZero = false;
          let significantDigits = 0;
          let nDigitsRead = 0;
          let nDigits = 0;
          let radixPosition = 0;
          let firstNonZero = 0;
          const digits = [0];
          let nDigitsStored = 0;
          let digitsInsert = 0;
          let lastDigit = 0;
          let exponent = 0;
          let significandHigh = new Long(0, 0);
          let significandLow = new Long(0, 0);
          let biasedExponent = 0;
          let index = 0;
          if (representation.length >= 7e3) {
            throw new BSONError("" + representation + " not a valid Decimal128 string");
          }
          const stringMatch = representation.match(PARSE_STRING_REGEXP);
          const infMatch = representation.match(PARSE_INF_REGEXP);
          const nanMatch = representation.match(PARSE_NAN_REGEXP);
          if (!stringMatch && !infMatch && !nanMatch || representation.length === 0) {
            throw new BSONError("" + representation + " not a valid Decimal128 string");
          }
          if (stringMatch) {
            const unsignedNumber = stringMatch[2];
            const e2 = stringMatch[4];
            const expSign = stringMatch[5];
            const expNumber = stringMatch[6];
            if (e2 && expNumber === void 0)
              invalidErr(representation, "missing exponent power");
            if (e2 && unsignedNumber === void 0)
              invalidErr(representation, "missing exponent base");
            if (e2 === void 0 && (expSign || expNumber)) {
              invalidErr(representation, "missing e before exponent");
            }
          }
          if (representation[index] === "+" || representation[index] === "-") {
            sawSign = true;
            isNegative = representation[index++] === "-";
          }
          if (!isDigit(representation[index]) && representation[index] !== ".") {
            if (representation[index] === "i" || representation[index] === "I") {
              return new _Decimal128(isNegative ? INF_NEGATIVE_BUFFER : INF_POSITIVE_BUFFER);
            } else if (representation[index] === "N") {
              return new _Decimal128(NAN_BUFFER);
            }
          }
          while (isDigit(representation[index]) || representation[index] === ".") {
            if (representation[index] === ".") {
              if (sawRadix)
                invalidErr(representation, "contains multiple periods");
              sawRadix = true;
              index = index + 1;
              continue;
            }
            if (nDigitsStored < MAX_DIGITS) {
              if (representation[index] !== "0" || foundNonZero) {
                if (!foundNonZero) {
                  firstNonZero = nDigitsRead;
                }
                foundNonZero = true;
                digits[digitsInsert++] = parseInt(representation[index], 10);
                nDigitsStored = nDigitsStored + 1;
              }
            }
            if (foundNonZero)
              nDigits = nDigits + 1;
            if (sawRadix)
              radixPosition = radixPosition + 1;
            nDigitsRead = nDigitsRead + 1;
            index = index + 1;
          }
          if (sawRadix && !nDigitsRead)
            throw new BSONError("" + representation + " not a valid Decimal128 string");
          if (representation[index] === "e" || representation[index] === "E") {
            const match = representation.substr(++index).match(EXPONENT_REGEX);
            if (!match || !match[2])
              return new _Decimal128(NAN_BUFFER);
            exponent = parseInt(match[0], 10);
            index = index + match[0].length;
          }
          if (representation[index])
            return new _Decimal128(NAN_BUFFER);
          if (!nDigitsStored) {
            digits[0] = 0;
            nDigits = 1;
            nDigitsStored = 1;
            significantDigits = 0;
          } else {
            lastDigit = nDigitsStored - 1;
            significantDigits = nDigits;
            if (significantDigits !== 1) {
              while (representation[firstNonZero + significantDigits - 1 + Number(sawSign) + Number(sawRadix)] === "0") {
                significantDigits = significantDigits - 1;
              }
            }
          }
          if (exponent <= radixPosition && radixPosition > exponent + (1 << 14)) {
            exponent = EXPONENT_MIN;
          } else {
            exponent = exponent - radixPosition;
          }
          while (exponent > EXPONENT_MAX) {
            lastDigit = lastDigit + 1;
            if (lastDigit >= MAX_DIGITS) {
              if (significantDigits === 0) {
                exponent = EXPONENT_MAX;
                break;
              }
              invalidErr(representation, "overflow");
            }
            exponent = exponent - 1;
          }
          if (options.allowRounding) {
            while (exponent < EXPONENT_MIN || nDigitsStored < nDigits) {
              if (lastDigit === 0 && significantDigits < nDigitsStored) {
                exponent = EXPONENT_MIN;
                significantDigits = 0;
                break;
              }
              if (nDigitsStored < nDigits) {
                nDigits = nDigits - 1;
              } else {
                lastDigit = lastDigit - 1;
              }
              if (exponent < EXPONENT_MAX) {
                exponent = exponent + 1;
              } else {
                const digitsString = digits.join("");
                if (digitsString.match(/^0+$/)) {
                  exponent = EXPONENT_MAX;
                  break;
                }
                invalidErr(representation, "overflow");
              }
            }
            if (lastDigit + 1 < significantDigits) {
              let endOfString = nDigitsRead;
              if (sawRadix) {
                firstNonZero = firstNonZero + 1;
                endOfString = endOfString + 1;
              }
              if (sawSign) {
                firstNonZero = firstNonZero + 1;
                endOfString = endOfString + 1;
              }
              const roundDigit = parseInt(representation[firstNonZero + lastDigit + 1], 10);
              let roundBit = 0;
              if (roundDigit >= 5) {
                roundBit = 1;
                if (roundDigit === 5) {
                  roundBit = digits[lastDigit] % 2 === 1 ? 1 : 0;
                  for (let i2 = firstNonZero + lastDigit + 2; i2 < endOfString; i2++) {
                    if (parseInt(representation[i2], 10)) {
                      roundBit = 1;
                      break;
                    }
                  }
                }
              }
              if (roundBit) {
                let dIdx = lastDigit;
                for (; dIdx >= 0; dIdx--) {
                  if (++digits[dIdx] > 9) {
                    digits[dIdx] = 0;
                    if (dIdx === 0) {
                      if (exponent < EXPONENT_MAX) {
                        exponent = exponent + 1;
                        digits[dIdx] = 1;
                      } else {
                        return new _Decimal128(isNegative ? INF_NEGATIVE_BUFFER : INF_POSITIVE_BUFFER);
                      }
                    }
                  } else {
                    break;
                  }
                }
              }
            }
          } else {
            while (exponent < EXPONENT_MIN || nDigitsStored < nDigits) {
              if (lastDigit === 0) {
                if (significantDigits === 0) {
                  exponent = EXPONENT_MIN;
                  break;
                }
                invalidErr(representation, "exponent underflow");
              }
              if (nDigitsStored < nDigits) {
                if (representation[nDigits - 1 + Number(sawSign) + Number(sawRadix)] !== "0" && significantDigits !== 0) {
                  invalidErr(representation, "inexact rounding");
                }
                nDigits = nDigits - 1;
              } else {
                if (digits[lastDigit] !== 0) {
                  invalidErr(representation, "inexact rounding");
                }
                lastDigit = lastDigit - 1;
              }
              if (exponent < EXPONENT_MAX) {
                exponent = exponent + 1;
              } else {
                invalidErr(representation, "overflow");
              }
            }
            if (lastDigit + 1 < significantDigits) {
              if (sawRadix) {
                firstNonZero = firstNonZero + 1;
              }
              if (sawSign) {
                firstNonZero = firstNonZero + 1;
              }
              const roundDigit = parseInt(representation[firstNonZero + lastDigit + 1], 10);
              if (roundDigit !== 0) {
                invalidErr(representation, "inexact rounding");
              }
            }
          }
          significandHigh = Long.fromNumber(0);
          significandLow = Long.fromNumber(0);
          if (significantDigits === 0) {
            significandHigh = Long.fromNumber(0);
            significandLow = Long.fromNumber(0);
          } else if (lastDigit < 17) {
            let dIdx = 0;
            significandLow = Long.fromNumber(digits[dIdx++]);
            significandHigh = new Long(0, 0);
            for (; dIdx <= lastDigit; dIdx++) {
              significandLow = significandLow.multiply(Long.fromNumber(10));
              significandLow = significandLow.add(Long.fromNumber(digits[dIdx]));
            }
          } else {
            let dIdx = 0;
            significandHigh = Long.fromNumber(digits[dIdx++]);
            for (; dIdx <= lastDigit - 17; dIdx++) {
              significandHigh = significandHigh.multiply(Long.fromNumber(10));
              significandHigh = significandHigh.add(Long.fromNumber(digits[dIdx]));
            }
            significandLow = Long.fromNumber(digits[dIdx++]);
            for (; dIdx <= lastDigit; dIdx++) {
              significandLow = significandLow.multiply(Long.fromNumber(10));
              significandLow = significandLow.add(Long.fromNumber(digits[dIdx]));
            }
          }
          const significand = multiply64x2(significandHigh, Long.fromString("100000000000000000"));
          significand.low = significand.low.add(significandLow);
          if (lessThan(significand.low, significandLow)) {
            significand.high = significand.high.add(Long.fromNumber(1));
          }
          biasedExponent = exponent + EXPONENT_BIAS;
          const dec = { low: Long.fromNumber(0), high: Long.fromNumber(0) };
          if (significand.high.shiftRightUnsigned(49).and(Long.fromNumber(1)).equals(Long.fromNumber(1))) {
            dec.high = dec.high.or(Long.fromNumber(3).shiftLeft(61));
            dec.high = dec.high.or(Long.fromNumber(biasedExponent).and(Long.fromNumber(16383).shiftLeft(47)));
            dec.high = dec.high.or(significand.high.and(Long.fromNumber(140737488355327)));
          } else {
            dec.high = dec.high.or(Long.fromNumber(biasedExponent & 16383).shiftLeft(49));
            dec.high = dec.high.or(significand.high.and(Long.fromNumber(562949953421311)));
          }
          dec.low = significand.low;
          if (isNegative) {
            dec.high = dec.high.or(Long.fromString("9223372036854775808"));
          }
          const buffer2 = ByteUtils.allocateUnsafe(16);
          index = 0;
          buffer2[index++] = dec.low.low & 255;
          buffer2[index++] = dec.low.low >> 8 & 255;
          buffer2[index++] = dec.low.low >> 16 & 255;
          buffer2[index++] = dec.low.low >> 24 & 255;
          buffer2[index++] = dec.low.high & 255;
          buffer2[index++] = dec.low.high >> 8 & 255;
          buffer2[index++] = dec.low.high >> 16 & 255;
          buffer2[index++] = dec.low.high >> 24 & 255;
          buffer2[index++] = dec.high.low & 255;
          buffer2[index++] = dec.high.low >> 8 & 255;
          buffer2[index++] = dec.high.low >> 16 & 255;
          buffer2[index++] = dec.high.low >> 24 & 255;
          buffer2[index++] = dec.high.high & 255;
          buffer2[index++] = dec.high.high >> 8 & 255;
          buffer2[index++] = dec.high.high >> 16 & 255;
          buffer2[index++] = dec.high.high >> 24 & 255;
          return new _Decimal128(buffer2);
        }
        toString() {
          let biased_exponent;
          let significand_digits = 0;
          const significand = new Array(36);
          for (let i2 = 0; i2 < significand.length; i2++)
            significand[i2] = 0;
          let index = 0;
          let is_zero = false;
          let significand_msb;
          let significand128 = { parts: [0, 0, 0, 0] };
          let j2, k4;
          const string = [];
          index = 0;
          const buffer2 = this.bytes;
          const low = buffer2[index++] | buffer2[index++] << 8 | buffer2[index++] << 16 | buffer2[index++] << 24;
          const midl = buffer2[index++] | buffer2[index++] << 8 | buffer2[index++] << 16 | buffer2[index++] << 24;
          const midh = buffer2[index++] | buffer2[index++] << 8 | buffer2[index++] << 16 | buffer2[index++] << 24;
          const high = buffer2[index++] | buffer2[index++] << 8 | buffer2[index++] << 16 | buffer2[index++] << 24;
          index = 0;
          const dec = {
            low: new Long(low, midl),
            high: new Long(midh, high)
          };
          if (dec.high.lessThan(Long.ZERO)) {
            string.push("-");
          }
          const combination = high >> 26 & COMBINATION_MASK;
          if (combination >> 3 === 3) {
            if (combination === COMBINATION_INFINITY) {
              return string.join("") + "Infinity";
            } else if (combination === COMBINATION_NAN) {
              return "NaN";
            } else {
              biased_exponent = high >> 15 & EXPONENT_MASK;
              significand_msb = 8 + (high >> 14 & 1);
            }
          } else {
            significand_msb = high >> 14 & 7;
            biased_exponent = high >> 17 & EXPONENT_MASK;
          }
          const exponent = biased_exponent - EXPONENT_BIAS;
          significand128.parts[0] = (high & 16383) + ((significand_msb & 15) << 14);
          significand128.parts[1] = midh;
          significand128.parts[2] = midl;
          significand128.parts[3] = low;
          if (significand128.parts[0] === 0 && significand128.parts[1] === 0 && significand128.parts[2] === 0 && significand128.parts[3] === 0) {
            is_zero = true;
          } else {
            for (k4 = 3; k4 >= 0; k4--) {
              let least_digits = 0;
              const result = divideu128(significand128);
              significand128 = result.quotient;
              least_digits = result.rem.low;
              if (!least_digits)
                continue;
              for (j2 = 8; j2 >= 0; j2--) {
                significand[k4 * 9 + j2] = least_digits % 10;
                least_digits = Math.floor(least_digits / 10);
              }
            }
          }
          if (is_zero) {
            significand_digits = 1;
            significand[index] = 0;
          } else {
            significand_digits = 36;
            while (!significand[index]) {
              significand_digits = significand_digits - 1;
              index = index + 1;
            }
          }
          const scientific_exponent = significand_digits - 1 + exponent;
          if (scientific_exponent >= 34 || scientific_exponent <= -7 || exponent > 0) {
            if (significand_digits > 34) {
              string.push(`${0}`);
              if (exponent > 0)
                string.push(`E+${exponent}`);
              else if (exponent < 0)
                string.push(`E${exponent}`);
              return string.join("");
            }
            string.push(`${significand[index++]}`);
            significand_digits = significand_digits - 1;
            if (significand_digits) {
              string.push(".");
            }
            for (let i2 = 0; i2 < significand_digits; i2++) {
              string.push(`${significand[index++]}`);
            }
            string.push("E");
            if (scientific_exponent > 0) {
              string.push(`+${scientific_exponent}`);
            } else {
              string.push(`${scientific_exponent}`);
            }
          } else {
            if (exponent >= 0) {
              for (let i2 = 0; i2 < significand_digits; i2++) {
                string.push(`${significand[index++]}`);
              }
            } else {
              let radix_position = significand_digits + exponent;
              if (radix_position > 0) {
                for (let i2 = 0; i2 < radix_position; i2++) {
                  string.push(`${significand[index++]}`);
                }
              } else {
                string.push("0");
              }
              string.push(".");
              while (radix_position++ < 0) {
                string.push("0");
              }
              for (let i2 = 0; i2 < significand_digits - Math.max(radix_position - 1, 0); i2++) {
                string.push(`${significand[index++]}`);
              }
            }
          }
          return string.join("");
        }
        toJSON() {
          return { $numberDecimal: this.toString() };
        }
        toExtendedJSON() {
          return { $numberDecimal: this.toString() };
        }
        static fromExtendedJSON(doc) {
          return _Decimal128.fromString(doc.$numberDecimal);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          const d128string = inspect(this.toString(), options);
          return `new Decimal128(${d128string})`;
        }
      };
      Double = class _Double extends BSONValue {
        get _bsontype() {
          return "Double";
        }
        value;
        constructor(value) {
          super();
          if (value instanceof Number) {
            value = value.valueOf();
          }
          this.value = +value;
        }
        static fromString(value) {
          const coercedValue = Number(value);
          if (value === "NaN")
            return new _Double(NaN);
          if (value === "Infinity")
            return new _Double(Infinity);
          if (value === "-Infinity")
            return new _Double(-Infinity);
          if (!Number.isFinite(coercedValue)) {
            throw new BSONError(`Input: ${value} is not representable as a Double`);
          }
          if (value.trim() !== value) {
            throw new BSONError(`Input: '${value}' contains whitespace`);
          }
          if (value === "") {
            throw new BSONError(`Input is an empty string`);
          }
          if (/[^-0-9.+eE]/.test(value)) {
            throw new BSONError(`Input: '${value}' is not in decimal or exponential notation`);
          }
          return new _Double(coercedValue);
        }
        valueOf() {
          return this.value;
        }
        toJSON() {
          return this.value;
        }
        toString(radix) {
          return this.value.toString(radix);
        }
        toExtendedJSON(options) {
          if (options && (options.legacy || options.relaxed && isFinite(this.value))) {
            return this.value;
          }
          if (Object.is(Math.sign(this.value), -0)) {
            return { $numberDouble: "-0.0" };
          }
          return {
            $numberDouble: Number.isInteger(this.value) ? this.value.toFixed(1) : this.value.toString()
          };
        }
        static fromExtendedJSON(doc, options) {
          const doubleValue = parseFloat(doc.$numberDouble);
          return options && options.relaxed ? doubleValue : new _Double(doubleValue);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          return `new Double(${inspect(this.value, options)})`;
        }
      };
      Int32 = class _Int32 extends BSONValue {
        get _bsontype() {
          return "Int32";
        }
        value;
        constructor(value) {
          super();
          if (value instanceof Number) {
            value = value.valueOf();
          }
          this.value = +value | 0;
        }
        static fromString(value) {
          const cleanedValue = removeLeadingZerosAndExplicitPlus(value);
          const coercedValue = Number(value);
          if (BSON_INT32_MAX < coercedValue) {
            throw new BSONError(`Input: '${value}' is larger than the maximum value for Int32`);
          } else if (BSON_INT32_MIN > coercedValue) {
            throw new BSONError(`Input: '${value}' is smaller than the minimum value for Int32`);
          } else if (!Number.isSafeInteger(coercedValue)) {
            throw new BSONError(`Input: '${value}' is not a safe integer`);
          } else if (coercedValue.toString() !== cleanedValue) {
            throw new BSONError(`Input: '${value}' is not a valid Int32 string`);
          }
          return new _Int32(coercedValue);
        }
        valueOf() {
          return this.value;
        }
        toString(radix) {
          return this.value.toString(radix);
        }
        toJSON() {
          return this.value;
        }
        toExtendedJSON(options) {
          if (options && (options.relaxed || options.legacy))
            return this.value;
          return { $numberInt: this.value.toString() };
        }
        static fromExtendedJSON(doc, options) {
          return options && options.relaxed ? parseInt(doc.$numberInt, 10) : new _Int32(doc.$numberInt);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          return `new Int32(${inspect(this.value, options)})`;
        }
      };
      MaxKey = class _MaxKey extends BSONValue {
        get _bsontype() {
          return "MaxKey";
        }
        toExtendedJSON() {
          return { $maxKey: 1 };
        }
        static fromExtendedJSON() {
          return new _MaxKey();
        }
        inspect() {
          return "new MaxKey()";
        }
      };
      MinKey = class _MinKey extends BSONValue {
        get _bsontype() {
          return "MinKey";
        }
        toExtendedJSON() {
          return { $minKey: 1 };
        }
        static fromExtendedJSON() {
          return new _MinKey();
        }
        inspect() {
          return "new MinKey()";
        }
      };
      PROCESS_UNIQUE = null;
      __idCache = /* @__PURE__ */ new WeakMap();
      ObjectId = class _ObjectId extends BSONValue {
        get _bsontype() {
          return "ObjectId";
        }
        static index = Math.floor(Math.random() * 16777215);
        static cacheHexString;
        buffer;
        constructor(inputId) {
          super();
          let workingId;
          if (typeof inputId === "object" && inputId && "id" in inputId) {
            if (typeof inputId.id !== "string" && !ArrayBuffer.isView(inputId.id)) {
              throw new BSONError("Argument passed in must have an id that is of type string or Buffer");
            }
            if ("toHexString" in inputId && typeof inputId.toHexString === "function") {
              workingId = ByteUtils.fromHex(inputId.toHexString());
            } else {
              workingId = inputId.id;
            }
          } else {
            workingId = inputId;
          }
          if (workingId == null) {
            this.buffer = _ObjectId.generate();
          } else if (ArrayBuffer.isView(workingId) && workingId.byteLength === 12) {
            this.buffer = ByteUtils.toLocalBufferType(workingId);
          } else if (typeof workingId === "string") {
            if (_ObjectId.validateHexString(workingId)) {
              this.buffer = ByteUtils.fromHex(workingId);
              if (_ObjectId.cacheHexString) {
                __idCache.set(this, workingId);
              }
            } else {
              throw new BSONError("input must be a 24 character hex string, 12 byte Uint8Array, or an integer");
            }
          } else {
            throw new BSONError("Argument passed in does not match the accepted types");
          }
        }
        get id() {
          return this.buffer;
        }
        set id(value) {
          this.buffer = value;
          if (_ObjectId.cacheHexString) {
            __idCache.set(this, ByteUtils.toHex(value));
          }
        }
        static validateHexString(string) {
          if (string?.length !== 24)
            return false;
          for (let i2 = 0; i2 < 24; i2++) {
            const char = string.charCodeAt(i2);
            if (char >= 48 && char <= 57 || char >= 97 && char <= 102 || char >= 65 && char <= 70) {
              continue;
            }
            return false;
          }
          return true;
        }
        toHexString() {
          if (_ObjectId.cacheHexString) {
            const __id = __idCache.get(this);
            if (__id)
              return __id;
          }
          const hexString = ByteUtils.toHex(this.id);
          if (_ObjectId.cacheHexString) {
            __idCache.set(this, hexString);
          }
          return hexString;
        }
        static getInc() {
          return _ObjectId.index = (_ObjectId.index + 1) % 16777215;
        }
        static generate(time) {
          if ("number" !== typeof time) {
            time = Math.floor(Date.now() / 1e3);
          }
          const inc = _ObjectId.getInc();
          const buffer2 = ByteUtils.allocateUnsafe(12);
          NumberUtils.setInt32BE(buffer2, 0, time);
          if (PROCESS_UNIQUE === null) {
            PROCESS_UNIQUE = ByteUtils.randomBytes(5);
          }
          buffer2[4] = PROCESS_UNIQUE[0];
          buffer2[5] = PROCESS_UNIQUE[1];
          buffer2[6] = PROCESS_UNIQUE[2];
          buffer2[7] = PROCESS_UNIQUE[3];
          buffer2[8] = PROCESS_UNIQUE[4];
          buffer2[11] = inc & 255;
          buffer2[10] = inc >> 8 & 255;
          buffer2[9] = inc >> 16 & 255;
          return buffer2;
        }
        toString(encoding) {
          if (encoding === "base64")
            return ByteUtils.toBase64(this.id);
          if (encoding === "hex")
            return this.toHexString();
          return this.toHexString();
        }
        toJSON() {
          return this.toHexString();
        }
        static is(variable) {
          return variable != null && typeof variable === "object" && "_bsontype" in variable && variable._bsontype === "ObjectId";
        }
        equals(otherId) {
          if (otherId === void 0 || otherId === null) {
            return false;
          }
          if (_ObjectId.is(otherId)) {
            return this.buffer[11] === otherId.buffer[11] && ByteUtils.equals(this.buffer, otherId.buffer);
          }
          if (typeof otherId === "string") {
            return otherId.toLowerCase() === this.toHexString();
          }
          if (typeof otherId === "object" && typeof otherId.toHexString === "function") {
            const otherIdString = otherId.toHexString();
            const thisIdString = this.toHexString();
            return typeof otherIdString === "string" && otherIdString.toLowerCase() === thisIdString;
          }
          return false;
        }
        getTimestamp() {
          const timestamp = /* @__PURE__ */ new Date();
          const time = NumberUtils.getUint32BE(this.buffer, 0);
          timestamp.setTime(Math.floor(time) * 1e3);
          return timestamp;
        }
        static createPk() {
          return new _ObjectId();
        }
        serializeInto(uint8array, index) {
          uint8array[index] = this.buffer[0];
          uint8array[index + 1] = this.buffer[1];
          uint8array[index + 2] = this.buffer[2];
          uint8array[index + 3] = this.buffer[3];
          uint8array[index + 4] = this.buffer[4];
          uint8array[index + 5] = this.buffer[5];
          uint8array[index + 6] = this.buffer[6];
          uint8array[index + 7] = this.buffer[7];
          uint8array[index + 8] = this.buffer[8];
          uint8array[index + 9] = this.buffer[9];
          uint8array[index + 10] = this.buffer[10];
          uint8array[index + 11] = this.buffer[11];
          return 12;
        }
        static createFromTime(time) {
          const buffer2 = ByteUtils.allocate(12);
          for (let i2 = 11; i2 >= 4; i2--)
            buffer2[i2] = 0;
          NumberUtils.setInt32BE(buffer2, 0, time);
          return new _ObjectId(buffer2);
        }
        static createFromHexString(hexString) {
          if (hexString?.length !== 24) {
            throw new BSONError("hex string must be 24 characters");
          }
          return new _ObjectId(ByteUtils.fromHex(hexString));
        }
        static createFromBase64(base64) {
          if (base64?.length !== 16) {
            throw new BSONError("base64 string must be 16 characters");
          }
          return new _ObjectId(ByteUtils.fromBase64(base64));
        }
        static isValid(id) {
          if (id == null)
            return false;
          if (typeof id === "string")
            return _ObjectId.validateHexString(id);
          try {
            new _ObjectId(id);
            return true;
          } catch {
            return false;
          }
        }
        toExtendedJSON() {
          if (this.toHexString)
            return { $oid: this.toHexString() };
          return { $oid: this.toString("hex") };
        }
        static fromExtendedJSON(doc) {
          return new _ObjectId(doc.$oid);
        }
        isCached() {
          return _ObjectId.cacheHexString && __idCache.has(this);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          return `new ObjectId(${inspect(this.toHexString(), options)})`;
        }
      };
      BSONRegExp = class _BSONRegExp extends BSONValue {
        get _bsontype() {
          return "BSONRegExp";
        }
        pattern;
        options;
        constructor(pattern, options) {
          super();
          this.pattern = pattern;
          this.options = alphabetize(options ?? "");
          if (this.pattern.indexOf("\0") !== -1) {
            throw new BSONError(`BSON Regex patterns cannot contain null bytes, found: ${JSON.stringify(this.pattern)}`);
          }
          if (this.options.indexOf("\0") !== -1) {
            throw new BSONError(`BSON Regex options cannot contain null bytes, found: ${JSON.stringify(this.options)}`);
          }
          for (let i2 = 0; i2 < this.options.length; i2++) {
            if (!(this.options[i2] === "i" || this.options[i2] === "m" || this.options[i2] === "x" || this.options[i2] === "l" || this.options[i2] === "s" || this.options[i2] === "u")) {
              throw new BSONError(`The regular expression option [${this.options[i2]}] is not supported`);
            }
          }
        }
        static parseOptions(options) {
          return options ? options.split("").sort().join("") : "";
        }
        toExtendedJSON(options) {
          options = options || {};
          if (options.legacy) {
            return { $regex: this.pattern, $options: this.options };
          }
          return { $regularExpression: { pattern: this.pattern, options: this.options } };
        }
        static fromExtendedJSON(doc) {
          if ("$regex" in doc) {
            if (typeof doc.$regex !== "string") {
              if (doc.$regex._bsontype === "BSONRegExp") {
                return doc;
              }
            } else {
              return new _BSONRegExp(doc.$regex, _BSONRegExp.parseOptions(doc.$options));
            }
          }
          if ("$regularExpression" in doc) {
            return new _BSONRegExp(doc.$regularExpression.pattern, _BSONRegExp.parseOptions(doc.$regularExpression.options));
          }
          throw new BSONError(`Unexpected BSONRegExp EJSON object form: ${JSON.stringify(doc)}`);
        }
        inspect(depth, options, inspect) {
          const stylize = getStylizeFunction(options) ?? ((v3) => v3);
          inspect ??= defaultInspect;
          const pattern = stylize(inspect(this.pattern), "regexp");
          const flags = stylize(inspect(this.options), "regexp");
          return `new BSONRegExp(${pattern}, ${flags})`;
        }
      };
      BSONSymbol = class _BSONSymbol extends BSONValue {
        get _bsontype() {
          return "BSONSymbol";
        }
        value;
        constructor(value) {
          super();
          this.value = value;
        }
        valueOf() {
          return this.value;
        }
        toString() {
          return this.value;
        }
        toJSON() {
          return this.value;
        }
        toExtendedJSON() {
          return { $symbol: this.value };
        }
        static fromExtendedJSON(doc) {
          return new _BSONSymbol(doc.$symbol);
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          return `new BSONSymbol(${inspect(this.value, options)})`;
        }
      };
      LongWithoutOverridesClass = Long;
      Timestamp = class _Timestamp extends LongWithoutOverridesClass {
        get _bsontype() {
          return "Timestamp";
        }
        get [bsonType]() {
          return "Timestamp";
        }
        static MAX_VALUE = Long.MAX_UNSIGNED_VALUE;
        get i() {
          return this.low >>> 0;
        }
        get t() {
          return this.high >>> 0;
        }
        constructor(low) {
          if (low == null) {
            super(0, 0, true);
          } else if (typeof low === "bigint") {
            super(low, true);
          } else if (Long.isLong(low)) {
            super(low.low, low.high, true);
          } else if (typeof low === "object" && "t" in low && "i" in low) {
            if (typeof low.t !== "number" && (typeof low.t !== "object" || low.t._bsontype !== "Int32")) {
              throw new BSONError("Timestamp constructed from { t, i } must provide t as a number");
            }
            if (typeof low.i !== "number" && (typeof low.i !== "object" || low.i._bsontype !== "Int32")) {
              throw new BSONError("Timestamp constructed from { t, i } must provide i as a number");
            }
            const t4 = Number(low.t);
            const i2 = Number(low.i);
            if (t4 < 0 || Number.isNaN(t4)) {
              throw new BSONError("Timestamp constructed from { t, i } must provide a positive t");
            }
            if (i2 < 0 || Number.isNaN(i2)) {
              throw new BSONError("Timestamp constructed from { t, i } must provide a positive i");
            }
            if (t4 > 4294967295) {
              throw new BSONError("Timestamp constructed from { t, i } must provide t equal or less than uint32 max");
            }
            if (i2 > 4294967295) {
              throw new BSONError("Timestamp constructed from { t, i } must provide i equal or less than uint32 max");
            }
            super(i2, t4, true);
          } else {
            throw new BSONError("A Timestamp can only be constructed with: bigint, Long, or { t: number; i: number }");
          }
        }
        toJSON() {
          return {
            $timestamp: this.toString()
          };
        }
        static fromInt(value) {
          return new _Timestamp(Long.fromInt(value, true));
        }
        static fromNumber(value) {
          return new _Timestamp(Long.fromNumber(value, true));
        }
        static fromBits(lowBits, highBits) {
          return new _Timestamp({ i: lowBits, t: highBits });
        }
        static fromString(str, optRadix) {
          return new _Timestamp(Long.fromString(str, true, optRadix));
        }
        toExtendedJSON() {
          return { $timestamp: { t: this.t, i: this.i } };
        }
        static fromExtendedJSON(doc) {
          const i2 = Long.isLong(doc.$timestamp.i) ? doc.$timestamp.i.getLowBitsUnsigned() : doc.$timestamp.i;
          const t4 = Long.isLong(doc.$timestamp.t) ? doc.$timestamp.t.getLowBitsUnsigned() : doc.$timestamp.t;
          return new _Timestamp({ t: t4, i: i2 });
        }
        inspect(depth, options, inspect) {
          inspect ??= defaultInspect;
          const t4 = inspect(this.t, options);
          const i2 = inspect(this.i, options);
          return `new Timestamp({ t: ${t4}, i: ${i2} })`;
        }
      };
      JS_INT_MAX_LONG = Long.fromNumber(JS_INT_MAX);
      JS_INT_MIN_LONG = Long.fromNumber(JS_INT_MIN);
      allowedDBRefKeys = /^\$ref$|^\$id$|^\$db$/;
      keysToCodecs = {
        $oid: ObjectId,
        $binary: Binary,
        $uuid: Binary,
        $symbol: BSONSymbol,
        $numberInt: Int32,
        $numberDecimal: Decimal128,
        $numberDouble: Double,
        $numberLong: Long,
        $minKey: MinKey,
        $maxKey: MaxKey,
        $regex: BSONRegExp,
        $regularExpression: BSONRegExp,
        $timestamp: Timestamp
      };
      BSON_TYPE_MAPPINGS = {
        Binary: (o5) => new Binary(o5.value(), o5.sub_type),
        Code: (o5) => new Code(o5.code, o5.scope),
        DBRef: (o5) => new DBRef(o5.collection || o5.namespace, o5.oid, o5.db, o5.fields),
        Decimal128: (o5) => new Decimal128(o5.bytes),
        Double: (o5) => new Double(o5.value),
        Int32: (o5) => new Int32(o5.value),
        Long: (o5) => Long.fromBits(o5.low != null ? o5.low : o5.low_, o5.low != null ? o5.high : o5.high_, o5.low != null ? o5.unsigned : o5.unsigned_),
        MaxKey: () => new MaxKey(),
        MinKey: () => new MinKey(),
        ObjectId: (o5) => new ObjectId(o5),
        BSONRegExp: (o5) => new BSONRegExp(o5.pattern, o5.options),
        BSONSymbol: (o5) => new BSONSymbol(o5.value),
        Timestamp: (o5) => Timestamp.fromBits(o5.low, o5.high)
      };
      EJSON = /* @__PURE__ */ Object.create(null);
      EJSON.parse = parse;
      EJSON.stringify = stringify;
      EJSON.serialize = EJSONserialize;
      EJSON.deserialize = EJSONdeserialize;
      Object.freeze(EJSON);
      BSONElementType = {
        double: 1,
        string: 2,
        object: 3,
        array: 4,
        binData: 5,
        undefined: 6,
        objectId: 7,
        bool: 8,
        date: 9,
        null: 10,
        regex: 11,
        dbPointer: 12,
        javascript: 13,
        symbol: 14,
        javascriptWithScope: 15,
        int: 16,
        timestamp: 17,
        long: 18,
        decimal: 19,
        minKey: 255,
        maxKey: 127
      };
      onDemand = /* @__PURE__ */ Object.create(null);
      onDemand.parseToElements = parseToElements;
      onDemand.ByteUtils = ByteUtils;
      onDemand.NumberUtils = NumberUtils;
      Object.freeze(onDemand);
      MAXSIZE = 1024 * 1024 * 17;
      buffer = ByteUtils.allocate(MAXSIZE);
    }
  });

  // node_modules/cbor2/lib/constants.js
  var f, T, I, o, A, S;
  var init_constants = __esm({
    "node_modules/cbor2/lib/constants.js"() {
      f = { POS_INT: 0, NEG_INT: 1, BYTE_STRING: 2, UTF8_STRING: 3, ARRAY: 4, MAP: 5, TAG: 6, SIMPLE_FLOAT: 7 };
      T = { DATE_STRING: 0, DATE_EPOCH: 1, POS_BIGINT: 2, NEG_BIGINT: 3, DECIMAL_FRAC: 4, BIGFLOAT: 5, BASE64URL_EXPECTED: 21, BASE64_EXPECTED: 22, BASE16_EXPECTED: 23, CBOR: 24, URI: 32, BASE64URL: 33, BASE64: 34, MIME: 36, DATE_EPOCH_DAYS: 100, SET: 258, JSON: 262, WTF8: 273, SYMBOL: 280, DATE_FULL: 1004, REGEXP: 21066, SELF_DESCRIBED: 55799, INVALID_16: 65535, INVALID_32: 4294967295, INVALID_64: 0xffffffffffffffffn };
      I = { ZERO: 0, ONE: 24, TWO: 25, FOUR: 26, EIGHT: 27, INDEFINITE: 31 };
      o = { FALSE: 20, TRUE: 21, NULL: 22, UNDEFINED: 23 };
      A = class {
        static BREAK = Symbol.for("github.com/hildjj/cbor2/break");
        static ENCODED = Symbol.for("github.com/hildjj/cbor2/cbor-encoded");
        static LENGTH = Symbol.for("github.com/hildjj/cbor2/length");
      };
      S = { MIN: -(2n ** 63n), MAX: 2n ** 64n - 1n };
    }
  });

  // node_modules/cbor2/lib/tag.js
  var o2;
  var init_tag = __esm({
    "node_modules/cbor2/lib/tag.js"() {
      o2 = class _o {
        static #e = /* @__PURE__ */ new Map();
        tag;
        contents;
        constructor(e2, t4 = void 0) {
          this.tag = e2, this.contents = t4;
        }
        get noChildren() {
          return !!_o.#e.get(this.tag)?.noChildren;
        }
        static registerDecoder(e2, t4, n2) {
          const i2 = this.#e.get(e2);
          return this.#e.set(e2, t4), i2 && ("comment" in t4 || (t4.comment = i2.comment), "noChildren" in t4 || (t4.noChildren = i2.noChildren)), n2 && !t4.comment && (t4.comment = () => `(${n2})`), i2;
        }
        static clearDecoder(e2) {
          const t4 = this.#e.get(e2);
          return this.#e.delete(e2), t4;
        }
        static getDecoder(e2) {
          return this.#e.get(e2);
        }
        static getAllDecoders() {
          return new Map(this.#e);
        }
        *[Symbol.iterator]() {
          yield this.contents;
        }
        push(e2) {
          return this.contents = e2, 1;
        }
        decode(e2) {
          const t4 = e2?.tags?.get(this.tag) ?? (e2?.ignoreGlobalTags ? void 0 : _o.#e.get(this.tag));
          return t4 ? t4(this, e2) : this;
        }
        comment(e2, t4) {
          const n2 = e2?.tags?.get(this.tag) ?? (e2?.ignoreGlobalTags ? void 0 : _o.#e.get(this.tag));
          if (n2?.comment) return n2.comment(this, e2, t4);
        }
        toCBOR() {
          return [this.tag, this.contents];
        }
        [Symbol.for("nodejs.util.inspect.custom")](e2, t4, n2) {
          return `${this.tag}(${n2(this.contents, t4)})`;
        }
      };
    }
  });

  // node_modules/cbor2/lib/box.js
  function f2(n2) {
    if (n2 != null && typeof n2 == "object") return n2[A.ENCODED];
  }
  function s(n2) {
    if (n2 != null && typeof n2 == "object") return n2[A.LENGTH];
  }
  function u(n2, e2) {
    Object.defineProperty(n2, A.ENCODED, { configurable: true, enumerable: false, value: e2 });
  }
  function d(n2, e2) {
    const r3 = Object(n2);
    return u(r3, e2), r3;
  }
  var init_box = __esm({
    "node_modules/cbor2/lib/box.js"() {
      init_constants();
      init_tag();
    }
  });

  // node_modules/cbor2/lib/utils.js
  function c(r3, n2) {
    Object.defineProperty(r3, g, { configurable: false, enumerable: false, writable: false, value: n2 });
  }
  function f3(r3) {
    return r3[g];
  }
  function l(r3) {
    return f3(r3) !== void 0;
  }
  function R(r3, n2 = 0, t4 = r3.length - 1) {
    const o5 = r3.subarray(n2, t4), a4 = f3(r3);
    if (a4) {
      const s3 = [];
      for (const e2 of a4) if (e2[0] >= n2 && e2[0] + e2[1] <= t4) {
        const i2 = [...e2];
        i2[0] -= n2, s3.push(i2);
      }
      s3.length && c(o5, s3);
    }
    return o5;
  }
  function b(r3) {
    let n2 = Math.ceil(r3.length / 2);
    const t4 = new Uint8Array(n2);
    n2--;
    for (let o5 = r3.length, a4 = o5 - 2; o5 >= 0; o5 = a4, a4 -= 2, n2--) t4[n2] = parseInt(r3.substring(a4, o5), 16);
    return t4;
  }
  function A2(r3) {
    return r3.reduce((n2, t4) => n2 + t4.toString(16).padStart(2, "0"), "");
  }
  function d2(r3) {
    const n2 = r3.reduce((e2, i2) => e2 + i2.length, 0), t4 = r3.some((e2) => l(e2)), o5 = [], a4 = new Uint8Array(n2);
    let s3 = 0;
    for (const e2 of r3) {
      if (!(e2 instanceof Uint8Array)) throw new TypeError(`Invalid array: ${e2}`);
      if (a4.set(e2, s3), t4) {
        const i2 = e2[g] ?? [[0, e2.length]];
        for (const u3 of i2) u3[0] += s3;
        o5.push(...i2);
      }
      s3 += e2.length;
    }
    return t4 && c(a4, o5), a4;
  }
  function y(r3) {
    const n2 = atob(r3);
    return Uint8Array.from(n2, (t4) => t4.codePointAt(0));
  }
  function x(r3) {
    const n2 = r3.replace(/[_-]/g, (t4) => p[t4]);
    return y(n2.padEnd(Math.ceil(n2.length / 4) * 4, "="));
  }
  function h() {
    const r3 = new Uint8Array(4), n2 = new Uint32Array(r3.buffer);
    return !((n2[0] = 1) & r3[0]);
  }
  function U(r3) {
    let n2 = "";
    for (const t4 of r3) {
      const o5 = t4.codePointAt(0)?.toString(16).padStart(4, "0");
      n2 && (n2 += ", "), n2 += `U+${o5}`;
    }
    return n2;
  }
  var g, p;
  var init_utils = __esm({
    "node_modules/cbor2/lib/utils.js"() {
      g = Symbol("CBOR_RANGES");
      p = { "-": "+", _: "/" };
    }
  });

  // node_modules/cbor2/lib/typeEncoderMap.js
  var s2;
  var init_typeEncoderMap = __esm({
    "node_modules/cbor2/lib/typeEncoderMap.js"() {
      s2 = class {
        #e = /* @__PURE__ */ new Map();
        registerEncoder(e2, t4) {
          const n2 = this.#e.get(e2);
          return this.#e.set(e2, t4), n2;
        }
        get(e2) {
          return this.#e.get(e2);
        }
        delete(e2) {
          return this.#e.delete(e2);
        }
        clear() {
          this.#e.clear();
        }
      };
    }
  });

  // node_modules/cbor2/lib/sorts.js
  function f4(c5, d6) {
    const [u3, a4, n2] = c5, [l6, s3, t4] = d6, r3 = Math.min(n2.length, t4.length);
    for (let o5 = 0; o5 < r3; o5++) {
      const e2 = n2[o5] - t4[o5];
      if (e2 !== 0) return e2;
    }
    return 0;
  }
  var init_sorts = __esm({
    "node_modules/cbor2/lib/sorts.js"() {
    }
  });

  // node_modules/cbor2/lib/writer.js
  var e;
  var init_writer = __esm({
    "node_modules/cbor2/lib/writer.js"() {
      e = class _e {
        static defaultOptions = { chunkSize: 4096 };
        #r;
        #i = [];
        #s = null;
        #t = 0;
        #a = 0;
        constructor(t4 = {}) {
          if (this.#r = { ..._e.defaultOptions, ...t4 }, this.#r.chunkSize < 8) throw new RangeError(`Expected size >= 8, got ${this.#r.chunkSize}`);
          this.#n();
        }
        get length() {
          return this.#a;
        }
        read() {
          this.#o();
          const t4 = new Uint8Array(this.#a);
          let i2 = 0;
          for (const s3 of this.#i) t4.set(s3, i2), i2 += s3.length;
          return this.#n(), t4;
        }
        write(t4) {
          const i2 = t4.length;
          i2 > this.#l() ? (this.#o(), i2 > this.#r.chunkSize ? (this.#i.push(t4), this.#n()) : (this.#n(), this.#i[this.#i.length - 1].set(t4), this.#t = i2)) : (this.#i[this.#i.length - 1].set(t4, this.#t), this.#t += i2), this.#a += i2;
        }
        writeUint8(t4) {
          this.#e(1), this.#s.setUint8(this.#t, t4), this.#h(1);
        }
        writeUint16(t4, i2 = false) {
          this.#e(2), this.#s.setUint16(this.#t, t4, i2), this.#h(2);
        }
        writeUint32(t4, i2 = false) {
          this.#e(4), this.#s.setUint32(this.#t, t4, i2), this.#h(4);
        }
        writeBigUint64(t4, i2 = false) {
          this.#e(8), this.#s.setBigUint64(this.#t, t4, i2), this.#h(8);
        }
        writeInt16(t4, i2 = false) {
          this.#e(2), this.#s.setInt16(this.#t, t4, i2), this.#h(2);
        }
        writeInt32(t4, i2 = false) {
          this.#e(4), this.#s.setInt32(this.#t, t4, i2), this.#h(4);
        }
        writeBigInt64(t4, i2 = false) {
          this.#e(8), this.#s.setBigInt64(this.#t, t4, i2), this.#h(8);
        }
        writeFloat32(t4, i2 = false) {
          this.#e(4), this.#s.setFloat32(this.#t, t4, i2), this.#h(4);
        }
        writeFloat64(t4, i2 = false) {
          this.#e(8), this.#s.setFloat64(this.#t, t4, i2), this.#h(8);
        }
        clear() {
          this.#a = 0, this.#i = [], this.#n();
        }
        #n() {
          const t4 = new Uint8Array(this.#r.chunkSize);
          this.#i.push(t4), this.#t = 0, this.#s = new DataView(t4.buffer, t4.byteOffset, t4.byteLength);
        }
        #o() {
          if (this.#t === 0) {
            this.#i.pop();
            return;
          }
          const t4 = this.#i.length - 1;
          this.#i[t4] = this.#i[t4].subarray(0, this.#t), this.#t = 0, this.#s = null;
        }
        #l() {
          const t4 = this.#i.length - 1;
          return this.#i[t4].length - this.#t;
        }
        #e(t4) {
          this.#l() < t4 && (this.#o(), this.#n());
        }
        #h(t4) {
          this.#t += t4, this.#a += t4;
        }
      };
    }
  });

  // node_modules/cbor2/lib/float.js
  function U2(i2, n2, e2, r3) {
    let o5 = "nan'";
    return i2.quiet || (o5 += "!"), i2.sign === -1 && (o5 += "-"), o5 += r3(Math.abs(i2.payload), e2), o5 += "'", o5 += i2.encodingIndicator, o5;
  }
  function v(i2, n2 = 0, e2 = false) {
    const r3 = i2[n2] & 128 ? -1 : 1, o5 = (i2[n2] & 124) >> 2, t4 = (i2[n2] & 3) << 8 | i2[n2 + 1];
    if (o5 === 0) {
      if (e2 && t4 !== 0) throw new Error(`Unwanted subnormal: ${r3 * 5960464477539063e-23 * t4}`);
      return r3 * 5960464477539063e-23 * t4;
    } else if (o5 === 31) return t4 ? NaN : r3 * (1 / 0);
    return r3 * 2 ** (o5 - 25) * (1024 + t4);
  }
  function Z(i2) {
    const n2 = new DataView(new ArrayBuffer(4));
    n2.setFloat32(0, i2, false);
    const e2 = n2.getUint32(0, false);
    if ((e2 & 8191) !== 0) return null;
    let r3 = e2 >> 16 & 32768;
    const o5 = e2 >> 23 & 255, t4 = e2 & 8388607;
    if (!(o5 === 0 && t4 === 0)) if (o5 >= 113 && o5 <= 142) r3 += (o5 - 112 << 10) + (t4 >> 13);
    else if (o5 >= 103 && o5 < 113) {
      if (t4 & (1 << 126 - o5) - 1) return null;
      r3 += t4 + 8388608 >> 126 - o5;
    } else if (o5 === 255) r3 |= 31744, r3 |= t4 >> 13;
    else return null;
    return r3;
  }
  function k(i2) {
    if (i2 !== 0) {
      const n2 = new ArrayBuffer(8), e2 = new DataView(n2);
      e2.setFloat64(0, i2, false);
      const r3 = e2.getBigUint64(0, false);
      if ((r3 & 0x7ff0000000000000n) === 0n) return r3 & 0x8000000000000000n ? -0 : 0;
    }
    return i2;
  }
  function B(i2) {
    switch (i2.length) {
      case 2:
        v(i2, 0, true);
        break;
      case 4: {
        const n2 = new DataView(i2.buffer, i2.byteOffset, i2.byteLength), e2 = n2.getUint32(0, false);
        if ((e2 & 2139095040) === 0 && e2 & 8388607) throw new Error(`Unwanted subnormal: ${n2.getFloat32(0, false)}`);
        break;
      }
      case 8: {
        const n2 = new DataView(i2.buffer, i2.byteOffset, i2.byteLength), e2 = n2.getBigUint64(0, false);
        if ((e2 & 0x7ff0000000000000n) === 0n && e2 & 0x000fffffffffffn) throw new Error(`Unwanted subnormal: ${n2.getFloat64(0, false)}`);
        break;
      }
      default:
        throw new TypeError(`Bad input to isSubnormal: ${i2}`);
    }
  }
  var w, f5, b2, d3, I2, g2, l2, A3, E, p2, N, a, c2, u2, F, _, S2, m, x2, O;
  var init_float = __esm({
    "node_modules/cbor2/lib/float.js"() {
      w = 1n << 15n;
      f5 = 0b11111n << 10n;
      b2 = 1n << 9n;
      d3 = b2 - 1n;
      I2 = b2 | d3;
      g2 = 1n << 31n;
      l2 = 0b11111111n << 23n;
      A3 = 1n << 22n;
      E = A3 - 1n;
      p2 = A3 | E;
      N = 1n << 63n;
      a = 0b11111111111n << 52n;
      c2 = 1n << 51n;
      u2 = c2 - 1n;
      F = c2 | u2;
      _ = u2 - (d3 << 42n);
      S2 = u2 - (E << 29n);
      m = { 2: "0b", 8: "0o", 16: "0x" };
      x2 = ((t4) => (t4[t4.NATURAL = -2] = "NATURAL", t4[t4.UNKNOWN = -1] = "UNKNOWN", t4[t4.F16 = 2] = "F16", t4[t4.F32 = 4] = "F32", t4[t4.F64 = 8] = "F64", t4))(x2 || {});
      O = class extends Number {
        #n;
        #t = -1;
        constructor(n2, e2 = true, r3 = -1) {
          super(NaN);
          const o5 = n2;
          if (typeof n2 == "number") {
            if (!Number.isSafeInteger(n2)) throw new Error(`Invalid NAN payload: ${n2}`);
            n2 = BigInt(n2);
            let t4 = 0n;
            if (n2 < 0 && (t4 = N, n2 = -n2), n2 >= c2) throw new Error(`Payload too large: ${o5}`);
            const s3 = e2 ? c2 : 0n;
            switch (this.#n = t4 | a | s3 | n2, r3) {
              case -2:
                throw new Error("NAN_SIZE.NATURAL only valid for bigint constructor");
              case -1:
                r3 = this.preferredSize;
                break;
              case 2:
                if (this.#n & _) throw new Error("Invalid size for payload");
                break;
              case 4:
                if (this.#n & S2) throw new Error("Invalid size for payload");
                break;
              case 8:
                break;
              default:
                throw new Error(`Invalid size: ${r3}`);
            }
            this.#t = r3;
          } else if (typeof n2 == "bigint") {
            let t4 = -1;
            if ((n2 & a) === a) this.#n = n2, t4 = 8;
            else if ((n2 & l2) === l2) {
              const s3 = (n2 & g2) << 32n;
              this.#n = s3 | a | (n2 & p2) << 29n, t4 = 4;
            } else if ((n2 & f5) === f5) {
              const s3 = (n2 & w) << 48n;
              this.#n = s3 | a | (n2 & I2) << 42n, t4 = 2;
            } else throw new Error(`Invalid raw NaN value: ${n2}`);
            if (r3 === -1) this.#t = this.preferredSize;
            else if (r3 === -2) this.#t = t4;
            else {
              if (r3 < t4) throw new Error("Invalid bigint NaN size");
              this.#t = r3;
            }
          } else {
            const t4 = new DataView(n2.buffer, n2.byteOffset, n2.byteLength);
            switch (n2.length) {
              case 3: {
                if (n2[0] !== 249) throw new Error("Invalid CBOR encoding for half float");
                const s3 = BigInt(t4.getUint16(1, false));
                if ((s3 & f5) !== f5) throw new Error("Not a NaN");
                const h4 = (s3 & w) << 48n;
                this.#n = h4 | a | (s3 & I2) << 42n, this.#t = 2;
                break;
              }
              case 5: {
                if (n2[0] !== 250) throw new Error("Invalid CBOR encoding for single float");
                const s3 = BigInt(t4.getUint32(1, false));
                if ((s3 & l2) !== l2) throw new Error("Not a NaN");
                const h4 = (s3 & g2) << 32n;
                this.#n = h4 | a | (s3 & p2) << 29n, this.#t = 4;
                break;
              }
              case 9: {
                if (n2[0] !== 251) throw new Error("Invalid CBOR encoding for double float");
                if (this.#n = t4.getBigUint64(1, false), (this.#n & a) !== a) throw new Error("Not a NaN (NaNaN)");
                this.#t = 8;
                break;
              }
              default:
                throw new RangeError(`Invalid NAN size (should be 2, 4, or 8): ${n2.length - 1}`);
            }
          }
          if (!this.payload && !this.quiet) throw new Error("Signalling NaN with zero payload");
        }
        get bytes() {
          const n2 = new ArrayBuffer(this.#t + 1), e2 = new DataView(n2);
          switch (this.#t) {
            case 2: {
              e2.setUint8(0, 249);
              const o5 = (this.#n & N ? w : 0n) | f5 | (this.#n & F) >> 42n;
              e2.setUint16(1, Number(o5), false);
              break;
            }
            case 4: {
              e2.setUint8(0, 250);
              const o5 = (this.#n & N ? g2 : 0n) | l2 | (this.#n & F) >> 29n;
              e2.setUint32(1, Number(o5), false);
              break;
            }
            case 8:
              e2.setUint8(0, 251), e2.setBigUint64(1, this.#n);
              break;
          }
          return new Uint8Array(n2);
        }
        get quiet() {
          return !!(this.#n & c2);
        }
        get sign() {
          return this.#n & N ? -1 : 1;
        }
        get payload() {
          return Number(this.#n & u2) * this.sign;
        }
        get raw() {
          return this.#n;
        }
        get encodingIndicator() {
          switch (this.#t) {
            case 2:
              return "_1";
            case 4:
              return "_2";
          }
          return "_3";
        }
        get size() {
          return this.#t;
        }
        get preferredSize() {
          return (this.#n & _) === 0n ? 2 : (this.#n & S2) === 0n ? 4 : 8;
        }
        get isShortestEncoding() {
          return this.preferredSize === this.#t;
        }
        toCBOR(n2) {
          n2.write(this.bytes);
        }
        toString(n2 = 10) {
          return U2(this, 1, {}, (e2) => (m[n2] ?? "") + e2.toString(n2));
        }
        [Symbol.for("nodejs.util.inspect.custom")](n2, e2, r3) {
          return U2(this, n2, e2, r3);
        }
      };
    }
  });

  // node_modules/@cto.af/wtf8/lib/errors.js
  var DecodeError, InvalidEncodingError;
  var init_errors = __esm({
    "node_modules/@cto.af/wtf8/lib/errors.js"() {
      DecodeError = class extends TypeError {
        code = "ERR_ENCODING_INVALID_ENCODED_DATA";
        constructor() {
          super("The encoded data was not valid for encoding wtf-8");
        }
      };
      InvalidEncodingError = class extends RangeError {
        code = "ERR_ENCODING_NOT_SUPPORTED";
        constructor(label) {
          super(`Invalid encoding: "${label}"`);
        }
      };
    }
  });

  // node_modules/@cto.af/wtf8/lib/const.js
  var BOM, EMPTY, MIN_HIGH_SURROGATE, MIN_LOW_SURROGATE, REPLACEMENT, WTF8;
  var init_const = __esm({
    "node_modules/@cto.af/wtf8/lib/const.js"() {
      BOM = 65279;
      EMPTY = new Uint8Array(0);
      MIN_HIGH_SURROGATE = 55296;
      MIN_LOW_SURROGATE = 56320;
      REPLACEMENT = 65533;
      WTF8 = "wtf-8";
    }
  });

  // node_modules/@cto.af/wtf8/lib/decode.js
  function isArrayBufferView(input) {
    return input && typeof input === "object" && !(input instanceof ArrayBuffer) && !(input instanceof SharedArrayBuffer) && input.buffer instanceof ArrayBuffer;
  }
  function getUint8(input) {
    if (!input) {
      return EMPTY;
    }
    if (input instanceof Uint8Array) {
      return input;
    }
    if (isArrayBufferView(input)) {
      return new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
    }
    return new Uint8Array(input);
  }
  var REMAINDER, Wtf8Decoder;
  var init_decode = __esm({
    "node_modules/@cto.af/wtf8/lib/decode.js"() {
      init_const();
      init_errors();
      REMAINDER = [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -1,
        -1,
        -1,
        -1,
        1,
        1,
        2,
        3
      ];
      Wtf8Decoder = class _Wtf8Decoder {
        static DEFAULT_BUFFERSIZE = 4096;
        encoding = WTF8;
        fatal;
        ignoreBOM;
        bufferSize;
        #left = 0;
        #cur = 0;
        #pending = 0;
        #first = true;
        #buf;
        constructor(label = "wtf8", options = void 0) {
          if (label.toLowerCase().replace("-", "") !== "wtf8") {
            throw new InvalidEncodingError(label);
          }
          this.fatal = Boolean(options?.fatal);
          this.ignoreBOM = Boolean(options?.ignoreBOM);
          this.bufferSize = Math.floor(options?.bufferSize ?? _Wtf8Decoder.DEFAULT_BUFFERSIZE);
          if (isNaN(this.bufferSize) || this.bufferSize < 1) {
            throw new RangeError(`Invalid buffer size: ${options?.bufferSize}`);
          }
          this.#buf = new Uint16Array(this.bufferSize);
        }
        decode(input, options) {
          const streaming = Boolean(options?.stream);
          const bytes = getUint8(input);
          const res = [];
          const out = this.#buf;
          const maxSize = this.bufferSize - 3;
          let pos = 0;
          const fatal = () => {
            this.#cur = 0;
            this.#left = 0;
            this.#pending = 0;
            if (this.fatal) {
              throw new DecodeError();
            }
            out[pos++] = REPLACEMENT;
          };
          const fatals = () => {
            const p5 = this.#pending;
            for (let i2 = 0; i2 < p5; i2++) {
              fatal();
            }
          };
          const oneByte = (b5) => {
            if (this.#left === 0) {
              const n2 = REMAINDER[b5 >> 4];
              switch (n2) {
                case -1:
                  fatal();
                  break;
                case 0:
                  out[pos++] = b5;
                  break;
                case 1:
                  this.#cur = b5 & 31;
                  if ((this.#cur & 30) === 0) {
                    fatal();
                  } else {
                    this.#left = 1;
                    this.#pending = 1;
                  }
                  break;
                case 2:
                  this.#cur = b5 & 15;
                  this.#left = 2;
                  this.#pending = 1;
                  break;
                case 3:
                  if (b5 & 8) {
                    fatal();
                  } else {
                    this.#cur = b5 & 7;
                    this.#left = 3;
                    this.#pending = 1;
                  }
                  break;
              }
            } else {
              if ((b5 & 192) !== 128) {
                fatals();
                return oneByte(b5);
              }
              if (this.#pending === 1 && this.#left === 2 && this.#cur === 0 && (b5 & 32) === 0) {
                fatals();
                return oneByte(b5);
              }
              if (this.#left === 3 && this.#cur === 0 && (b5 & 48) === 0) {
                fatals();
                return oneByte(b5);
              }
              this.#cur = this.#cur << 6 | b5 & 63;
              this.#pending++;
              if (--this.#left === 0) {
                if (this.ignoreBOM || !this.#first || this.#cur !== BOM) {
                  if (this.#cur < 65536) {
                    out[pos++] = this.#cur;
                  } else {
                    const cp = this.#cur - 65536;
                    out[pos++] = cp >>> 10 & 1023 | MIN_HIGH_SURROGATE;
                    out[pos++] = cp & 1023 | MIN_LOW_SURROGATE;
                  }
                }
                this.#cur = 0;
                this.#pending = 0;
                this.#first = false;
              }
            }
          };
          for (const b5 of bytes) {
            if (pos >= maxSize) {
              res.push(String.fromCharCode.apply(null, out.subarray(0, pos)));
              pos = 0;
            }
            oneByte(b5);
          }
          if (!streaming) {
            this.#first = true;
            if (this.#cur || this.#left) {
              fatals();
            }
          }
          if (pos > 0) {
            res.push(String.fromCharCode.apply(null, out.subarray(0, pos)));
          }
          return res.join("");
        }
      };
    }
  });

  // node_modules/@cto.af/wtf8/lib/encode.js
  function utf8length(str) {
    let len = 0;
    for (const s3 of str) {
      const cp = s3.codePointAt(0);
      if (cp < 128) {
        len++;
      } else if (cp < 2048) {
        len += 2;
      } else if (cp < 65536) {
        len += 3;
      } else {
        len += 4;
      }
    }
    return len;
  }
  var Wtf8Encoder;
  var init_encode = __esm({
    "node_modules/@cto.af/wtf8/lib/encode.js"() {
      init_const();
      Wtf8Encoder = class {
        encoding = WTF8;
        encode(input) {
          if (!input) {
            return EMPTY;
          }
          const buf = new Uint8Array(utf8length(String(input)));
          this.encodeInto(input, buf);
          return buf;
        }
        encodeInto(source, destination) {
          const str = String(source);
          const len = str.length;
          const outLen = destination.length;
          let written = 0;
          let read = 0;
          for (read = 0; read < len; read++) {
            const c5 = str.codePointAt(read);
            if (c5 < 128) {
              if (written >= outLen) {
                break;
              }
              destination[written++] = c5;
            } else if (c5 < 2048) {
              if (written >= outLen - 1) {
                break;
              }
              destination[written++] = 192 | c5 >> 6;
              destination[written++] = 128 | c5 & 63;
            } else if (c5 < 65536) {
              if (written >= outLen - 2) {
                break;
              }
              destination[written++] = 224 | c5 >> 12;
              destination[written++] = 128 | c5 >> 6 & 63;
              destination[written++] = 128 | c5 & 63;
            } else {
              if (written >= outLen - 3) {
                break;
              }
              destination[written++] = 240 | c5 >> 18;
              destination[written++] = 128 | c5 >> 12 & 63;
              destination[written++] = 128 | c5 >> 6 & 63;
              destination[written++] = 128 | c5 & 63;
              read++;
            }
          }
          return {
            read,
            written
          };
        }
      };
    }
  });

  // node_modules/@cto.af/wtf8/lib/decodeStream.js
  var init_decodeStream = __esm({
    "node_modules/@cto.af/wtf8/lib/decodeStream.js"() {
      init_decode();
    }
  });

  // node_modules/@cto.af/wtf8/lib/encodeStream.js
  var init_encodeStream = __esm({
    "node_modules/@cto.af/wtf8/lib/encodeStream.js"() {
      init_const();
      init_encode();
    }
  });

  // node_modules/@cto.af/wtf8/lib/index.js
  var init_lib = __esm({
    "node_modules/@cto.af/wtf8/lib/index.js"() {
      init_errors();
      init_decode();
      init_encode();
      init_decodeStream();
      init_encodeStream();
    }
  });

  // node_modules/cbor2/lib/encoder.js
  function y2(e2) {
    const n2 = e2 < 0;
    return typeof e2 == "bigint" ? [n2 ? -e2 - 1n : e2, n2] : [n2 ? -e2 - 1 : e2, n2];
  }
  function N2(e2, n2, i2) {
    if (i2.rejectFloats) throw new Error(`Attempt to encode an unwanted floating point number: ${e2}`);
    if (isNaN(e2)) n2.writeUint8(U3), n2.writeUint16(32256);
    else if (!i2.float64 && Math.fround(e2) === e2) {
      const r3 = Z(e2);
      r3 === null ? (n2.writeUint8(h2), n2.writeFloat32(e2)) : (n2.writeUint8(U3), n2.writeUint16(r3));
    } else n2.writeUint8(B2), n2.writeFloat64(e2);
  }
  function a2(e2, n2, i2) {
    const [r3, t4] = y2(e2);
    if (t4 && i2) throw new TypeError(`Negative size: ${e2}`);
    i2 ??= t4 ? f.NEG_INT : f.POS_INT, i2 <<= 5, r3 < 24 ? n2.writeUint8(i2 | r3) : r3 <= 255 ? (n2.writeUint8(i2 | I.ONE), n2.writeUint8(r3)) : r3 <= 65535 ? (n2.writeUint8(i2 | I.TWO), n2.writeUint16(r3)) : r3 <= 4294967295 ? (n2.writeUint8(i2 | I.FOUR), n2.writeUint32(r3)) : (n2.writeUint8(i2 | I.EIGHT), n2.writeBigUint64(BigInt(r3)));
  }
  function m2(e2, n2, i2) {
    typeof e2 == "number" ? a2(e2, n2, f.TAG) : typeof e2 == "object" && !i2.ignoreOriginalEncoding && A.ENCODED in e2 ? n2.write(e2[A.ENCODED]) : e2 <= Number.MAX_SAFE_INTEGER ? a2(Number(e2), n2, f.TAG) : (n2.writeUint8(f.TAG << 5 | I.EIGHT), n2.writeBigUint64(BigInt(e2)));
  }
  function O2(e2, n2, i2) {
    const [r3, t4] = y2(e2);
    if (i2.collapseBigInts && (!i2.largeNegativeAsBigInt || e2 >= -0x8000000000000000n)) {
      if (r3 <= 0xffffffffn) {
        a2(Number(e2), n2);
        return;
      }
      if (r3 <= 0xffffffffffffffffn) {
        const l6 = (t4 ? f.NEG_INT : f.POS_INT) << 5;
        n2.writeUint8(l6 | I.EIGHT), n2.writeBigUint64(r3);
        return;
      }
    }
    if (i2.rejectBigInts) throw new Error(`Attempt to encode unwanted bigint: ${e2}`);
    const o5 = t4 ? T.NEG_BIGINT : T.POS_BIGINT, c5 = r3.toString(16), s3 = c5.length % 2 ? "0" : "";
    m2(o5, n2, i2);
    const u3 = b(s3 + c5);
    a2(u3.length, n2, f.BYTE_STRING), n2.write(u3);
  }
  function Z2(e2, n2, i2) {
    i2.flushToZero && (e2 = k(e2)), Object.is(e2, -0) ? i2.simplifyNegativeZero ? i2.avoidInts ? N2(0, n2, i2) : a2(0, n2) : N2(e2, n2, i2) : !i2.avoidInts && Number.isSafeInteger(e2) ? a2(e2, n2) : i2.reduceUnsafeNumbers && Math.floor(e2) === e2 && e2 >= S.MIN && e2 <= S.MAX ? O2(BigInt(e2), n2, i2) : N2(e2, n2, i2);
  }
  function R2(e2, n2, i2) {
    const r3 = i2.stringNormalization ? e2.normalize(i2.stringNormalization) : e2;
    if (i2.wtf8 && !e2.isWellFormed()) {
      const t4 = K.encode(r3);
      m2(T.WTF8, n2, i2), a2(t4.length, n2, f.BYTE_STRING), n2.write(t4);
    } else {
      const t4 = H.encode(r3);
      a2(t4.length, n2, f.UTF8_STRING), n2.write(t4);
    }
  }
  function J(e2, n2, i2) {
    const r3 = e2;
    L(r3, r3.length, f.ARRAY, n2, i2);
    for (const t4 of r3) g3(t4, n2, i2);
  }
  function V(e2, n2) {
    a2(e2.length, n2, f.BYTE_STRING), n2.write(e2);
  }
  function ue(e2, n2) {
    return b3.registerEncoder(e2, n2);
  }
  function L(e2, n2, i2, r3, t4) {
    const o5 = s(e2);
    o5 && !t4.ignoreOriginalEncoding ? r3.write(o5) : a2(n2, r3, i2);
  }
  function X(e2, n2, i2) {
    if (e2 === null) {
      n2.writeUint8(z);
      return;
    }
    if (!i2.ignoreOriginalEncoding && A.ENCODED in e2) {
      n2.write(e2[A.ENCODED]);
      return;
    }
    const r3 = e2.constructor;
    if (r3) {
      const o5 = i2.types?.get(r3) ?? (i2.ignoreGlobalTags ? void 0 : b3.get(r3));
      if (o5) {
        const c5 = o5(e2, n2, i2);
        if (c5 !== void 0) {
          if (!Array.isArray(c5) || c5.length !== 2) throw new Error("Invalid encoder return value");
          (typeof c5[0] == "bigint" || isFinite(Number(c5[0]))) && m2(c5[0], n2, i2), g3(c5[1], n2, i2);
        }
        return;
      }
    }
    if (typeof e2.toCBOR == "function") {
      const o5 = e2.toCBOR(n2, i2);
      o5 && ((typeof o5[0] == "bigint" || isFinite(Number(o5[0]))) && m2(o5[0], n2, i2), g3(o5[1], n2, i2));
      return;
    }
    if (typeof e2.toJSON == "function") {
      g3(e2.toJSON(), n2, i2);
      return;
    }
    const t4 = Object.entries(e2).map((o5) => [o5[0], o5[1], v2(o5[0], i2)]);
    i2.sortKeys && t4.sort(i2.sortKeys), L(e2, t4.length, f.MAP, n2, i2);
    for (const [o5, c5, s3] of t4) n2.write(s3), g3(c5, n2, i2);
  }
  function Q(e2, n2, i2) {
    if (!e2.description || e2 !== Symbol.for(e2.description)) throw new TypeError(`Private or empty symbol: ${e2.toString()}`);
    m2(280, n2, i2), R2(e2.description, n2, i2);
  }
  function g3(e2, n2, i2) {
    switch (typeof e2) {
      case "number":
        Z2(e2, n2, i2);
        break;
      case "bigint":
        O2(e2, n2, i2);
        break;
      case "string":
        R2(e2, n2, i2);
        break;
      case "boolean":
        n2.writeUint8(e2 ? j : q);
        break;
      case "undefined":
        if (i2.rejectUndefined) throw new Error("Attempt to encode unwanted undefined.");
        n2.writeUint8($);
        break;
      case "object":
        X(e2, n2, i2);
        break;
      case "symbol":
        Q(e2, n2, i2);
        break;
      default:
        throw new TypeError(`Unknown type: ${typeof e2}, ${String(e2)}`);
    }
  }
  function v2(e2, n2 = {}) {
    const i2 = { ...k2 };
    n2.dcbor ? Object.assign(i2, Y) : n2.cde && Object.assign(i2, F2), Object.assign(i2, n2);
    const r3 = new e(i2);
    return g3(e2, r3, i2), r3.read();
  }
  var ce, U3, h2, B2, j, q, $, z, H, K, k2, F2, Y, b3;
  var init_encoder = __esm({
    "node_modules/cbor2/lib/encoder.js"() {
      init_typeEncoderMap();
      init_constants();
      init_sorts();
      init_writer();
      init_box();
      init_float();
      init_lib();
      init_utils();
      ({ ENCODED: ce } = A);
      U3 = f.SIMPLE_FLOAT << 5 | I.TWO;
      h2 = f.SIMPLE_FLOAT << 5 | I.FOUR;
      B2 = f.SIMPLE_FLOAT << 5 | I.EIGHT;
      j = f.SIMPLE_FLOAT << 5 | o.TRUE;
      q = f.SIMPLE_FLOAT << 5 | o.FALSE;
      $ = f.SIMPLE_FLOAT << 5 | o.UNDEFINED;
      z = f.SIMPLE_FLOAT << 5 | o.NULL;
      H = new TextEncoder();
      K = new Wtf8Encoder();
      k2 = { ...e.defaultOptions, avoidInts: false, cde: false, collapseBigInts: true, dateTag: T.DATE_EPOCH, dcbor: false, float64: false, flushToZero: false, forceEndian: null, ignoreOriginalEncoding: false, largeNegativeAsBigInt: false, reduceUnsafeNumbers: false, rejectBigInts: false, rejectCustomSimples: false, rejectDuplicateKeys: false, rejectFloats: false, rejectUndefined: false, simplifyNegativeZero: false, sortKeys: null, stringNormalization: null, types: null, wtf8: false, ignoreGlobalTags: false };
      F2 = { cde: true, ignoreOriginalEncoding: true, sortKeys: f4 };
      Y = { ...F2, dcbor: true, largeNegativeAsBigInt: true, reduceUnsafeNumbers: true, rejectCustomSimples: true, rejectDuplicateKeys: true, rejectUndefined: true, simplifyNegativeZero: true, stringNormalization: "NFC" };
      b3 = new s2();
      b3.registerEncoder(Array, J), b3.registerEncoder(Uint8Array, V);
    }
  });

  // node_modules/cbor2/lib/options.js
  var o3;
  var init_options = __esm({
    "node_modules/cbor2/lib/options.js"() {
      o3 = ((e2) => (e2[e2.NEVER = -1] = "NEVER", e2[e2.PREFERRED = 0] = "PREFERRED", e2[e2.ALWAYS = 1] = "ALWAYS", e2))(o3 || {});
    }
  });

  // node_modules/cbor2/lib/simple.js
  var t;
  var init_simple = __esm({
    "node_modules/cbor2/lib/simple.js"() {
      init_constants();
      init_encoder();
      t = class _t2 {
        static KnownSimple = /* @__PURE__ */ new Map([[o.FALSE, false], [o.TRUE, true], [o.NULL, null], [o.UNDEFINED, void 0]]);
        value;
        constructor(e2) {
          this.value = e2;
        }
        static create(e2) {
          return _t2.KnownSimple.has(e2) ? _t2.KnownSimple.get(e2) : new _t2(e2);
        }
        toCBOR(e2, i2) {
          if (i2.rejectCustomSimples) throw new Error(`Cannot encode non-standard Simple value: ${this.value}`);
          a2(this.value, e2, f.SIMPLE_FLOAT);
        }
        toString() {
          return `simple(${this.value})`;
        }
        decode() {
          return _t2.KnownSimple.has(this.value) ? _t2.KnownSimple.get(this.value) : this;
        }
        [Symbol.for("nodejs.util.inspect.custom")](e2, i2, r3) {
          return `simple(${r3(this.value, i2)})`;
        }
      };
    }
  });

  // node_modules/cbor2/lib/decodeStream.js
  var p3, y3;
  var init_decodeStream2 = __esm({
    "node_modules/cbor2/lib/decodeStream.js"() {
      init_constants();
      init_utils();
      init_simple();
      init_float();
      p3 = new TextDecoder("utf8", { fatal: true, ignoreBOM: true });
      y3 = class _y {
        static defaultOptions = { maxDepth: 1024, encoding: "hex", requirePreferred: false };
        #t;
        #r;
        #e = 0;
        #i;
        constructor(t4, r3) {
          if (this.#i = { ..._y.defaultOptions, ...r3 }, typeof t4 == "string") switch (this.#i.encoding) {
            case "hex":
              this.#t = b(t4);
              break;
            case "base64":
              this.#t = y(t4);
              break;
            default:
              throw new TypeError(`Encoding not implemented: "${this.#i.encoding}"`);
          }
          else this.#t = t4;
          this.#r = new DataView(this.#t.buffer, this.#t.byteOffset, this.#t.byteLength);
        }
        toHere(t4) {
          return R(this.#t, t4, this.#e);
        }
        *[Symbol.iterator]() {
          if (yield* this.#n(0), this.#e !== this.#t.length) throw new Error("Extra data in input");
        }
        *seq() {
          for (; this.#e < this.#t.length; ) yield* this.#n(0);
        }
        *#n(t4) {
          if (t4++ > this.#i.maxDepth) throw new Error(`Maximum depth ${this.#i.maxDepth} exceeded`);
          const r3 = this.#e, c5 = this.#r.getUint8(this.#e++), i2 = c5 >> 5, n2 = c5 & 31;
          let e2 = n2, f7 = false, a4 = 0;
          switch (n2) {
            case I.ONE:
              if (a4 = 1, e2 = this.#r.getUint8(this.#e), i2 === f.SIMPLE_FLOAT) {
                if (e2 < 32) throw new Error(`Invalid simple encoding in extra byte: ${e2}`);
                f7 = true;
              } else if (this.#i.requirePreferred && e2 < 24) throw new Error(`Unexpectedly long integer encoding (1) for ${e2}`);
              break;
            case I.TWO:
              if (a4 = 2, i2 === f.SIMPLE_FLOAT) e2 = v(this.#t, this.#e);
              else if (e2 = this.#r.getUint16(this.#e, false), this.#i.requirePreferred && e2 <= 255) throw new Error(`Unexpectedly long integer encoding (2) for ${e2}`);
              break;
            case I.FOUR:
              if (a4 = 4, i2 === f.SIMPLE_FLOAT) e2 = this.#r.getFloat32(this.#e, false);
              else if (e2 = this.#r.getUint32(this.#e, false), this.#i.requirePreferred && e2 <= 65535) throw new Error(`Unexpectedly long integer encoding (4) for ${e2}`);
              break;
            case I.EIGHT: {
              if (a4 = 8, i2 === f.SIMPLE_FLOAT) e2 = this.#r.getFloat64(this.#e, false);
              else if (e2 = this.#r.getBigUint64(this.#e, false), e2 <= Number.MAX_SAFE_INTEGER && (e2 = Number(e2)), this.#i.requirePreferred && e2 <= 4294967295) throw new Error(`Unexpectedly long integer encoding (8) for ${e2}`);
              break;
            }
            case 28:
            case 29:
            case 30:
              throw new Error(`Additional info not implemented: ${n2}`);
            case I.INDEFINITE:
              switch (i2) {
                case f.POS_INT:
                case f.NEG_INT:
                case f.TAG:
                  throw new Error(`Invalid indefinite encoding for MT ${i2}`);
                case f.SIMPLE_FLOAT:
                  yield [i2, n2, A.BREAK, r3, 0];
                  return;
              }
              e2 = 1 / 0;
              break;
            default:
              f7 = true;
          }
          switch (this.#e += a4, i2) {
            case f.POS_INT:
              yield [i2, n2, e2, r3, a4];
              break;
            case f.NEG_INT:
              yield [i2, n2, typeof e2 == "bigint" ? -1n - e2 : -1 - Number(e2), r3, a4];
              break;
            case f.BYTE_STRING:
              e2 === 1 / 0 ? yield* this.#s(i2, t4, r3) : yield [i2, n2, this.#a(e2), r3, e2];
              break;
            case f.UTF8_STRING:
              e2 === 1 / 0 ? yield* this.#s(i2, t4, r3) : yield [i2, n2, p3.decode(this.#a(e2)), r3, e2];
              break;
            case f.ARRAY:
              if (e2 === 1 / 0) yield* this.#s(i2, t4, r3, false);
              else {
                const o5 = Number(e2);
                yield [i2, n2, o5, r3, a4];
                for (let h4 = 0; h4 < o5; h4++) yield* this.#n(t4 + 1);
              }
              break;
            case f.MAP:
              if (e2 === 1 / 0) yield* this.#s(i2, t4, r3, false);
              else {
                const o5 = Number(e2);
                yield [i2, n2, o5, r3, a4];
                for (let h4 = 0; h4 < o5; h4++) yield* this.#n(t4), yield* this.#n(t4);
              }
              break;
            case f.TAG:
              yield [i2, n2, e2, r3, a4], yield* this.#n(t4);
              break;
            case f.SIMPLE_FLOAT: {
              const o5 = e2;
              f7 && (e2 = t.create(Number(e2))), yield [i2, n2, e2, r3, o5];
              break;
            }
          }
        }
        #a(t4) {
          const r3 = R(this.#t, this.#e, this.#e += t4);
          if (r3.length !== t4) throw new Error(`Unexpected end of stream reading ${t4} bytes, got ${r3.length}`);
          return r3;
        }
        *#s(t4, r3, c5, i2 = true) {
          for (yield [t4, I.INDEFINITE, 1 / 0, c5, 1 / 0]; ; ) {
            const n2 = this.#n(r3), e2 = n2.next(), [f7, a4, o5] = e2.value;
            if (o5 === A.BREAK) {
              yield e2.value, n2.next();
              return;
            }
            if (i2) {
              if (f7 !== t4) throw new Error(`Unmatched major type.  Expected ${t4}, got ${f7}.`);
              if (a4 === I.INDEFINITE) throw new Error("New stream started in typed stream");
            }
            yield e2.value, yield* n2;
          }
        }
      };
    }
  });

  // node_modules/cbor2/lib/container.js
  function F3(h4, r3) {
    return !r3.boxed && !r3.preferMap && h4.every(([i2]) => typeof i2 == "string") ? Object.fromEntries(h4) : new Map(h4);
  }
  var A4, R3, y4;
  var init_container = __esm({
    "node_modules/cbor2/lib/container.js"() {
      init_constants();
      init_options();
      init_sorts();
      init_float();
      init_box();
      init_encoder();
      init_utils();
      init_decodeStream2();
      init_simple();
      init_tag();
      A4 = /* @__PURE__ */ new Map([[I.ZERO, 1], [I.ONE, 2], [I.TWO, 3], [I.FOUR, 5], [I.EIGHT, 9]]);
      R3 = new Uint8Array(0);
      y4 = class _y {
        static defaultDecodeOptions = { ...y3.defaultOptions, ParentType: _y, boxed: false, cde: false, dcbor: false, diagnosticSizes: o3.PREFERRED, collapseBigInts: false, convertUnsafeIntsToFloat: false, createObject: F3, keepNanPayloads: false, pretty: false, preferBigInt: false, preferMap: false, rejectLargeNegatives: false, rejectBigInts: false, rejectDuplicateKeys: false, rejectFloats: false, rejectInts: false, rejectLongLoundNaN: false, rejectLongFloats: false, rejectNegativeZero: false, rejectSimple: false, rejectStreaming: false, rejectStringsNotNormalizedAs: null, rejectSubnormals: false, rejectUndefined: false, rejectUnsafeFloatInts: false, saveOriginal: false, sortKeys: null, tags: null, ignoreGlobalTags: false };
        static cdeDecodeOptions = { cde: true, rejectStreaming: true, requirePreferred: true, sortKeys: f4 };
        static dcborDecodeOptions = { ...this.cdeDecodeOptions, dcbor: true, convertUnsafeIntsToFloat: true, rejectDuplicateKeys: true, rejectLargeNegatives: true, rejectLongLoundNaN: true, rejectLongFloats: true, rejectNegativeZero: true, rejectSimple: true, rejectUndefined: true, rejectUnsafeFloatInts: true, rejectStringsNotNormalizedAs: "NFC" };
        parent;
        mt;
        ai;
        left;
        offset;
        count = 0;
        children = [];
        depth = 0;
        #e;
        #t = null;
        constructor(r3, i2, e2, t4) {
          if ([this.mt, this.ai, , this.offset] = r3, this.left = i2, this.parent = e2, this.#e = t4, e2 && (this.depth = e2.depth + 1), this.mt === f.MAP && (this.#e.sortKeys || this.#e.rejectDuplicateKeys) && (this.#t = []), this.#e.rejectStreaming && this.ai === I.INDEFINITE) throw new Error("Streaming not supported");
        }
        get isStreaming() {
          return this.left === 1 / 0;
        }
        get done() {
          return this.left === 0;
        }
        static create(r3, i2, e2, t4) {
          const [s3, u3, n2, c5] = r3;
          switch (s3) {
            case f.POS_INT:
            case f.NEG_INT: {
              if (e2.rejectInts) throw new Error(`Unexpected integer: ${n2}`);
              if (e2.rejectLargeNegatives && n2 < -0x8000000000000000n) throw new Error(`Invalid 65bit negative number: ${n2}`);
              let o5 = n2;
              return e2.preferBigInt ? o5 = BigInt(o5) : e2.convertUnsafeIntsToFloat && o5 >= S.MIN && o5 <= S.MAX && (o5 = Number(n2)), e2.boxed ? d(o5, t4.toHere(c5)) : o5;
            }
            case f.SIMPLE_FLOAT:
              if (u3 > I.ONE) {
                if (typeof n2 == "symbol") return n2;
                if (e2.rejectFloats) throw new Error(`Decoding unwanted floating point number: ${n2}`);
                if (e2.rejectNegativeZero && Object.is(n2, -0)) throw new Error("Decoding negative zero");
                if (isNaN(n2)) {
                  const o5 = t4.toHere(c5), f7 = new O(o5);
                  if (e2.rejectLongLoundNaN) {
                    if (f7.payload || o5.length > 3) throw new Error(`Invalid NaN encoding: "${A2(o5)}"`);
                  } else if (e2.keepNanPayloads && (f7.payload || f7.sign === -1)) {
                    if (e2.rejectLongFloats && !f7.isShortestEncoding) throw new Error(`NaN should have been encoded shorter: ${n2}`);
                    return f7;
                  }
                }
                if (e2.rejectSubnormals && B(t4.toHere(c5 + 1)), e2.rejectLongFloats) {
                  const o5 = v2(n2, { chunkSize: 9, reduceUnsafeNumbers: e2.rejectUnsafeFloatInts });
                  if (o5[0] >> 5 !== s3) throw new Error(`Should have been encoded as int, not float: ${n2}`);
                  if (o5.length < A4.get(u3)) throw new Error(`Number should have been encoded shorter: ${n2}`);
                }
                if (typeof n2 == "number" && e2.boxed) return d(n2, t4.toHere(c5));
              } else {
                if (e2.rejectSimple && n2 instanceof t) throw new Error(`Invalid simple value: ${n2}`);
                if (e2.rejectUndefined && n2 === void 0) throw new Error("Unexpected undefined");
              }
              return n2;
            case f.BYTE_STRING:
            case f.UTF8_STRING:
              if (n2 === 1 / 0) return new e2.ParentType(r3, 1 / 0, i2, e2);
              if (e2.rejectStringsNotNormalizedAs && typeof n2 == "string") {
                const o5 = n2.normalize(e2.rejectStringsNotNormalizedAs);
                if (n2 !== o5) throw new Error(`String not normalized as "${e2.rejectStringsNotNormalizedAs}", got [${U(n2)}] instead of [${U(o5)}]`);
              }
              return e2.boxed ? d(n2, t4.toHere(c5)) : n2;
            case f.ARRAY:
              return new e2.ParentType(r3, n2, i2, e2);
            case f.MAP:
              return new e2.ParentType(r3, n2 * 2, i2, e2);
            case f.TAG: {
              const o5 = new e2.ParentType(r3, 1, i2, e2);
              return o5.children = new o2(n2), o5;
            }
          }
          throw new TypeError(`Invalid major type: ${s3}`);
        }
        static decodeToEncodeOpts(r3) {
          return { ...k2, avoidInts: r3.rejectInts, float64: !r3.rejectLongFloats, flushToZero: r3.rejectSubnormals, largeNegativeAsBigInt: r3.rejectLargeNegatives, sortKeys: r3.sortKeys };
        }
        push(r3, i2, e2) {
          if (this.children.push(r3), this.#t) {
            const t4 = f2(r3) || i2.toHere(e2);
            this.#t.push(t4);
          }
          return --this.left;
        }
        replaceLast(r3, i2, e2) {
          let t4, s3 = -1 / 0;
          if (this.children instanceof o2 ? (s3 = 0, t4 = this.children.contents, this.children.contents = r3) : (s3 = this.children.length - 1, t4 = this.children[s3], this.children[s3] = r3), this.#t) {
            const u3 = f2(r3) || e2.toHere(i2.offset);
            this.#t[s3] = u3;
          }
          return t4;
        }
        convert(r3) {
          let i2;
          switch (this.mt) {
            case f.ARRAY:
              i2 = this.children;
              break;
            case f.MAP: {
              const e2 = this.#r();
              if (this.#e.sortKeys) {
                let t4;
                for (const s3 of e2) {
                  if (t4 && this.#e.sortKeys(t4, s3) >= 0) throw new Error(`Duplicate or out of order key: "0x${s3[2]}"`);
                  t4 = s3;
                }
              } else if (this.#e.rejectDuplicateKeys) {
                const t4 = /* @__PURE__ */ new Set();
                for (const [s3, u3, n2] of e2) {
                  const c5 = A2(n2);
                  if (t4.has(c5)) throw new Error(`Duplicate key: "0x${c5}"`);
                  t4.add(c5);
                }
              }
              i2 = this.#e.createObject(e2, this.#e);
              break;
            }
            case f.BYTE_STRING:
              return d2(this.children);
            case f.UTF8_STRING: {
              const e2 = this.children.join("");
              i2 = this.#e.boxed ? d(e2, r3.toHere(this.offset)) : e2;
              break;
            }
            case f.TAG:
              i2 = this.children.decode(this.#e);
              break;
            default:
              throw new TypeError(`Invalid mt on convert: ${this.mt}`);
          }
          return this.#e.saveOriginal && i2 && typeof i2 == "object" && u(i2, r3.toHere(this.offset)), i2;
        }
        #r() {
          const r3 = this.children, i2 = r3.length;
          if (i2 % 2) throw new Error("Missing map value");
          const e2 = new Array(i2 / 2);
          if (this.#t) for (let t4 = 0; t4 < i2; t4 += 2) e2[t4 >> 1] = [r3[t4], r3[t4 + 1], this.#t[t4]];
          else for (let t4 = 0; t4 < i2; t4 += 2) e2[t4 >> 1] = [r3[t4], r3[t4 + 1], R3];
          return e2;
        }
      };
    }
  });

  // node_modules/cbor2/lib/diagnostic.js
  function a3(m4, l6, n2, p5) {
    let t4 = "";
    if (l6 === I.INDEFINITE) t4 += "_";
    else {
      if (p5.diagnosticSizes === o3.NEVER) return "";
      {
        let r3 = p5.diagnosticSizes === o3.ALWAYS;
        if (!r3) {
          let e2 = I.ZERO;
          if (Object.is(n2, -0)) e2 = I.TWO;
          else if (m4 === f.POS_INT || m4 === f.NEG_INT) {
            const T2 = n2 < 0, u3 = typeof n2 == "bigint" ? 1n : 1, o5 = T2 ? -n2 - u3 : n2;
            o5 <= 23 ? e2 = Number(o5) : o5 <= 255 ? e2 = I.ONE : o5 <= 65535 ? e2 = I.TWO : o5 <= 4294967295 ? e2 = I.FOUR : e2 = I.EIGHT;
          } else isFinite(n2) ? Math.fround(n2) === n2 ? Z(n2) == null ? e2 = I.FOUR : e2 = I.TWO : e2 = I.EIGHT : e2 = I.TWO;
          r3 = e2 !== l6;
        }
        r3 && (t4 += "_", l6 < I.ONE ? t4 += "i" : t4 += String(l6 - 24));
      }
    }
    return t4;
  }
  function M(m4, l6) {
    const n2 = { ...y4.defaultDecodeOptions, ...l6, ParentType: g4 }, p5 = new y3(m4, n2);
    let t4, r3, e2 = "";
    for (const T2 of p5) {
      const [u3, o5, i2] = T2;
      switch (t4 && (t4.count > 0 && i2 !== A.BREAK && (t4.mt === f.MAP && t4.count % 2 ? e2 += ": " : (e2 += ",", n2.pretty || (e2 += " "))), n2.pretty && (t4.mt !== f.MAP || t4.count % 2 === 0) && (e2 += `
${O3.repeat(t4.depth + 1)}`)), r3 = y4.create(T2, t4, n2, p5), u3) {
        case f.POS_INT:
        case f.NEG_INT:
          e2 += String(i2), e2 += a3(u3, o5, i2, n2);
          break;
        case f.SIMPLE_FLOAT:
          if (i2 !== A.BREAK) if (typeof i2 == "number") {
            const c5 = Object.is(i2, -0) ? "-0.0" : String(i2);
            e2 += c5, isFinite(i2) && !/[.e]/.test(c5) && (e2 += ".0"), e2 += a3(u3, o5, i2, n2);
          } else i2 instanceof t ? (e2 += "simple(", e2 += String(i2.value), e2 += a3(f.POS_INT, o5, i2.value, n2), e2 += ")") : e2 += String(i2);
          break;
        case f.BYTE_STRING:
          i2 === 1 / 0 ? (e2 += "(_ ", r3.close = ")", r3.quote = "'") : (e2 += "h'", e2 += A2(i2), e2 += "'", e2 += a3(f.POS_INT, o5, i2.length, n2));
          break;
        case f.UTF8_STRING:
          i2 === 1 / 0 ? (e2 += "(_ ", r3.close = ")") : (e2 += JSON.stringify(i2), e2 += a3(f.POS_INT, o5, y5.encode(i2).length, n2));
          break;
        case f.ARRAY: {
          e2 += "[";
          const c5 = a3(f.POS_INT, o5, i2, n2);
          e2 += c5, c5 && (e2 += " "), n2.pretty && i2 ? r3.close = `
${O3.repeat(r3.depth)}]` : r3.close = "]";
          break;
        }
        case f.MAP: {
          e2 += "{";
          const c5 = a3(f.POS_INT, o5, i2, n2);
          e2 += c5, c5 && (e2 += " "), n2.pretty && i2 ? r3.close = `
${O3.repeat(r3.depth)}}` : r3.close = "}";
          break;
        }
        case f.TAG:
          e2 += String(i2), e2 += a3(f.POS_INT, o5, i2, n2), e2 += "(", r3.close = ")";
          break;
      }
      if (r3 === A.BREAK) if (t4?.isStreaming) t4.left = 0;
      else throw new Error("Unexpected BREAK");
      else t4 && (t4.count++, t4.left--);
      for (r3 instanceof g4 && (t4 = r3); t4?.done; ) {
        if (t4.isEmptyStream) e2 = e2.slice(0, -3), e2 += `${t4.quote}${t4.quote}_`;
        else {
          if (t4.mt === f.MAP && t4.count % 2 !== 0) throw new Error(`Odd streaming map size: ${t4.count}`);
          e2 += t4.close;
        }
        t4 = t4.parent;
      }
    }
    return e2;
  }
  var O3, y5, g4;
  var init_diagnostic = __esm({
    "node_modules/cbor2/lib/diagnostic.js"() {
      init_options();
      init_constants();
      init_container();
      init_decodeStream2();
      init_simple();
      init_float();
      init_utils();
      O3 = "  ";
      y5 = new TextEncoder();
      g4 = class extends y4 {
        close = "";
        quote = '"';
        get isEmptyStream() {
          return (this.mt === f.UTF8_STRING || this.mt === f.BYTE_STRING) && this.count === 0;
        }
      };
    }
  });

  // node_modules/cbor2/lib/comment.js
  function k3(t4) {
    return t4 instanceof A5;
  }
  function O4(t4, a4) {
    return t4 === 1 / 0 ? "Indefinite" : a4 ? `${t4} ${a4}${t4 !== 1 && t4 !== 1n ? "s" : ""}` : String(t4);
  }
  function y6(t4) {
    return "".padStart(t4, " ");
  }
  function x3(t4, a4, f7) {
    let e2 = "";
    e2 += y6(t4.depth * 2);
    const n2 = f2(t4);
    e2 += A2(n2.subarray(0, 1));
    const r3 = t4.numBytes();
    r3 && (e2 += " ", e2 += A2(n2.subarray(1, r3 + 1))), e2 = e2.padEnd(a4.minCol + 1, " "), e2 += "-- ", f7 !== void 0 && (e2 += y6(t4.depth * 2), f7 !== "" && (e2 += `[${f7}] `));
    let p5 = false;
    const [s3] = t4.children;
    switch (t4.mt) {
      case f.POS_INT:
        e2 += `Unsigned: ${s3}`, typeof s3 == "bigint" && (e2 += "n");
        break;
      case f.NEG_INT:
        e2 += `Negative: ${s3}`, typeof s3 == "bigint" && (e2 += "n");
        break;
      case f.BYTE_STRING:
        e2 += `Bytes (Length: ${O4(t4.length)})`;
        break;
      case f.UTF8_STRING:
        e2 += `UTF8 (Length: ${O4(t4.length)})`, t4.length !== 1 / 0 && (e2 += `: ${JSON.stringify(s3)}`);
        break;
      case f.ARRAY:
        e2 += `Array (Length: ${O4(t4.value, "item")})`;
        break;
      case f.MAP:
        e2 += `Map (Length: ${O4(t4.value, "pair")})`;
        break;
      case f.TAG: {
        e2 += `Tag #${t4.value}`;
        const o5 = t4.children, [m4] = o5.contents.children, i2 = new o2(o5.tag, m4);
        u(i2, n2);
        const l6 = i2.comment(a4, t4.depth);
        l6 && (e2 += ": ", e2 += l6), p5 ||= i2.noChildren;
        break;
      }
      case f.SIMPLE_FLOAT:
        s3 === A.BREAK ? e2 += "BREAK" : t4.ai > I.ONE ? Object.is(s3, -0) ? e2 += "Float: -0" : e2 += `Float: ${s3}` : (e2 += "Simple: ", s3 instanceof t ? e2 += s3.value : e2 += s3);
        break;
    }
    if (!p5) if (t4.leaf) {
      if (e2 += `
`, n2.length > r3 + 1) {
        const o5 = y6((t4.depth + 1) * 2), m4 = f3(n2);
        if (m4?.length) {
          m4.sort((l6, c5) => {
            const g5 = l6[0] - c5[0];
            return g5 || c5[1] - l6[1];
          });
          let i2 = 0;
          for (const [l6, c5, g5] of m4) if (!(l6 < i2)) {
            if (i2 = l6 + c5, g5 === "<<") {
              e2 += y6(a4.minCol + 1), e2 += "--", e2 += o5, e2 += "<< ";
              const d6 = R(n2, l6, l6 + c5), h4 = f3(d6);
              if (h4) {
                const $3 = h4.findIndex(([w3, D, v3]) => w3 === 0 && D === c5 && v3 === "<<");
                $3 >= 0 && h4.splice($3, 1);
              }
              e2 += M(d6), e2 += ` >>
`, e2 += L2(d6, { initialDepth: t4.depth + 1, minCol: a4.minCol, noPrefixHex: true });
              continue;
            } else g5 === "'" && (e2 += y6(a4.minCol + 1), e2 += "--", e2 += o5, e2 += "'", e2 += H2.decode(n2.subarray(l6, l6 + c5)), e2 += `'
`);
            if (l6 > r3) for (let d6 = l6; d6 < l6 + c5; d6 += 8) {
              const h4 = Math.min(d6 + 8, l6 + c5);
              e2 += o5, e2 += A2(n2.subarray(d6, h4)), e2 += `
`;
            }
          }
        } else for (let i2 = r3 + 1; i2 < n2.length; i2 += 8) e2 += o5, e2 += A2(n2.subarray(i2, i2 + 8)), e2 += `
`;
      }
    } else {
      e2 += `
`;
      let o5 = 0;
      for (const m4 of t4.children) {
        if (k3(m4)) {
          let i2 = String(o5);
          t4.mt === f.MAP ? i2 = o5 % 2 ? `val ${(o5 - 1) / 2}` : `key ${o5 / 2}` : t4.mt === f.TAG && (i2 = ""), e2 += x3(m4, a4, i2);
        }
        o5++;
      }
    }
    return e2;
  }
  function L2(t4, a4) {
    const f7 = { ...q2, ...a4, ParentType: A5, saveOriginal: true }, e2 = new y3(t4, f7);
    let n2, r3;
    for (const s3 of e2) {
      if (r3 = y4.create(s3, n2, f7, e2), s3[2] === A.BREAK) if (n2?.isStreaming) n2.left = 1;
      else throw new Error("Unexpected BREAK");
      if (!k3(r3)) {
        const i2 = new A5(s3, 0, n2, f7);
        i2.leaf = true, i2.children.push(r3), u(i2, e2.toHere(s3[3])), r3 = i2;
      }
      let o5 = (r3.depth + 1) * 2;
      const m4 = r3.numBytes();
      for (m4 && (o5 += 1, o5 += m4 * 2), f7.minCol = Math.max(f7.minCol, o5), n2 && n2.push(r3, e2, s3[3]), n2 = r3; n2?.done; ) r3 = n2, r3.leaf || u(r3, e2.toHere(r3.offset)), { parent: n2 } = n2;
    }
    a4 && (a4.minCol = f7.minCol);
    let p5 = f7.noPrefixHex ? "" : `0x${A2(e2.toHere(0))}
`;
    return p5 += x3(r3, f7), p5;
  }
  var H2, A5, q2;
  var init_comment = __esm({
    "node_modules/cbor2/lib/comment.js"() {
      init_constants();
      init_box();
      init_utils();
      init_container();
      init_decodeStream2();
      init_simple();
      init_tag();
      init_diagnostic();
      H2 = new TextDecoder();
      A5 = class extends y4 {
        depth = 0;
        leaf = false;
        value;
        length;
        [A.ENCODED];
        constructor(a4, f7, e2, n2) {
          super(a4, f7, e2, n2), this.parent ? this.depth = this.parent.depth + 1 : this.depth = n2.initialDepth, [, , this.value, , this.length] = a4;
        }
        numBytes() {
          switch (this.ai) {
            case I.ONE:
              return 1;
            case I.TWO:
              return 2;
            case I.FOUR:
              return 4;
            case I.EIGHT:
              return 8;
          }
          return 0;
        }
      };
      q2 = { ...y4.defaultDecodeOptions, initialDepth: 0, noPrefixHex: false, minCol: 0 };
    }
  });

  // node_modules/cbor2/lib/types.js
  function I3(e2) {
    if (typeof e2 == "object" && e2) {
      if (e2.constructor !== Number) throw new Error(`Expected number: ${e2}`);
    } else if (typeof e2 != "number") throw new Error(`Expected number: ${e2}`);
  }
  function f6(e2) {
    if (typeof e2 == "object" && e2) {
      if (e2.constructor !== String) throw new Error(`Expected string: ${e2}`);
    } else if (typeof e2 != "string") throw new Error(`Expected string: ${e2}`);
  }
  function E2(e2) {
    if (!(e2 instanceof Uint8Array)) throw new Error(`Expected Uint8Array: ${e2}`);
  }
  function _2(e2) {
    if (!Array.isArray(e2)) throw new Error(`Expected Array: ${e2}`);
  }
  function w2(e2) {
    return f6(e2.contents), new Date(e2.contents);
  }
  function N3(e2) {
    return I3(e2.contents), new Date(e2.contents * 1e3);
  }
  function $2(e2) {
    return I3(e2.contents), new Date(e2.contents * S3);
  }
  function m3(e2, r3, n2) {
    if (E2(r3.contents), n2.rejectBigInts) throw new Error(`Decoding unwanted big integer: ${r3}(h'${A2(r3.contents)}')`);
    if (n2.requirePreferred && r3.contents[0] === 0) throw new Error(`Decoding overly-large bigint: ${r3.tag}(h'${A2(r3.contents)})`);
    let i2 = r3.contents.reduce((d6, u3) => d6 << 8n | BigInt(u3), 0n);
    e2 && (i2 = -1n - i2);
    const a4 = i2 >= Number.MIN_SAFE_INTEGER && i2 <= Number.MAX_SAFE_INTEGER;
    if (n2.requirePreferred && a4) throw new Error(`Decoding bigint that could have been int: ${i2}n`);
    return n2.collapseBigInts && a4 && (i2 = Number(i2)), n2.boxed ? d(i2, r3.contents) : i2;
  }
  function R4(e2, r3) {
    return E2(e2.contents), e2;
  }
  function c3(e2, r3, n2) {
    E2(e2.contents);
    let i2 = e2.contents.length;
    if (i2 % r3.BYTES_PER_ELEMENT !== 0) throw new Error(`Number of bytes must be divisible by ${r3.BYTES_PER_ELEMENT}, got: ${i2}`);
    i2 /= r3.BYTES_PER_ELEMENT;
    const a4 = new r3(i2), d6 = new DataView(e2.contents.buffer, e2.contents.byteOffset, e2.contents.byteLength), u3 = d6[`get${r3.name.replace(/Array/, "")}`].bind(d6);
    for (let g5 = 0; g5 < i2; g5++) a4[g5] = u3(g5 * r3.BYTES_PER_ELEMENT, n2);
    return a4;
  }
  function l3(e2, r3, n2, i2, a4) {
    const d6 = a4.forceEndian ?? U4;
    if (m2(d6 ? r3 : n2, e2, a4), a2(i2.byteLength, e2, f.BYTE_STRING), U4 === d6) e2.write(new Uint8Array(i2.buffer, i2.byteOffset, i2.byteLength));
    else {
      const g5 = `write${i2.constructor.name.replace(/Array/, "")}`, y7 = e2[g5].bind(e2);
      for (const D of i2) y7(D, d6);
    }
  }
  function C(e2) {
    return E2(e2.contents), new Wtf8Decoder().decode(e2.contents);
  }
  function h3(e2) {
    throw new Error(`Encoding ${e2.constructor.name} intentionally unimplmented.  It is not concrete enough to interoperate.  Convert to Uint8Array first.`);
  }
  function p4(e2) {
    return [NaN, e2.valueOf()];
  }
  var U4, S3, L3, x4;
  var init_types = __esm({
    "node_modules/cbor2/lib/types.js"() {
      init_constants();
      init_box();
      init_utils();
      init_encoder();
      init_container();
      init_tag();
      init_lib();
      init_comment();
      U4 = !h();
      ue(Map, (e2, r3, n2) => {
        const i2 = [...e2.entries()].map((a4) => [a4[0], a4[1], v2(a4[0], n2)]);
        if (n2.rejectDuplicateKeys) {
          const a4 = /* @__PURE__ */ new Set();
          for (const [d6, u3, g5] of i2) {
            const y7 = A2(g5);
            if (a4.has(y7)) throw new Error(`Duplicate map key: 0x${y7}`);
            a4.add(y7);
          }
        }
        n2.sortKeys && i2.sort(n2.sortKeys), L(e2, e2.size, f.MAP, r3, n2);
        for (const [a4, d6, u3] of i2) r3.write(u3), g3(d6, r3, n2);
      });
      w2.comment = (e2) => {
        f6(e2.contents);
        const r3 = new Date(e2.contents);
        return `(String ${e2.tag === T.DATE_FULL ? "Full " : ""}Date) ${r3.toISOString()}`;
      }, o2.registerDecoder(T.DATE_STRING, w2), o2.registerDecoder(T.DATE_FULL, w2);
      N3.comment = (e2) => (I3(e2.contents), `(Epoch Date) ${new Date(e2.contents * 1e3).toISOString()}`), o2.registerDecoder(T.DATE_EPOCH, N3);
      S3 = 1e3 * 60 * 60 * 24;
      $2.comment = (e2) => (I3(e2.contents), `(Epoch Date) ${new Date(e2.contents * S3).toISOString()}`), o2.registerDecoder(T.DATE_EPOCH_DAYS, $2), ue(Date, (e2, r3, n2) => {
        switch (n2.dateTag) {
          case T.DATE_EPOCH:
            return [n2.dateTag, e2.valueOf() / 1e3];
          case T.DATE_STRING:
            return [n2.dateTag, e2.toISOString().replace(/\.000Z$/, "Z")];
          case T.DATE_EPOCH_DAYS:
            return [n2.dateTag, Math.floor(e2.valueOf() / S3)];
          case T.DATE_FULL:
            return [n2.dateTag, e2.toISOString().split("T")[0]];
          default:
            throw new Error(`Unsupported date tag: ${n2.dateTag}`);
        }
      });
      L3 = m3.bind(null, false);
      x4 = m3.bind(null, true);
      L3.comment = (e2, r3) => `(Positive BigInt) ${m3(false, e2, r3)}n`, x4.comment = (e2, r3) => `(Negative BigInt) ${m3(true, e2, r3)}n`, o2.registerDecoder(T.POS_BIGINT, L3), o2.registerDecoder(T.NEG_BIGINT, x4);
      R4.comment = (e2, r3, n2) => {
        E2(e2.contents);
        const i2 = { ...r3, initialDepth: n2 + 2, noPrefixHex: true }, a4 = f2(e2);
        let u3 = 2 ** ((a4[0] & 31) - 24) + 1;
        const g5 = a4[u3] & 31;
        let y7 = A2(a4.subarray(u3, ++u3));
        g5 >= 24 && (y7 += " ", y7 += A2(a4.subarray(u3, u3 + 2 ** (g5 - 24)))), i2.minCol = Math.max(i2.minCol, (n2 + 1) * 2 + y7.length);
        const D = L2(e2.contents, i2);
        let T2 = `Embedded CBOR
`;
        return T2 += `${"".padStart((n2 + 1) * 2, " ")}${y7}`.padEnd(i2.minCol + 1, " "), T2 += `-- Bytes (Length: ${e2.contents.length})
`, T2 += D, T2;
      }, R4.noChildren = true, o2.registerDecoder(T.CBOR, R4), o2.registerDecoder(T.URI, (e2) => (f6(e2.contents), new URL(e2.contents)), "URI"), ue(URL, (e2) => [T.URI, e2.toString()]), o2.registerDecoder(T.BASE64URL, (e2) => (f6(e2.contents), x(e2.contents)), "Base64url-encoded"), o2.registerDecoder(T.BASE64, (e2) => (f6(e2.contents), y(e2.contents)), "Base64-encoded"), o2.registerDecoder(35, (e2) => (f6(e2.contents), new RegExp(e2.contents)), "RegExp"), o2.registerDecoder(21065, (e2) => {
        f6(e2.contents);
        const r3 = `^(?:${e2.contents})$`;
        return new RegExp(r3, "u");
      }, "I-RegExp"), o2.registerDecoder(T.REGEXP, (e2) => {
        if (_2(e2.contents), e2.contents.length < 1 || e2.contents.length > 2) throw new Error(`Invalid RegExp Array: ${e2.contents}`);
        return new RegExp(e2.contents[0], e2.contents[1]);
      }, "RegExp"), ue(RegExp, (e2) => [T.REGEXP, [e2.source, e2.flags]]), o2.registerDecoder(64, (e2) => (E2(e2.contents), e2.contents), "uint8 Typed Array");
      o2.registerDecoder(65, (e2) => c3(e2, Uint16Array, false), "uint16, big endian, Typed Array"), o2.registerDecoder(66, (e2) => c3(e2, Uint32Array, false), "uint32, big endian, Typed Array"), o2.registerDecoder(67, (e2) => c3(e2, BigUint64Array, false), "uint64, big endian, Typed Array"), o2.registerDecoder(68, (e2) => (E2(e2.contents), new Uint8ClampedArray(e2.contents)), "uint8 Typed Array, clamped arithmetic"), ue(Uint8ClampedArray, (e2) => [68, new Uint8Array(e2.buffer, e2.byteOffset, e2.byteLength)]), o2.registerDecoder(69, (e2) => c3(e2, Uint16Array, true), "uint16, little endian, Typed Array"), ue(Uint16Array, (e2, r3, n2) => l3(r3, 69, 65, e2, n2)), o2.registerDecoder(70, (e2) => c3(e2, Uint32Array, true), "uint32, little endian, Typed Array"), ue(Uint32Array, (e2, r3, n2) => l3(r3, 70, 66, e2, n2)), o2.registerDecoder(71, (e2) => c3(e2, BigUint64Array, true), "uint64, little endian, Typed Array"), ue(BigUint64Array, (e2, r3, n2) => l3(r3, 71, 67, e2, n2)), o2.registerDecoder(72, (e2) => (E2(e2.contents), new Int8Array(e2.contents)), "sint8 Typed Array"), ue(Int8Array, (e2) => [72, new Uint8Array(e2.buffer, e2.byteOffset, e2.byteLength)]), o2.registerDecoder(73, (e2) => c3(e2, Int16Array, false), "sint16, big endian, Typed Array"), o2.registerDecoder(74, (e2) => c3(e2, Int32Array, false), "sint32, big endian, Typed Array"), o2.registerDecoder(75, (e2) => c3(e2, BigInt64Array, false), "sint64, big endian, Typed Array"), o2.registerDecoder(77, (e2) => c3(e2, Int16Array, true), "sint16, little endian, Typed Array"), ue(Int16Array, (e2, r3, n2) => l3(r3, 77, 73, e2, n2)), o2.registerDecoder(78, (e2) => c3(e2, Int32Array, true), "sint32, little endian, Typed Array"), ue(Int32Array, (e2, r3, n2) => l3(r3, 78, 74, e2, n2)), o2.registerDecoder(79, (e2) => c3(e2, BigInt64Array, true), "sint64, little endian, Typed Array"), ue(BigInt64Array, (e2, r3, n2) => l3(r3, 79, 75, e2, n2)), o2.registerDecoder(81, (e2) => c3(e2, Float32Array, false), "IEEE 754 binary32, big endian, Typed Array"), o2.registerDecoder(82, (e2) => c3(e2, Float64Array, false), "IEEE 754 binary64, big endian, Typed Array"), o2.registerDecoder(85, (e2) => c3(e2, Float32Array, true), "IEEE 754 binary32, little endian, Typed Array"), ue(Float32Array, (e2, r3, n2) => l3(r3, 85, 81, e2, n2)), o2.registerDecoder(86, (e2) => c3(e2, Float64Array, true), "IEEE 754 binary64, big endian, Typed Array"), ue(Float64Array, (e2, r3, n2) => l3(r3, 86, 82, e2, n2)), o2.registerDecoder(T.SET, (e2, r3) => {
        if (_2(e2.contents), r3.sortKeys) {
          const n2 = y4.decodeToEncodeOpts(r3);
          let i2 = null;
          for (const a4 of e2.contents) {
            const d6 = [a4, void 0, v2(a4, n2)];
            if (i2 && r3.sortKeys(i2, d6) >= 0) throw new Error(`Set items out of order in tag #${T.SET}`);
            i2 = d6;
          }
        }
        return new Set(e2.contents);
      }, "Set"), ue(Set, (e2, r3, n2) => {
        let i2 = [...e2];
        if (n2.sortKeys) {
          const a4 = i2.map((d6) => [d6, void 0, v2(d6, n2)]);
          a4.sort(n2.sortKeys), i2 = a4.map(([d6]) => d6);
        }
        return [T.SET, i2];
      }), o2.registerDecoder(T.JSON, (e2) => (f6(e2.contents), JSON.parse(e2.contents)), "JSON-encoded");
      C.comment = (e2) => {
        E2(e2.contents);
        const r3 = new Wtf8Decoder();
        return `(WTF8 string): ${JSON.stringify(r3.decode(e2.contents))}`;
      }, o2.registerDecoder(T.WTF8, C), o2.registerDecoder(T.SELF_DESCRIBED, (e2) => e2.contents, "Self-Described"), o2.registerDecoder(T.INVALID_16, () => {
        throw new Error(`Tag always invalid: ${T.INVALID_16}`);
      }, "Invalid"), o2.registerDecoder(T.INVALID_32, () => {
        throw new Error(`Tag always invalid: ${T.INVALID_32}`);
      }, "Invalid"), o2.registerDecoder(T.INVALID_64, () => {
        throw new Error(`Tag always invalid: ${T.INVALID_64}`);
      }, "Invalid"), o2.registerDecoder(T.SYMBOL, (e2) => {
        let r3 = e2.contents;
        if (Array.isArray(e2.contents)) {
          if (e2.contents.length !== 1) throw new Error(`Expected Array of size 1: ${e2.contents}`);
          [r3] = e2.contents;
        }
        if (f6(r3), !r3.length) throw new Error(`Expected non-empty string: ${e2.contents}`);
        return Symbol.for(r3);
      }, "Symbol");
      ue(ArrayBuffer, h3), ue(DataView, h3), typeof SharedArrayBuffer < "u" && ue(SharedArrayBuffer, h3);
      ue(Boolean, p4), ue(Number, p4), ue(String, p4), ue(BigInt, p4);
    }
  });

  // node_modules/cbor2/lib/version.js
  var init_version = __esm({
    "node_modules/cbor2/lib/version.js"() {
    }
  });

  // node_modules/cbor2/lib/decoder.js
  function c4(i2) {
    const e2 = { ...y4.defaultDecodeOptions };
    if (i2.dcbor ? Object.assign(e2, y4.dcborDecodeOptions) : i2.cde && Object.assign(e2, y4.cdeDecodeOptions), Object.assign(e2, i2), Object.hasOwn(e2, "rejectLongNumbers")) throw new TypeError("rejectLongNumbers has changed to requirePreferred");
    return e2.boxed && (e2.saveOriginal = true), e2;
  }
  function l4(i2, e2 = {}) {
    const n2 = c4(e2), t4 = new y3(i2, n2), r3 = new d4();
    for (const o5 of t4) r3.step(o5, n2, t4);
    return r3.ret;
  }
  var d4;
  var init_decoder = __esm({
    "node_modules/cbor2/lib/decoder.js"() {
      init_decodeStream2();
      init_container();
      init_constants();
      d4 = class {
        parent = void 0;
        ret = void 0;
        step(e2, n2, t4) {
          if (this.ret = y4.create(e2, this.parent, n2, t4), e2[2] === A.BREAK) if (this.parent?.isStreaming) this.parent.left = 0;
          else throw new Error("Unexpected BREAK");
          else this.parent && this.parent.push(this.ret, t4, e2[3]);
          for (this.ret instanceof y4 && (this.parent = this.ret); this.parent?.done; ) {
            this.ret = this.parent.convert(t4);
            const r3 = this.parent.parent;
            r3?.replaceLast(this.ret, this.parent, t4), this.parent = r3;
          }
        }
      };
    }
  });

  // node_modules/cbor2/lib/index.js
  var r, n, d5;
  var init_lib2 = __esm({
    "node_modules/cbor2/lib/index.js"() {
      init_types();
      init_version();
      init_container();
      init_options();
      init_decoder();
      init_diagnostic();
      init_comment();
      init_encoder();
      init_simple();
      init_tag();
      init_writer();
      init_box();
      init_typeEncoderMap();
      init_float();
      ({ cdeDecodeOptions: r, dcborDecodeOptions: n, defaultDecodeOptions: d5 } = y4);
    }
  });

  // node_modules/fflate/esm/browser.js
  function unzlibSync(data, opts) {
    return inflt(data.subarray(zls(data, opts && opts.dictionary), -4), { i: 2 }, opts && opts.out, opts && opts.dictionary);
  }
  var u8, u16, i32, fleb, fdeb, clim, freb, _a, fl, revfl, _b, fd, revfd, rev, x5, i2, hMap, flt, i2, i2, i2, i2, fdt, i2, flrm, fdrm, max, bits, bits16, shft, slc, ec, err, inflt, et, zls, Inflate, Unzlib, td, tds;
  var init_browser = __esm({
    "node_modules/fflate/esm/browser.js"() {
      u8 = Uint8Array;
      u16 = Uint16Array;
      i32 = Int32Array;
      fleb = new u8([
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1,
        1,
        1,
        1,
        2,
        2,
        2,
        2,
        3,
        3,
        3,
        3,
        4,
        4,
        4,
        4,
        5,
        5,
        5,
        5,
        0,
        /* unused */
        0,
        0,
        /* impossible */
        0
      ]);
      fdeb = new u8([
        0,
        0,
        0,
        0,
        1,
        1,
        2,
        2,
        3,
        3,
        4,
        4,
        5,
        5,
        6,
        6,
        7,
        7,
        8,
        8,
        9,
        9,
        10,
        10,
        11,
        11,
        12,
        12,
        13,
        13,
        /* unused */
        0,
        0
      ]);
      clim = new u8([16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15]);
      freb = function(eb, start) {
        var b5 = new u16(31);
        for (var i2 = 0; i2 < 31; ++i2) {
          b5[i2] = start += 1 << eb[i2 - 1];
        }
        var r3 = new i32(b5[30]);
        for (var i2 = 1; i2 < 30; ++i2) {
          for (var j2 = b5[i2]; j2 < b5[i2 + 1]; ++j2) {
            r3[j2] = j2 - b5[i2] << 5 | i2;
          }
        }
        return { b: b5, r: r3 };
      };
      _a = freb(fleb, 2);
      fl = _a.b;
      revfl = _a.r;
      fl[28] = 258, revfl[258] = 28;
      _b = freb(fdeb, 0);
      fd = _b.b;
      revfd = _b.r;
      rev = new u16(32768);
      for (i2 = 0; i2 < 32768; ++i2) {
        x5 = (i2 & 43690) >> 1 | (i2 & 21845) << 1;
        x5 = (x5 & 52428) >> 2 | (x5 & 13107) << 2;
        x5 = (x5 & 61680) >> 4 | (x5 & 3855) << 4;
        rev[i2] = ((x5 & 65280) >> 8 | (x5 & 255) << 8) >> 1;
      }
      hMap = (function(cd, mb, r3) {
        var s3 = cd.length;
        var i2 = 0;
        var l6 = new u16(mb);
        for (; i2 < s3; ++i2) {
          if (cd[i2])
            ++l6[cd[i2] - 1];
        }
        var le = new u16(mb);
        for (i2 = 1; i2 < mb; ++i2) {
          le[i2] = le[i2 - 1] + l6[i2 - 1] << 1;
        }
        var co;
        if (r3) {
          co = new u16(1 << mb);
          var rvb = 15 - mb;
          for (i2 = 0; i2 < s3; ++i2) {
            if (cd[i2]) {
              var sv = i2 << 4 | cd[i2];
              var r_1 = mb - cd[i2];
              var v3 = le[cd[i2] - 1]++ << r_1;
              for (var m4 = v3 | (1 << r_1) - 1; v3 <= m4; ++v3) {
                co[rev[v3] >> rvb] = sv;
              }
            }
          }
        } else {
          co = new u16(s3);
          for (i2 = 0; i2 < s3; ++i2) {
            if (cd[i2]) {
              co[i2] = rev[le[cd[i2] - 1]++] >> 15 - cd[i2];
            }
          }
        }
        return co;
      });
      flt = new u8(288);
      for (i2 = 0; i2 < 144; ++i2)
        flt[i2] = 8;
      for (i2 = 144; i2 < 256; ++i2)
        flt[i2] = 9;
      for (i2 = 256; i2 < 280; ++i2)
        flt[i2] = 7;
      for (i2 = 280; i2 < 288; ++i2)
        flt[i2] = 8;
      fdt = new u8(32);
      for (i2 = 0; i2 < 32; ++i2)
        fdt[i2] = 5;
      flrm = /* @__PURE__ */ hMap(flt, 9, 1);
      fdrm = /* @__PURE__ */ hMap(fdt, 5, 1);
      max = function(a4) {
        var m4 = a4[0];
        for (var i2 = 1; i2 < a4.length; ++i2) {
          if (a4[i2] > m4)
            m4 = a4[i2];
        }
        return m4;
      };
      bits = function(d6, p5, m4) {
        var o5 = p5 / 8 | 0;
        return (d6[o5] | d6[o5 + 1] << 8) >> (p5 & 7) & m4;
      };
      bits16 = function(d6, p5) {
        var o5 = p5 / 8 | 0;
        return (d6[o5] | d6[o5 + 1] << 8 | d6[o5 + 2] << 16) >> (p5 & 7);
      };
      shft = function(p5) {
        return (p5 + 7) / 8 | 0;
      };
      slc = function(v3, s3, e2) {
        if (s3 == null || s3 < 0)
          s3 = 0;
        if (e2 == null || e2 > v3.length)
          e2 = v3.length;
        return new u8(v3.subarray(s3, e2));
      };
      ec = [
        "unexpected EOF",
        "invalid block type",
        "invalid length/literal",
        "invalid distance",
        "stream finished",
        "no stream handler",
        ,
        "no callback",
        "invalid UTF-8 data",
        "extra field too long",
        "date not in range 1980-2099",
        "filename too long",
        "stream finishing",
        "invalid zip data"
        // determined by unknown compression method
      ];
      err = function(ind, msg, nt) {
        var e2 = new Error(msg || ec[ind]);
        e2.code = ind;
        if (Error.captureStackTrace)
          Error.captureStackTrace(e2, err);
        if (!nt)
          throw e2;
        return e2;
      };
      inflt = function(dat, st2, buf, dict) {
        var sl = dat.length, dl = dict ? dict.length : 0;
        if (!sl || st2.f && !st2.l)
          return buf || new u8(0);
        var noBuf = !buf;
        var resize = noBuf || st2.i != 2;
        var noSt = st2.i;
        if (noBuf)
          buf = new u8(sl * 3);
        var cbuf = function(l7) {
          var bl = buf.length;
          if (l7 > bl) {
            var nbuf = new u8(Math.max(bl * 2, l7));
            nbuf.set(buf);
            buf = nbuf;
          }
        };
        var final = st2.f || 0, pos = st2.p || 0, bt = st2.b || 0, lm = st2.l, dm = st2.d, lbt = st2.m, dbt = st2.n;
        var tbts = sl * 8;
        do {
          if (!lm) {
            final = bits(dat, pos, 1);
            var type = bits(dat, pos + 1, 3);
            pos += 3;
            if (!type) {
              var s3 = shft(pos) + 4, l6 = dat[s3 - 4] | dat[s3 - 3] << 8, t4 = s3 + l6;
              if (t4 > sl) {
                if (noSt)
                  err(0);
                break;
              }
              if (resize)
                cbuf(bt + l6);
              buf.set(dat.subarray(s3, t4), bt);
              st2.b = bt += l6, st2.p = pos = t4 * 8, st2.f = final;
              continue;
            } else if (type == 1)
              lm = flrm, dm = fdrm, lbt = 9, dbt = 5;
            else if (type == 2) {
              var hLit = bits(dat, pos, 31) + 257, hcLen = bits(dat, pos + 10, 15) + 4;
              var tl = hLit + bits(dat, pos + 5, 31) + 1;
              pos += 14;
              var ldt = new u8(tl);
              var clt = new u8(19);
              for (var i2 = 0; i2 < hcLen; ++i2) {
                clt[clim[i2]] = bits(dat, pos + i2 * 3, 7);
              }
              pos += hcLen * 3;
              var clb = max(clt), clbmsk = (1 << clb) - 1;
              var clm = hMap(clt, clb, 1);
              for (var i2 = 0; i2 < tl; ) {
                var r3 = clm[bits(dat, pos, clbmsk)];
                pos += r3 & 15;
                var s3 = r3 >> 4;
                if (s3 < 16) {
                  ldt[i2++] = s3;
                } else {
                  var c5 = 0, n2 = 0;
                  if (s3 == 16)
                    n2 = 3 + bits(dat, pos, 3), pos += 2, c5 = ldt[i2 - 1];
                  else if (s3 == 17)
                    n2 = 3 + bits(dat, pos, 7), pos += 3;
                  else if (s3 == 18)
                    n2 = 11 + bits(dat, pos, 127), pos += 7;
                  while (n2--)
                    ldt[i2++] = c5;
                }
              }
              var lt = ldt.subarray(0, hLit), dt2 = ldt.subarray(hLit);
              lbt = max(lt);
              dbt = max(dt2);
              lm = hMap(lt, lbt, 1);
              dm = hMap(dt2, dbt, 1);
            } else
              err(1);
            if (pos > tbts) {
              if (noSt)
                err(0);
              break;
            }
          }
          if (resize)
            cbuf(bt + 131072);
          var lms = (1 << lbt) - 1, dms = (1 << dbt) - 1;
          var lpos = pos;
          for (; ; lpos = pos) {
            var c5 = lm[bits16(dat, pos) & lms], sym = c5 >> 4;
            pos += c5 & 15;
            if (pos > tbts) {
              if (noSt)
                err(0);
              break;
            }
            if (!c5)
              err(2);
            if (sym < 256)
              buf[bt++] = sym;
            else if (sym == 256) {
              lpos = pos, lm = null;
              break;
            } else {
              var add = sym - 254;
              if (sym > 264) {
                var i2 = sym - 257, b5 = fleb[i2];
                add = bits(dat, pos, (1 << b5) - 1) + fl[i2];
                pos += b5;
              }
              var d6 = dm[bits16(dat, pos) & dms], dsym = d6 >> 4;
              if (!d6)
                err(3);
              pos += d6 & 15;
              var dt2 = fd[dsym];
              if (dsym > 3) {
                var b5 = fdeb[dsym];
                dt2 += bits16(dat, pos) & (1 << b5) - 1, pos += b5;
              }
              if (pos > tbts) {
                if (noSt)
                  err(0);
                break;
              }
              if (resize)
                cbuf(bt + 131072);
              var end = bt + add;
              if (bt < dt2) {
                var shift = dl - dt2, dend = Math.min(dt2, end);
                if (shift + bt < 0)
                  err(3);
                for (; bt < dend; ++bt)
                  buf[bt] = dict[shift + bt];
              }
              for (; bt < end; ++bt)
                buf[bt] = buf[bt - dt2];
            }
          }
          st2.l = lm, st2.p = lpos, st2.b = bt, st2.f = final;
          if (lm)
            final = 1, st2.m = lbt, st2.d = dm, st2.n = dbt;
        } while (!final);
        return bt != buf.length && noBuf ? slc(buf, 0, bt) : buf.subarray(0, bt);
      };
      et = /* @__PURE__ */ new u8(0);
      zls = function(d6, dict) {
        if ((d6[0] & 15) != 8 || d6[0] >> 4 > 7 || (d6[0] << 8 | d6[1]) % 31)
          err(6, "invalid zlib data");
        if ((d6[1] >> 5 & 1) == +!dict)
          err(6, "invalid zlib data: " + (d6[1] & 32 ? "need" : "unexpected") + " dictionary");
        return (d6[1] >> 3 & 4) + 2;
      };
      Inflate = /* @__PURE__ */ (function() {
        function Inflate2(opts, cb) {
          if (typeof opts == "function")
            cb = opts, opts = {};
          this.ondata = cb;
          var dict = opts && opts.dictionary && opts.dictionary.subarray(-32768);
          this.s = { i: 0, b: dict ? dict.length : 0 };
          this.o = new u8(32768);
          this.p = new u8(0);
          if (dict)
            this.o.set(dict);
        }
        Inflate2.prototype.e = function(c5) {
          if (!this.ondata)
            err(5);
          if (this.d)
            err(4);
          if (!this.p.length)
            this.p = c5;
          else if (c5.length) {
            var n2 = new u8(this.p.length + c5.length);
            n2.set(this.p), n2.set(c5, this.p.length), this.p = n2;
          }
        };
        Inflate2.prototype.c = function(final) {
          this.s.i = +(this.d = final || false);
          var bts = this.s.b;
          var dt2 = inflt(this.p, this.s, this.o);
          this.ondata(slc(dt2, bts, this.s.b), this.d);
          this.o = slc(dt2, this.s.b - 32768), this.s.b = this.o.length;
          this.p = slc(this.p, this.s.p / 8 | 0), this.s.p &= 7;
        };
        Inflate2.prototype.push = function(chunk, final) {
          this.e(chunk), this.c(final);
        };
        return Inflate2;
      })();
      Unzlib = /* @__PURE__ */ (function() {
        function Unzlib2(opts, cb) {
          Inflate.call(this, opts, cb);
          this.v = opts && opts.dictionary ? 2 : 1;
        }
        Unzlib2.prototype.push = function(chunk, final) {
          Inflate.prototype.e.call(this, chunk);
          if (this.v) {
            if (this.p.length < 6 && !final)
              return;
            this.p = this.p.subarray(zls(this.p, this.v - 1)), this.v = 0;
          }
          if (final) {
            if (this.p.length < 4)
              err(6, "invalid zlib data");
            this.p = this.p.subarray(0, -4);
          }
          Inflate.prototype.c.call(this, final);
        };
        return Unzlib2;
      })();
      td = typeof TextDecoder != "undefined" && /* @__PURE__ */ new TextDecoder();
      tds = 0;
      try {
        td.decode(et, { stream: true });
        tds = 1;
      } catch (e2) {
      }
    }
  });

  // node_modules/iobuffer/lib/text.js
  function decode(bytes, encoding = "utf8") {
    const decoder = new TextDecoder(encoding);
    return decoder.decode(bytes);
  }
  function encode(str) {
    return encoder.encode(str);
  }
  var encoder;
  var init_text = __esm({
    "node_modules/iobuffer/lib/text.js"() {
      encoder = new TextEncoder();
    }
  });

  // node_modules/iobuffer/lib/iobuffer.js
  var defaultByteLength, hostBigEndian, typedArrays, IOBuffer;
  var init_iobuffer = __esm({
    "node_modules/iobuffer/lib/iobuffer.js"() {
      init_text();
      defaultByteLength = 1024 * 8;
      hostBigEndian = (() => {
        const array = new Uint8Array(4);
        const view = new Uint32Array(array.buffer);
        return !((view[0] = 1) & array[0]);
      })();
      typedArrays = {
        int8: globalThis.Int8Array,
        uint8: globalThis.Uint8Array,
        int16: globalThis.Int16Array,
        uint16: globalThis.Uint16Array,
        int32: globalThis.Int32Array,
        uint32: globalThis.Uint32Array,
        uint64: globalThis.BigUint64Array,
        int64: globalThis.BigInt64Array,
        float32: globalThis.Float32Array,
        float64: globalThis.Float64Array
      };
      IOBuffer = class _IOBuffer {
        /**
         * Reference to the internal ArrayBuffer object.
         */
        buffer;
        /**
         * Byte length of the internal ArrayBuffer.
         */
        byteLength;
        /**
         * Byte offset of the internal ArrayBuffer.
         */
        byteOffset;
        /**
         * Byte length of the internal ArrayBuffer.
         */
        length;
        /**
         * The current offset of the buffer's pointer.
         */
        offset;
        lastWrittenByte;
        littleEndian;
        _data;
        _mark;
        _marks;
        /**
         * Create a new IOBuffer.
         * @param data - The data to construct the IOBuffer with.
         * If data is a number, it will be the new buffer's length<br>
         * If data is `undefined`, the buffer will be initialized with a default length of 8Kb<br>
         * If data is an ArrayBuffer, SharedArrayBuffer, an ArrayBufferView (Typed Array), an IOBuffer instance,
         * or a Node.js Buffer, a view will be created over the underlying ArrayBuffer.
         * @param options - An object for the options.
         * @returns A new IOBuffer instance.
         */
        constructor(data = defaultByteLength, options = {}) {
          let dataIsGiven = false;
          if (typeof data === "number") {
            data = new ArrayBuffer(data);
          } else {
            dataIsGiven = true;
            this.lastWrittenByte = data.byteLength;
          }
          const offset = options.offset ? options.offset >>> 0 : 0;
          const byteLength = data.byteLength - offset;
          let dvOffset = offset;
          if (ArrayBuffer.isView(data) || data instanceof _IOBuffer) {
            if (data.byteLength !== data.buffer.byteLength) {
              dvOffset = data.byteOffset + offset;
            }
            data = data.buffer;
          }
          if (dataIsGiven) {
            this.lastWrittenByte = byteLength;
          } else {
            this.lastWrittenByte = 0;
          }
          this.buffer = data;
          this.length = byteLength;
          this.byteLength = byteLength;
          this.byteOffset = dvOffset;
          this.offset = 0;
          this.littleEndian = true;
          this._data = new DataView(this.buffer, dvOffset, byteLength);
          this._mark = 0;
          this._marks = [];
        }
        /**
         * Checks if the memory allocated to the buffer is sufficient to store more
         * bytes after the offset.
         * @param byteLength - The needed memory in bytes.
         * @returns `true` if there is sufficient space and `false` otherwise.
         */
        available(byteLength = 1) {
          return this.offset + byteLength <= this.length;
        }
        /**
         * Check if little-endian mode is used for reading and writing multi-byte
         * values.
         * @returns `true` if little-endian mode is used, `false` otherwise.
         */
        isLittleEndian() {
          return this.littleEndian;
        }
        /**
         * Set little-endian mode for reading and writing multi-byte values.
         * @returns This.
         */
        setLittleEndian() {
          this.littleEndian = true;
          return this;
        }
        /**
         * Check if big-endian mode is used for reading and writing multi-byte values.
         * @returns `true` if big-endian mode is used, `false` otherwise.
         */
        isBigEndian() {
          return !this.littleEndian;
        }
        /**
         * Switches to big-endian mode for reading and writing multi-byte values.
         * @returns This.
         */
        setBigEndian() {
          this.littleEndian = false;
          return this;
        }
        /**
         * Move the pointer n bytes forward.
         * @param n - Number of bytes to skip.
         * @returns This.
         */
        skip(n2 = 1) {
          this.offset += n2;
          return this;
        }
        /**
         * Move the pointer n bytes backward.
         * @param n - Number of bytes to move back.
         * @returns This.
         */
        back(n2 = 1) {
          this.offset -= n2;
          return this;
        }
        /**
         * Move the pointer to the given offset.
         * @param offset - The offset to move to.
         * @returns This.
         */
        seek(offset) {
          this.offset = offset;
          return this;
        }
        /**
         * Store the current pointer offset.
         * @see {@link IOBuffer#reset}
         * @returns This.
         */
        mark() {
          this._mark = this.offset;
          return this;
        }
        /**
         * Move the pointer back to the last pointer offset set by mark.
         * @see {@link IOBuffer#mark}
         * @returns This.
         */
        reset() {
          this.offset = this._mark;
          return this;
        }
        /**
         * Push the current pointer offset to the mark stack.
         * @see {@link IOBuffer#popMark}
         * @returns This.
         */
        pushMark() {
          this._marks.push(this.offset);
          return this;
        }
        /**
         * Pop the last pointer offset from the mark stack, and set the current
         * pointer offset to the popped value.
         * @see {@link IOBuffer#pushMark}
         * @returns This.
         */
        popMark() {
          const offset = this._marks.pop();
          if (offset === void 0) {
            throw new Error("Mark stack empty");
          }
          this.seek(offset);
          return this;
        }
        /**
         * Move the pointer offset back to 0.
         * @returns This.
         */
        rewind() {
          this.offset = 0;
          return this;
        }
        /**
         * Make sure the buffer has sufficient memory to write a given byteLength at
         * the current pointer offset.
         * If the buffer's memory is insufficient, this method will create a new
         * buffer (a copy) with a length that is twice (byteLength + current offset).
         * @param byteLength - The needed memory in bytes.
         * @returns This.
         */
        ensureAvailable(byteLength = 1) {
          if (!this.available(byteLength)) {
            const lengthNeeded = this.offset + byteLength;
            const newLength = lengthNeeded * 2;
            const newArray = new Uint8Array(newLength);
            newArray.set(new Uint8Array(this.buffer));
            this.buffer = newArray.buffer;
            this.length = newLength;
            this.byteLength = newLength;
            this._data = new DataView(this.buffer);
          }
          return this;
        }
        /**
         * Read a byte and return false if the byte's value is 0, or true otherwise.
         * Moves pointer forward by one byte.
         * @returns The read boolean.
         */
        readBoolean() {
          return this.readUint8() !== 0;
        }
        /**
         * Read a signed 8-bit integer and move pointer forward by 1 byte.
         * @returns The read byte.
         */
        readInt8() {
          return this._data.getInt8(this.offset++);
        }
        /**
         * Read an unsigned 8-bit integer and move pointer forward by 1 byte.
         * @returns The read byte.
         */
        readUint8() {
          return this._data.getUint8(this.offset++);
        }
        /**
         * Alias for {@link IOBuffer#readUint8}.
         * @returns The read byte.
         */
        readByte() {
          return this.readUint8();
        }
        /**
         * Read `n` bytes and move pointer forward by `n` bytes.
         * @param n - Number of bytes to read.
         * @returns The read bytes.
         */
        readBytes(n2 = 1) {
          return this.readArray(n2, "uint8");
        }
        /**
         * Creates an array of corresponding to the type `type` and size `size`.
         * For example, type `uint8` will create a `Uint8Array`.
         * @param size - size of the resulting array
         * @param type - number type of elements to read
         * @returns The read array.
         */
        readArray(size, type) {
          const bytes = typedArrays[type].BYTES_PER_ELEMENT * size;
          const offset = this.byteOffset + this.offset;
          const slice = this.buffer.slice(offset, offset + bytes);
          if (this.littleEndian === hostBigEndian && type !== "uint8" && type !== "int8") {
            const slice2 = new Uint8Array(this.buffer.slice(offset, offset + bytes));
            slice2.reverse();
            const returnArray2 = new typedArrays[type](slice2.buffer);
            this.offset += bytes;
            returnArray2.reverse();
            return returnArray2;
          }
          const returnArray = new typedArrays[type](slice);
          this.offset += bytes;
          return returnArray;
        }
        /**
         * Read a 16-bit signed integer and move pointer forward by 2 bytes.
         * @returns The read value.
         */
        readInt16() {
          const value = this._data.getInt16(this.offset, this.littleEndian);
          this.offset += 2;
          return value;
        }
        /**
         * Read a 16-bit unsigned integer and move pointer forward by 2 bytes.
         * @returns The read value.
         */
        readUint16() {
          const value = this._data.getUint16(this.offset, this.littleEndian);
          this.offset += 2;
          return value;
        }
        /**
         * Read a 32-bit signed integer and move pointer forward by 4 bytes.
         * @returns The read value.
         */
        readInt32() {
          const value = this._data.getInt32(this.offset, this.littleEndian);
          this.offset += 4;
          return value;
        }
        /**
         * Read a 32-bit unsigned integer and move pointer forward by 4 bytes.
         * @returns The read value.
         */
        readUint32() {
          const value = this._data.getUint32(this.offset, this.littleEndian);
          this.offset += 4;
          return value;
        }
        /**
         * Read a 32-bit floating number and move pointer forward by 4 bytes.
         * @returns The read value.
         */
        readFloat32() {
          const value = this._data.getFloat32(this.offset, this.littleEndian);
          this.offset += 4;
          return value;
        }
        /**
         * Read a 64-bit floating number and move pointer forward by 8 bytes.
         * @returns The read value.
         */
        readFloat64() {
          const value = this._data.getFloat64(this.offset, this.littleEndian);
          this.offset += 8;
          return value;
        }
        /**
         * Read a 64-bit signed integer number and move pointer forward by 8 bytes.
         * @returns The read value.
         */
        readBigInt64() {
          const value = this._data.getBigInt64(this.offset, this.littleEndian);
          this.offset += 8;
          return value;
        }
        /**
         * Read a 64-bit unsigned integer number and move pointer forward by 8 bytes.
         * @returns The read value.
         */
        readBigUint64() {
          const value = this._data.getBigUint64(this.offset, this.littleEndian);
          this.offset += 8;
          return value;
        }
        /**
         * Read a 1-byte ASCII character and move pointer forward by 1 byte.
         * @returns The read character.
         */
        readChar() {
          return String.fromCharCode(this.readInt8());
        }
        /**
         * Read `n` 1-byte ASCII characters and move pointer forward by `n` bytes.
         * @param n - Number of characters to read.
         * @returns The read characters.
         */
        readChars(n2 = 1) {
          let result = "";
          for (let i2 = 0; i2 < n2; i2++) {
            result += this.readChar();
          }
          return result;
        }
        /**
         * Read the next `n` bytes, return a UTF-8 decoded string and move pointer
         * forward by `n` bytes.
         * @param n - Number of bytes to read.
         * @returns The decoded string.
         */
        readUtf8(n2 = 1) {
          return decode(this.readBytes(n2));
        }
        /**
         * Read the next `n` bytes, return a string decoded with `encoding` and move pointer
         * forward by `n` bytes.
         * If no encoding is passed, the function is equivalent to @see {@link IOBuffer#readUtf8}
         * @param n - Number of bytes to read.
         * @param encoding - The encoding to use. Default is 'utf8'.
         * @returns The decoded string.
         */
        decodeText(n2 = 1, encoding = "utf8") {
          return decode(this.readBytes(n2), encoding);
        }
        /**
         * Write 0xff if the passed value is truthy, 0x00 otherwise and move pointer
         * forward by 1 byte.
         * @param value - The value to write.
         * @returns This.
         */
        writeBoolean(value) {
          this.writeUint8(value ? 255 : 0);
          return this;
        }
        /**
         * Write `value` as an 8-bit signed integer and move pointer forward by 1 byte.
         * @param value - The value to write.
         * @returns This.
         */
        writeInt8(value) {
          this.ensureAvailable(1);
          this._data.setInt8(this.offset++, value);
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as an 8-bit unsigned integer and move pointer forward by 1
         * byte.
         * @param value - The value to write.
         * @returns This.
         */
        writeUint8(value) {
          this.ensureAvailable(1);
          this._data.setUint8(this.offset++, value);
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * An alias for {@link IOBuffer#writeUint8}.
         * @param value - The value to write.
         * @returns This.
         */
        writeByte(value) {
          return this.writeUint8(value);
        }
        /**
         * Write all elements of `bytes` as uint8 values and move pointer forward by
         * `bytes.length` bytes.
         * @param bytes - The array of bytes to write.
         * @returns This.
         */
        writeBytes(bytes) {
          this.ensureAvailable(bytes.length);
          for (let i2 = 0; i2 < bytes.length; i2++) {
            this._data.setUint8(this.offset++, bytes[i2]);
          }
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 16-bit signed integer and move pointer forward by 2
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeInt16(value) {
          this.ensureAvailable(2);
          this._data.setInt16(this.offset, value, this.littleEndian);
          this.offset += 2;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 16-bit unsigned integer and move pointer forward by 2
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeUint16(value) {
          this.ensureAvailable(2);
          this._data.setUint16(this.offset, value, this.littleEndian);
          this.offset += 2;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 32-bit signed integer and move pointer forward by 4
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeInt32(value) {
          this.ensureAvailable(4);
          this._data.setInt32(this.offset, value, this.littleEndian);
          this.offset += 4;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 32-bit unsigned integer and move pointer forward by 4
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeUint32(value) {
          this.ensureAvailable(4);
          this._data.setUint32(this.offset, value, this.littleEndian);
          this.offset += 4;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 32-bit floating number and move pointer forward by 4
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeFloat32(value) {
          this.ensureAvailable(4);
          this._data.setFloat32(this.offset, value, this.littleEndian);
          this.offset += 4;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 64-bit floating number and move pointer forward by 8
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeFloat64(value) {
          this.ensureAvailable(8);
          this._data.setFloat64(this.offset, value, this.littleEndian);
          this.offset += 8;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 64-bit signed bigint and move pointer forward by 8
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeBigInt64(value) {
          this.ensureAvailable(8);
          this._data.setBigInt64(this.offset, value, this.littleEndian);
          this.offset += 8;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write `value` as a 64-bit unsigned bigint and move pointer forward by 8
         * bytes.
         * @param value - The value to write.
         * @returns This.
         */
        writeBigUint64(value) {
          this.ensureAvailable(8);
          this._data.setBigUint64(this.offset, value, this.littleEndian);
          this.offset += 8;
          this._updateLastWrittenByte();
          return this;
        }
        /**
         * Write the charCode of `str`'s first character as an 8-bit unsigned integer
         * and move pointer forward by 1 byte.
         * @param str - The character to write.
         * @returns This.
         */
        writeChar(str) {
          return this.writeUint8(str.charCodeAt(0));
        }
        /**
         * Write the charCodes of all `str`'s characters as 8-bit unsigned integers
         * and move pointer forward by `str.length` bytes.
         * @param str - The characters to write.
         * @returns This.
         */
        writeChars(str) {
          for (let i2 = 0; i2 < str.length; i2++) {
            this.writeUint8(str.charCodeAt(i2));
          }
          return this;
        }
        /**
         * UTF-8 encode and write `str` to the current pointer offset and move pointer
         * forward according to the encoded length.
         * @param str - The string to write.
         * @returns This.
         */
        writeUtf8(str) {
          return this.writeBytes(encode(str));
        }
        /**
         * Export a Uint8Array view of the internal buffer.
         * The view starts at the byte offset and its length
         * is calculated to stop at the last written byte or the original length.
         * @returns A new Uint8Array view.
         */
        toArray() {
          return new Uint8Array(this.buffer, this.byteOffset, this.lastWrittenByte);
        }
        /**
         *  Get the total number of bytes written so far, regardless of the current offset.
         * @returns - Total number of bytes.
         */
        getWrittenByteLength() {
          return this.lastWrittenByte - this.byteOffset;
        }
        /**
         * Update the last written byte offset
         * @private
         */
        _updateLastWrittenByte() {
          if (this.offset > this.lastWrittenByte) {
            this.lastWrittenByte = this.offset;
          }
        }
      };
    }
  });

  // node_modules/fast-png/lib/helpers/crc.js
  function updateCrc(currentCrc, data, length) {
    let c5 = currentCrc;
    for (let n2 = 0; n2 < length; n2++) {
      c5 = crcTable[(c5 ^ data[n2]) & 255] ^ c5 >>> 8;
    }
    return c5;
  }
  function crc(data, length) {
    return (updateCrc(initialCrc, data, length) ^ initialCrc) >>> 0;
  }
  function checkCrc(buffer2, crcLength, chunkName) {
    const expectedCrc = buffer2.readUint32();
    const actualCrc = crc(new Uint8Array(buffer2.buffer, buffer2.byteOffset + buffer2.offset - crcLength - 4, crcLength), crcLength);
    if (actualCrc !== expectedCrc) {
      throw new Error(`CRC mismatch for chunk ${chunkName}. Expected ${expectedCrc}, found ${actualCrc}`);
    }
  }
  var crcTable, initialCrc;
  var init_crc = __esm({
    "node_modules/fast-png/lib/helpers/crc.js"() {
      crcTable = [];
      for (let n2 = 0; n2 < 256; n2++) {
        let c5 = n2;
        for (let k4 = 0; k4 < 8; k4++) {
          if (c5 & 1) {
            c5 = 3988292384 ^ c5 >>> 1;
          } else {
            c5 = c5 >>> 1;
          }
        }
        crcTable[n2] = c5;
      }
      initialCrc = 4294967295;
    }
  });

  // node_modules/fast-png/lib/helpers/unfilter.js
  function unfilterNone(currentLine, newLine, bytesPerLine) {
    for (let i2 = 0; i2 < bytesPerLine; i2++) {
      newLine[i2] = currentLine[i2];
    }
  }
  function unfilterSub(currentLine, newLine, bytesPerLine, bytesPerPixel) {
    let i2 = 0;
    for (; i2 < bytesPerPixel; i2++) {
      newLine[i2] = currentLine[i2];
    }
    for (; i2 < bytesPerLine; i2++) {
      newLine[i2] = currentLine[i2] + newLine[i2 - bytesPerPixel] & 255;
    }
  }
  function unfilterUp(currentLine, newLine, prevLine, bytesPerLine) {
    let i2 = 0;
    if (prevLine.length === 0) {
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2];
      }
    } else {
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2] + prevLine[i2] & 255;
      }
    }
  }
  function unfilterAverage(currentLine, newLine, prevLine, bytesPerLine, bytesPerPixel) {
    let i2 = 0;
    if (prevLine.length === 0) {
      for (; i2 < bytesPerPixel; i2++) {
        newLine[i2] = currentLine[i2];
      }
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2] + (newLine[i2 - bytesPerPixel] >> 1) & 255;
      }
    } else {
      for (; i2 < bytesPerPixel; i2++) {
        newLine[i2] = currentLine[i2] + (prevLine[i2] >> 1) & 255;
      }
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2] + (newLine[i2 - bytesPerPixel] + prevLine[i2] >> 1) & 255;
      }
    }
  }
  function unfilterPaeth(currentLine, newLine, prevLine, bytesPerLine, bytesPerPixel) {
    let i2 = 0;
    if (prevLine.length === 0) {
      for (; i2 < bytesPerPixel; i2++) {
        newLine[i2] = currentLine[i2];
      }
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2] + newLine[i2 - bytesPerPixel] & 255;
      }
    } else {
      for (; i2 < bytesPerPixel; i2++) {
        newLine[i2] = currentLine[i2] + prevLine[i2] & 255;
      }
      for (; i2 < bytesPerLine; i2++) {
        newLine[i2] = currentLine[i2] + paethPredictor(newLine[i2 - bytesPerPixel], prevLine[i2], prevLine[i2 - bytesPerPixel]) & 255;
      }
    }
  }
  function paethPredictor(a4, b5, c5) {
    const p5 = a4 + b5 - c5;
    const pa = Math.abs(p5 - a4);
    const pb = Math.abs(p5 - b5);
    const pc = Math.abs(p5 - c5);
    if (pa <= pb && pa <= pc)
      return a4;
    else if (pb <= pc)
      return b5;
    else
      return c5;
  }
  var init_unfilter = __esm({
    "node_modules/fast-png/lib/helpers/unfilter.js"() {
    }
  });

  // node_modules/fast-png/lib/helpers/apply_unfilter.js
  function applyUnfilter(filterType, currentLine, newLine, prevLine, passLineBytes, bytesPerPixel) {
    switch (filterType) {
      case 0:
        unfilterNone(currentLine, newLine, passLineBytes);
        break;
      case 1:
        unfilterSub(currentLine, newLine, passLineBytes, bytesPerPixel);
        break;
      case 2:
        unfilterUp(currentLine, newLine, prevLine, passLineBytes);
        break;
      case 3:
        unfilterAverage(currentLine, newLine, prevLine, passLineBytes, bytesPerPixel);
        break;
      case 4:
        unfilterPaeth(currentLine, newLine, prevLine, passLineBytes, bytesPerPixel);
        break;
      default:
        throw new Error(`Unsupported filter: ${filterType}`);
    }
  }
  var init_apply_unfilter = __esm({
    "node_modules/fast-png/lib/helpers/apply_unfilter.js"() {
      init_unfilter();
    }
  });

  // node_modules/fast-png/lib/helpers/decode_interlace_adam7.js
  function decodeInterlaceAdam7(params) {
    const { data, width, height, channels, depth } = params;
    const passes = [
      { x: 0, y: 0, xStep: 8, yStep: 8 },
      // Pass 1
      { x: 4, y: 0, xStep: 8, yStep: 8 },
      // Pass 2
      { x: 0, y: 4, xStep: 4, yStep: 8 },
      // Pass 3
      { x: 2, y: 0, xStep: 4, yStep: 4 },
      // Pass 4
      { x: 0, y: 2, xStep: 2, yStep: 4 },
      // Pass 5
      { x: 1, y: 0, xStep: 2, yStep: 2 },
      // Pass 6
      { x: 0, y: 1, xStep: 1, yStep: 2 }
      // Pass 7
    ];
    const bytesPerPixel = Math.ceil(depth / 8) * channels;
    const resultData = new Uint8Array(height * width * bytesPerPixel);
    let offset = 0;
    for (let passIndex = 0; passIndex < 7; passIndex++) {
      const pass = passes[passIndex];
      const passWidth = Math.ceil((width - pass.x) / pass.xStep);
      const passHeight = Math.ceil((height - pass.y) / pass.yStep);
      if (passWidth <= 0 || passHeight <= 0)
        continue;
      const passLineBytes = passWidth * bytesPerPixel;
      const prevLine = new Uint8Array(passLineBytes);
      for (let y7 = 0; y7 < passHeight; y7++) {
        const filterType = data[offset++];
        const currentLine = data.subarray(offset, offset + passLineBytes);
        offset += passLineBytes;
        const newLine = new Uint8Array(passLineBytes);
        applyUnfilter(filterType, currentLine, newLine, prevLine, passLineBytes, bytesPerPixel);
        prevLine.set(newLine);
        for (let x5 = 0; x5 < passWidth; x5++) {
          const outputX = pass.x + x5 * pass.xStep;
          const outputY = pass.y + y7 * pass.yStep;
          if (outputX >= width || outputY >= height)
            continue;
          for (let i2 = 0; i2 < bytesPerPixel; i2++) {
            resultData[(outputY * width + outputX) * bytesPerPixel + i2] = newLine[x5 * bytesPerPixel + i2];
          }
        }
      }
    }
    if (depth === 16) {
      const uint16Data = new Uint16Array(resultData.buffer);
      if (osIsLittleEndian) {
        for (let k4 = 0; k4 < uint16Data.length; k4++) {
          uint16Data[k4] = swap16(uint16Data[k4]);
        }
      }
      return uint16Data;
    } else {
      return resultData;
    }
  }
  function swap16(val) {
    return (val & 255) << 8 | val >> 8 & 255;
  }
  var uint16, uint8, osIsLittleEndian;
  var init_decode_interlace_adam7 = __esm({
    "node_modules/fast-png/lib/helpers/decode_interlace_adam7.js"() {
      init_apply_unfilter();
      uint16 = new Uint16Array([255]);
      uint8 = new Uint8Array(uint16.buffer);
      osIsLittleEndian = uint8[0] === 255;
    }
  });

  // node_modules/fast-png/lib/helpers/decode_interlace_null.js
  function decodeInterlaceNull(params) {
    const { data, width, height, channels, depth } = params;
    const bytesPerPixel = Math.ceil(depth / 8) * channels;
    const bytesPerLine = Math.ceil(depth / 8 * channels * width);
    const newData = new Uint8Array(height * bytesPerLine);
    let prevLine = empty;
    let offset = 0;
    let currentLine;
    let newLine;
    for (let i2 = 0; i2 < height; i2++) {
      currentLine = data.subarray(offset + 1, offset + 1 + bytesPerLine);
      newLine = newData.subarray(i2 * bytesPerLine, (i2 + 1) * bytesPerLine);
      switch (data[offset]) {
        case 0:
          unfilterNone(currentLine, newLine, bytesPerLine);
          break;
        case 1:
          unfilterSub(currentLine, newLine, bytesPerLine, bytesPerPixel);
          break;
        case 2:
          unfilterUp(currentLine, newLine, prevLine, bytesPerLine);
          break;
        case 3:
          unfilterAverage(currentLine, newLine, prevLine, bytesPerLine, bytesPerPixel);
          break;
        case 4:
          unfilterPaeth(currentLine, newLine, prevLine, bytesPerLine, bytesPerPixel);
          break;
        default:
          throw new Error(`Unsupported filter: ${data[offset]}`);
      }
      prevLine = newLine;
      offset += bytesPerLine + 1;
    }
    if (depth === 16) {
      const uint16Data = new Uint16Array(newData.buffer);
      if (osIsLittleEndian2) {
        for (let k4 = 0; k4 < uint16Data.length; k4++) {
          uint16Data[k4] = swap162(uint16Data[k4]);
        }
      }
      return uint16Data;
    } else {
      return newData;
    }
  }
  function swap162(val) {
    return (val & 255) << 8 | val >> 8 & 255;
  }
  var uint162, uint82, osIsLittleEndian2, empty;
  var init_decode_interlace_null = __esm({
    "node_modules/fast-png/lib/helpers/decode_interlace_null.js"() {
      init_unfilter();
      uint162 = new Uint16Array([255]);
      uint82 = new Uint8Array(uint162.buffer);
      osIsLittleEndian2 = uint82[0] === 255;
      empty = new Uint8Array(0);
    }
  });

  // node_modules/fast-png/lib/helpers/signature.js
  function checkSignature(buffer2) {
    if (!hasPngSignature(buffer2.readBytes(pngSignature.length))) {
      throw new Error("wrong PNG signature");
    }
  }
  function hasPngSignature(array) {
    if (array.length < pngSignature.length) {
      return false;
    }
    for (let i2 = 0; i2 < pngSignature.length; i2++) {
      if (array[i2] !== pngSignature[i2]) {
        return false;
      }
    }
    return true;
  }
  var pngSignature;
  var init_signature = __esm({
    "node_modules/fast-png/lib/helpers/signature.js"() {
      pngSignature = Uint8Array.of(137, 80, 78, 71, 13, 10, 26, 10);
    }
  });

  // node_modules/fast-png/lib/helpers/text.js
  function validateKeyword(keyword) {
    validateLatin1(keyword);
    if (keyword.length === 0 || keyword.length > 79) {
      throw new Error("keyword length must be between 1 and 79");
    }
  }
  function validateLatin1(text) {
    if (!latin1Regex.test(text)) {
      throw new Error("invalid latin1 text");
    }
  }
  function decodetEXt(text, buffer2, length) {
    const keyword = readKeyword(buffer2);
    text[keyword] = readLatin1(buffer2, length - keyword.length - 1);
  }
  function readKeyword(buffer2) {
    buffer2.mark();
    while (buffer2.readByte() !== NULL) {
    }
    const end = buffer2.offset;
    buffer2.reset();
    const keyword = latin1Decoder.decode(buffer2.readBytes(end - buffer2.offset - 1));
    buffer2.skip(1);
    validateKeyword(keyword);
    return keyword;
  }
  function readLatin1(buffer2, length) {
    return latin1Decoder.decode(buffer2.readBytes(length));
  }
  var textChunkName, NULL, latin1Decoder, latin1Regex;
  var init_text2 = __esm({
    "node_modules/fast-png/lib/helpers/text.js"() {
      init_crc();
      textChunkName = "tEXt";
      NULL = 0;
      latin1Decoder = new TextDecoder("latin1");
      latin1Regex = /^[\u0000-\u00FF]*$/;
    }
  });

  // node_modules/fast-png/lib/internal_types.js
  var ColorType, CompressionMethod, FilterMethod, InterlaceMethod, DisposeOpType, BlendOpType;
  var init_internal_types = __esm({
    "node_modules/fast-png/lib/internal_types.js"() {
      ColorType = {
        UNKNOWN: -1,
        GREYSCALE: 0,
        TRUECOLOUR: 2,
        INDEXED_COLOUR: 3,
        GREYSCALE_ALPHA: 4,
        TRUECOLOUR_ALPHA: 6
      };
      CompressionMethod = {
        UNKNOWN: -1,
        DEFLATE: 0
      };
      FilterMethod = {
        UNKNOWN: -1,
        ADAPTIVE: 0
      };
      InterlaceMethod = {
        UNKNOWN: -1,
        NO_INTERLACE: 0,
        ADAM7: 1
      };
      DisposeOpType = {
        NONE: 0,
        BACKGROUND: 1,
        PREVIOUS: 2
      };
      BlendOpType = {
        SOURCE: 0,
        OVER: 1
      };
    }
  });

  // node_modules/fast-png/lib/png_decoder.js
  function checkBitDepth(value) {
    if (value !== 1 && value !== 2 && value !== 4 && value !== 8 && value !== 16) {
      throw new Error(`invalid bit depth: ${value}`);
    }
    return value;
  }
  var PngDecoder;
  var init_png_decoder = __esm({
    "node_modules/fast-png/lib/png_decoder.js"() {
      init_browser();
      init_iobuffer();
      init_crc();
      init_decode_interlace_adam7();
      init_decode_interlace_null();
      init_signature();
      init_text2();
      init_internal_types();
      PngDecoder = class extends IOBuffer {
        _checkCrc;
        _inflator;
        _png;
        _apng;
        _end;
        _hasPalette;
        _palette;
        _hasTransparency;
        _transparency;
        _compressionMethod;
        _filterMethod;
        _interlaceMethod;
        _colorType;
        _isAnimated;
        _numberOfFrames;
        _numberOfPlays;
        _frames;
        _writingDataChunks;
        _chunks;
        _inflatorResult;
        constructor(data, options = {}) {
          super(data);
          const { checkCrc: checkCrc2 = false } = options;
          this._checkCrc = checkCrc2;
          this._inflator = new Unzlib((chunk, final) => {
            this._chunks.push(chunk);
            if (final) {
              const totalLength = this._chunks.reduce((sum, c5) => sum + c5.length, 0);
              this._inflatorResult = new Uint8Array(totalLength);
              let offset = 0;
              for (const chunk2 of this._chunks) {
                this._inflatorResult.set(chunk2, offset);
                offset += chunk2.length;
              }
              this._chunks = [];
            }
          });
          this._chunks = [];
          this._png = {
            width: -1,
            height: -1,
            channels: -1,
            data: new Uint8Array(0),
            depth: 1,
            text: {}
          };
          this._apng = {
            width: -1,
            height: -1,
            channels: -1,
            depth: 1,
            numberOfFrames: 1,
            numberOfPlays: 0,
            text: {},
            frames: []
          };
          this._end = false;
          this._hasPalette = false;
          this._palette = [];
          this._hasTransparency = false;
          this._transparency = new Uint16Array(0);
          this._compressionMethod = CompressionMethod.UNKNOWN;
          this._filterMethod = FilterMethod.UNKNOWN;
          this._interlaceMethod = InterlaceMethod.UNKNOWN;
          this._colorType = ColorType.UNKNOWN;
          this._isAnimated = false;
          this._numberOfFrames = 1;
          this._numberOfPlays = 0;
          this._frames = [];
          this._writingDataChunks = false;
          this._inflatorResult = new Uint8Array(0);
          this.setBigEndian();
        }
        decode() {
          checkSignature(this);
          while (!this._end) {
            const length = this.readUint32();
            const type = this.readChars(4);
            this.decodeChunk(length, type);
          }
          this._inflator.push(new Uint8Array(0), true);
          this.decodeImage();
          return this._png;
        }
        decodeApng() {
          checkSignature(this);
          while (!this._end) {
            const length = this.readUint32();
            const type = this.readChars(4);
            this.decodeApngChunk(length, type);
          }
          this.decodeApngImage();
          return this._apng;
        }
        // https://www.w3.org/TR/PNG/#5Chunk-layout
        decodeChunk(length, type) {
          const offset = this.offset;
          switch (type) {
            // 11.2 Critical chunks
            case "IHDR":
              this.decodeIHDR();
              break;
            case "PLTE":
              this.decodePLTE(length);
              break;
            case "IDAT":
              this.decodeIDAT(length);
              break;
            case "IEND":
              this._end = true;
              break;
            // 11.3 Ancillary chunks
            case "tRNS":
              this.decodetRNS(length);
              break;
            case "iCCP":
              this.decodeiCCP(length);
              break;
            case textChunkName:
              decodetEXt(this._png.text, this, length);
              break;
            case "pHYs":
              this.decodepHYs();
              break;
            default:
              this.skip(length);
              break;
          }
          if (this.offset - offset !== length) {
            throw new Error(`Length mismatch while decoding chunk ${type}`);
          }
          if (this._checkCrc) {
            checkCrc(this, length + 4, type);
          } else {
            this.skip(4);
          }
        }
        decodeApngChunk(length, type) {
          const offset = this.offset;
          if (type !== "fdAT" && type !== "IDAT" && this._writingDataChunks) {
            this.pushDataToFrame();
          }
          switch (type) {
            case "acTL":
              this.decodeACTL();
              break;
            case "fcTL":
              this.decodeFCTL();
              break;
            case "fdAT":
              this.decodeFDAT(length);
              break;
            default:
              this.decodeChunk(length, type);
              this.offset = offset + length;
              break;
          }
          if (this.offset - offset !== length) {
            throw new Error(`Length mismatch while decoding chunk ${type}`);
          }
          if (this._checkCrc) {
            checkCrc(this, length + 4, type);
          } else {
            this.skip(4);
          }
        }
        // https://www.w3.org/TR/PNG/#11IHDR
        decodeIHDR() {
          const image = this._png;
          image.width = this.readUint32();
          image.height = this.readUint32();
          image.depth = checkBitDepth(this.readUint8());
          const colorType = this.readUint8();
          this._colorType = colorType;
          let channels;
          switch (colorType) {
            case ColorType.GREYSCALE:
              channels = 1;
              break;
            case ColorType.TRUECOLOUR:
              channels = 3;
              break;
            case ColorType.INDEXED_COLOUR:
              channels = 1;
              break;
            case ColorType.GREYSCALE_ALPHA:
              channels = 2;
              break;
            case ColorType.TRUECOLOUR_ALPHA:
              channels = 4;
              break;
            // Kept for exhaustiveness.
            // eslint-disable-next-line unicorn/no-useless-switch-case
            case ColorType.UNKNOWN:
            default:
              throw new Error(`Unknown color type: ${colorType}`);
          }
          this._png.channels = channels;
          this._compressionMethod = this.readUint8();
          if (this._compressionMethod !== CompressionMethod.DEFLATE) {
            throw new Error(`Unsupported compression method: ${this._compressionMethod}`);
          }
          this._filterMethod = this.readUint8();
          this._interlaceMethod = this.readUint8();
        }
        decodeACTL() {
          this._numberOfFrames = this.readUint32();
          this._numberOfPlays = this.readUint32();
          this._isAnimated = true;
        }
        decodeFCTL() {
          const image = {
            sequenceNumber: this.readUint32(),
            width: this.readUint32(),
            height: this.readUint32(),
            xOffset: this.readUint32(),
            yOffset: this.readUint32(),
            delayNumber: this.readUint16(),
            delayDenominator: this.readUint16(),
            disposeOp: this.readUint8(),
            blendOp: this.readUint8(),
            data: new Uint8Array(0)
          };
          this._frames.push(image);
        }
        // https://www.w3.org/TR/PNG/#11PLTE
        decodePLTE(length) {
          if (length % 3 !== 0) {
            throw new RangeError(`PLTE field length must be a multiple of 3. Got ${length}`);
          }
          const l6 = length / 3;
          this._hasPalette = true;
          const palette = [];
          this._palette = palette;
          for (let i2 = 0; i2 < l6; i2++) {
            palette.push([this.readUint8(), this.readUint8(), this.readUint8()]);
          }
        }
        // https://www.w3.org/TR/PNG/#11IDAT
        decodeIDAT(length) {
          this._writingDataChunks = true;
          const dataLength = length;
          const dataOffset = this.offset + this.byteOffset;
          try {
            this._inflator.push(new Uint8Array(this.buffer, dataOffset, dataLength), false);
          } catch (error) {
            throw new Error("Error while decompressing the data:", { cause: error });
          }
          this.skip(length);
        }
        decodeFDAT(length) {
          this._writingDataChunks = true;
          let dataLength = length;
          let dataOffset = this.offset + this.byteOffset;
          dataOffset += 4;
          dataLength -= 4;
          try {
            this._inflator.push(new Uint8Array(this.buffer, dataOffset, dataLength), false);
          } catch (error) {
            throw new Error("Error while decompressing the data:", { cause: error });
          }
          this.skip(length);
        }
        // https://www.w3.org/TR/PNG/#11tRNS
        decodetRNS(length) {
          switch (this._colorType) {
            case ColorType.GREYSCALE:
            case ColorType.TRUECOLOUR: {
              if (length % 2 !== 0) {
                throw new RangeError(`tRNS chunk length must be a multiple of 2. Got ${length}`);
              }
              if (length / 2 > this._png.width * this._png.height) {
                throw new Error(`tRNS chunk contains more alpha values than there are pixels (${length / 2} vs ${this._png.width * this._png.height})`);
              }
              this._hasTransparency = true;
              this._transparency = new Uint16Array(length / 2);
              for (let i2 = 0; i2 < length / 2; i2++) {
                this._transparency[i2] = this.readUint16();
              }
              break;
            }
            case ColorType.INDEXED_COLOUR: {
              if (length > this._palette.length) {
                throw new Error(`tRNS chunk contains more alpha values than there are palette colors (${length} vs ${this._palette.length})`);
              }
              let i2 = 0;
              for (; i2 < length; i2++) {
                const alpha = this.readByte();
                this._palette[i2].push(alpha);
              }
              for (; i2 < this._palette.length; i2++) {
                this._palette[i2].push(255);
              }
              break;
            }
            // Kept for exhaustiveness.
            /* eslint-disable unicorn/no-useless-switch-case */
            case ColorType.UNKNOWN:
            case ColorType.GREYSCALE_ALPHA:
            case ColorType.TRUECOLOUR_ALPHA:
            default: {
              throw new Error(`tRNS chunk is not supported for color type ${this._colorType}`);
            }
          }
        }
        // https://www.w3.org/TR/PNG/#11iCCP
        decodeiCCP(length) {
          const name = readKeyword(this);
          const compressionMethod = this.readUint8();
          if (compressionMethod !== CompressionMethod.DEFLATE) {
            throw new Error(`Unsupported iCCP compression method: ${compressionMethod}`);
          }
          const compressedProfile = this.readBytes(length - name.length - 2);
          this._png.iccEmbeddedProfile = {
            name,
            profile: unzlibSync(compressedProfile)
          };
        }
        // https://www.w3.org/TR/PNG/#11pHYs
        decodepHYs() {
          const ppuX = this.readUint32();
          const ppuY = this.readUint32();
          const unitSpecifier = this.readByte();
          this._png.resolution = {
            x: ppuX,
            y: ppuY,
            unit: unitSpecifier
          };
        }
        decodeApngImage() {
          this._apng.width = this._png.width;
          this._apng.height = this._png.height;
          this._apng.channels = this._png.channels;
          this._apng.depth = this._png.depth;
          this._apng.numberOfFrames = this._numberOfFrames;
          this._apng.numberOfPlays = this._numberOfPlays;
          this._apng.text = this._png.text;
          this._apng.resolution = this._png.resolution;
          for (let i2 = 0; i2 < this._numberOfFrames; i2++) {
            const newFrame = {
              sequenceNumber: this._frames[i2].sequenceNumber,
              delayNumber: this._frames[i2].delayNumber,
              delayDenominator: this._frames[i2].delayDenominator,
              data: this._apng.depth === 8 ? new Uint8Array(this._apng.width * this._apng.height * this._apng.channels) : new Uint16Array(this._apng.width * this._apng.height * this._apng.channels)
            };
            const frame = this._frames.at(i2);
            if (frame) {
              frame.data = decodeInterlaceNull({
                data: frame.data,
                width: frame.width,
                height: frame.height,
                channels: this._apng.channels,
                depth: this._apng.depth
              });
              if (this._hasPalette) {
                this._apng.palette = this._palette;
              }
              if (this._hasTransparency) {
                this._apng.transparency = this._transparency;
              }
              if (i2 === 0 || frame.xOffset === 0 && frame.yOffset === 0 && frame.width === this._png.width && frame.height === this._png.height) {
                newFrame.data = frame.data;
              } else {
                const prevFrame = this._apng.frames.at(i2 - 1);
                this.disposeFrame(frame, prevFrame, newFrame);
                this.addFrameDataToCanvas(newFrame, frame);
              }
              this._apng.frames.push(newFrame);
            }
          }
          return this._apng;
        }
        disposeFrame(frame, prevFrame, imageFrame) {
          switch (frame.disposeOp) {
            case DisposeOpType.NONE:
              break;
            case DisposeOpType.BACKGROUND:
              for (let row = 0; row < this._png.height; row++) {
                for (let col = 0; col < this._png.width; col++) {
                  const index = (row * frame.width + col) * this._png.channels;
                  for (let channel = 0; channel < this._png.channels; channel++) {
                    imageFrame.data[index + channel] = 0;
                  }
                }
              }
              break;
            case DisposeOpType.PREVIOUS:
              imageFrame.data.set(prevFrame.data);
              break;
            default:
              throw new Error("Unknown disposeOp");
          }
        }
        addFrameDataToCanvas(imageFrame, frame) {
          const maxValue = 1 << this._png.depth;
          const calculatePixelIndices = (row, col) => {
            const index = ((row + frame.yOffset) * this._png.width + frame.xOffset + col) * this._png.channels;
            const frameIndex = (row * frame.width + col) * this._png.channels;
            return { index, frameIndex };
          };
          switch (frame.blendOp) {
            case BlendOpType.SOURCE:
              for (let row = 0; row < frame.height; row++) {
                for (let col = 0; col < frame.width; col++) {
                  const { index, frameIndex } = calculatePixelIndices(row, col);
                  for (let channel = 0; channel < this._png.channels; channel++) {
                    imageFrame.data[index + channel] = frame.data[frameIndex + channel];
                  }
                }
              }
              break;
            // https://www.w3.org/TR/png-3/#13Alpha-channel-processing
            case BlendOpType.OVER:
              for (let row = 0; row < frame.height; row++) {
                for (let col = 0; col < frame.width; col++) {
                  const { index, frameIndex } = calculatePixelIndices(row, col);
                  for (let channel = 0; channel < this._png.channels; channel++) {
                    const sourceAlpha = frame.data[frameIndex + this._png.channels - 1] / maxValue;
                    const foregroundValue = channel % (this._png.channels - 1) === 0 ? 1 : frame.data[frameIndex + channel];
                    const value = Math.floor(sourceAlpha * foregroundValue + (1 - sourceAlpha) * imageFrame.data[index + channel]);
                    imageFrame.data[index + channel] += value;
                  }
                }
              }
              break;
            default:
              throw new Error("Unknown blendOp");
          }
        }
        decodeImage() {
          const data = this._inflatorResult;
          if (this._filterMethod !== FilterMethod.ADAPTIVE) {
            throw new Error(`Filter method ${this._filterMethod} not supported`);
          }
          if (this._interlaceMethod === InterlaceMethod.NO_INTERLACE) {
            this._png.data = decodeInterlaceNull({
              data,
              width: this._png.width,
              height: this._png.height,
              channels: this._png.channels,
              depth: this._png.depth
            });
          } else if (this._interlaceMethod === InterlaceMethod.ADAM7) {
            this._png.data = decodeInterlaceAdam7({
              data,
              width: this._png.width,
              height: this._png.height,
              channels: this._png.channels,
              depth: this._png.depth
            });
          } else {
            throw new Error(`Interlace method ${this._interlaceMethod} not supported`);
          }
          if (this._hasPalette) {
            this._png.palette = this._palette;
          }
          if (this._hasTransparency) {
            this._png.transparency = this._transparency;
          }
        }
        pushDataToFrame() {
          this._inflator.push(new Uint8Array(0), true);
          const result = this._inflatorResult;
          const lastFrame = this._frames.at(-1);
          if (lastFrame) {
            lastFrame.data = result;
          } else {
            this._frames.push({
              sequenceNumber: 0,
              width: this._png.width,
              height: this._png.height,
              xOffset: 0,
              yOffset: 0,
              delayNumber: 0,
              delayDenominator: 0,
              disposeOp: DisposeOpType.NONE,
              blendOp: BlendOpType.SOURCE,
              data: result
            });
          }
          this._inflator = new Unzlib((chunk, final) => {
            this._chunks.push(chunk);
            if (final) {
              const totalLength = this._chunks.reduce((sum, c5) => sum + c5.length, 0);
              this._inflatorResult = new Uint8Array(totalLength);
              let offset = 0;
              for (const chunk2 of this._chunks) {
                this._inflatorResult.set(chunk2, offset);
                offset += chunk2.length;
              }
              this._chunks = [];
            }
          });
          this._chunks = [];
          this._writingDataChunks = false;
        }
      };
    }
  });

  // node_modules/fast-png/lib/png_encoder.js
  var init_png_encoder = __esm({
    "node_modules/fast-png/lib/png_encoder.js"() {
      init_iobuffer();
      init_crc();
      init_signature();
      init_text2();
      init_internal_types();
    }
  });

  // node_modules/fast-png/lib/types.js
  var init_types2 = __esm({
    "node_modules/fast-png/lib/types.js"() {
    }
  });

  // node_modules/fast-png/lib/convert_indexed_to_rgb.js
  var init_convert_indexed_to_rgb = __esm({
    "node_modules/fast-png/lib/convert_indexed_to_rgb.js"() {
    }
  });

  // node_modules/fast-png/lib/index.js
  function decodePng(data, options) {
    const decoder = new PngDecoder(data, options);
    return decoder.decode();
  }
  var init_lib3 = __esm({
    "node_modules/fast-png/lib/index.js"() {
      init_png_decoder();
      init_png_encoder();
      init_signature();
      init_types2();
      init_convert_indexed_to_rgb();
    }
  });

  // node_modules/roslib/dist/NativeWebSocketTransport-CF_ebnyS.js
  var NativeWebSocketTransport_CF_ebnyS_exports = {};
  __export(NativeWebSocketTransport_CF_ebnyS_exports, {
    NativeWebSocketTransport: () => r2
  });
  var r2;
  var init_NativeWebSocketTransport_CF_ebnyS = __esm({
    "node_modules/roslib/dist/NativeWebSocketTransport-CF_ebnyS.js"() {
      init_RosLib();
      r2 = class extends Pr {
        constructor(e2) {
          super(), this.socket = e2, this.registerEventListeners();
        }
        send(e2) {
          this.socket.send(JSON.stringify(e2));
        }
        close() {
          this.socket.close();
        }
        isConnecting() {
          return this.socket.readyState === WebSocket.CONNECTING;
        }
        isOpen() {
          return this.socket.readyState === WebSocket.OPEN;
        }
        isClosing() {
          return this.socket.readyState === WebSocket.CLOSING;
        }
        isClosed() {
          return this.socket.readyState === WebSocket.CLOSED;
        }
        registerEventListeners() {
          this.socket.onopen = (e2) => {
            this.emit("open", e2);
          }, this.socket.onclose = (e2) => {
            this.emit("close", e2);
          }, this.socket.onerror = (e2) => {
            this.emit("error", e2);
          }, this.socket.onmessage = (e2) => {
            this.handleRawMessage(e2.data);
          };
        }
      };
    }
  });

  // node_modules/ws/browser.js
  var require_browser = __commonJS({
    "node_modules/ws/browser.js"(exports, module) {
      "use strict";
      module.exports = function() {
        throw new Error(
          "ws does not work in the browser. Browser clients must use the native WebSocket object"
        );
      };
    }
  });

  // node_modules/roslib/dist/WsWebSocketTransport-6-v9C0gj.js
  var WsWebSocketTransport_6_v9C0gj_exports = {};
  __export(WsWebSocketTransport_6_v9C0gj_exports, {
    WsWebSocketTransport: () => i
  });
  var t3, i;
  var init_WsWebSocketTransport_6_v9C0gj = __esm({
    "node_modules/roslib/dist/WsWebSocketTransport-6-v9C0gj.js"() {
      t3 = __toESM(require_browser(), 1);
      init_RosLib();
      i = class extends Pr {
        constructor(e2) {
          super(), this.socket = e2, this.registerEventListeners();
        }
        send(e2) {
          this.socket.send(JSON.stringify(e2));
        }
        close() {
          this.socket.close();
        }
        isConnecting() {
          return this.socket.readyState === t3.WebSocket.CONNECTING;
        }
        isOpen() {
          return this.socket.readyState === t3.WebSocket.OPEN;
        }
        isClosing() {
          return this.socket.readyState === t3.WebSocket.CLOSING;
        }
        isClosed() {
          return this.socket.readyState === t3.WebSocket.CLOSED;
        }
        registerEventListeners() {
          this.socket.onopen = (e2) => {
            this.emit("open", e2);
          }, this.socket.onclose = (e2) => {
            this.emit("close", e2);
          }, this.socket.onerror = (e2) => {
            this.emit("error", e2);
          }, this.socket.onmessage = (e2) => {
            this.handleRawMessage(e2.data);
          };
        }
      };
    }
  });

  // node_modules/roslib/dist/RosLib.js
  var RosLib_exports = {};
  __export(RosLib_exports, {
    AbstractTransport: () => Pr,
    Action: () => nr,
    ActionClient: () => qt,
    ActionListener: () => kr,
    Goal: () => er,
    GoalStatus: () => Be,
    Param: () => Kt,
    Pose: () => pt,
    Quaternion: () => Qe,
    REVISION: () => Gr,
    ROS2TFClient: () => Ur,
    Ros: () => Lr,
    Service: () => re,
    SimpleActionServer: () => rr,
    TFClient: () => tr,
    Topic: () => fe,
    Transform: () => vt,
    UrdfAttrs: () => H3,
    UrdfBox: () => cr,
    UrdfColor: () => lr,
    UrdfCylinder: () => hr,
    UrdfJoint: () => Tr,
    UrdfLink: () => Er,
    UrdfMaterial: () => Vt,
    UrdfMesh: () => pr,
    UrdfModel: () => qr,
    UrdfSphere: () => fr,
    UrdfType: () => ft,
    UrdfVisual: () => dr,
    Vector3: () => Oe,
    WebSocketTransportFactory: () => ur,
    isElement: () => Ht,
    isRosbridgeActionFeedbackMessage: () => kt,
    isRosbridgeActionResultMessage: () => Ut,
    isRosbridgeAdvertiseActionMessage: () => Br,
    isRosbridgeAdvertiseMessage: () => Sr,
    isRosbridgeAdvertiseServiceMessage: () => Ir,
    isRosbridgeCallServiceMessage: () => At,
    isRosbridgeCancelActionGoalMessage: () => Pt,
    isRosbridgeFragmentMessage: () => Jt,
    isRosbridgeMessage: () => ut,
    isRosbridgePngMessage: () => Zt,
    isRosbridgePublishMessage: () => Bt,
    isRosbridgeSendActionGoalMessage: () => Lt,
    isRosbridgeServiceResponseMessage: () => Ft,
    isRosbridgeSetStatusLevelMessage: () => _r,
    isRosbridgeStatusMessage: () => Qt,
    isRosbridgeSubscribeMessage: () => Rr,
    isRosbridgeUnadvertiseActionMessage: () => Fr,
    isRosbridgeUnadvertiseMessage: () => Or,
    isRosbridgeUnadvertiseServiceMessage: () => xr,
    isRosbridgeUnsubscribeMessage: () => Mr,
    parseUrdfOrigin: () => zt
  });
  function ut(n2) {
    return n2 instanceof Object && "op" in n2 && typeof n2.op == "string";
  }
  function Qt(n2) {
    return n2.op === "status";
  }
  function _r(n2) {
    return n2.op === "set_level";
  }
  function Jt(n2) {
    return n2.op === "fragment";
  }
  function Zt(n2) {
    return n2.op === "png";
  }
  function Sr(n2) {
    return n2.op === "advertise";
  }
  function Or(n2) {
    return n2.op === "unadvertise";
  }
  function Bt(n2) {
    return n2.op === "publish";
  }
  function Rr(n2) {
    return n2.op === "subscribe";
  }
  function Mr(n2) {
    return n2.op === "unsubscribe";
  }
  function Ir(n2) {
    return n2.op === "advertise_service";
  }
  function xr(n2) {
    return n2.op === "unadvertise_service";
  }
  function At(n2) {
    return n2.op === "call_service";
  }
  function Ft(n2) {
    return n2.op === "service_response";
  }
  function Br(n2) {
    return n2.op === "advertise_action";
  }
  function Fr(n2) {
    return n2.op === "unadvertise_action";
  }
  function Lt(n2) {
    return n2.op === "send_action_goal";
  }
  function Pt(n2) {
    return n2.op === "cancel_action_goal";
  }
  function kt(n2) {
    return n2.op === "action_feedback";
  }
  function Ut(n2) {
    return n2.op === "action_result";
  }
  function ir(n2) {
    switch (n2) {
      case Be.STATUS_CANCELED:
        return "Action was canceled";
      case Be.STATUS_ABORTED:
        return "Action was aborted";
      case Be.STATUS_CANCELING:
        return "Action is canceling";
      case Be.STATUS_UNKNOWN:
        return "Action status unknown";
      default:
        return `Action failed with status ${String(n2)}`;
    }
  }
  function ar(n2) {
    const t4 = Uint8Array.from(atob(n2), (s3) => s3.charCodeAt(0)), u3 = or(t4);
    try {
      return JSON.parse(sr.decode(u3.data));
    } catch (s3) {
      throw new Error("Error parsing PNG JSON contents", { cause: s3 });
    }
  }
  function or(n2) {
    try {
      return decodePng(n2);
    } catch (t4) {
      throw new Error("Error decoding PNG buffer", { cause: t4 });
    }
  }
  function zt(n2) {
    const t4 = n2.getAttribute(H3.Xyz)?.split(" ");
    let u3 = new Oe();
    t4?.[0] && t4[1] && t4[2] && (u3 = new Oe({
      x: parseFloat(t4[0]),
      y: parseFloat(t4[1]),
      z: parseFloat(t4[2])
    }));
    const s3 = n2.getAttribute(H3.Rpy)?.split(" ");
    let c5 = new Qe();
    if (s3?.[0] && s3[1] && s3[2]) {
      const o5 = parseFloat(s3[0]), h4 = parseFloat(s3[1]), C2 = parseFloat(s3[2]), f7 = o5 / 2, g5 = h4 / 2, D = C2 / 2, p5 = Math.sin(f7) * Math.cos(g5) * Math.cos(D) - Math.cos(f7) * Math.sin(g5) * Math.sin(D), B3 = Math.cos(f7) * Math.sin(g5) * Math.cos(D) + Math.sin(f7) * Math.cos(g5) * Math.sin(D), k4 = Math.cos(f7) * Math.cos(g5) * Math.sin(D) - Math.sin(f7) * Math.sin(g5) * Math.cos(D), Y2 = Math.cos(f7) * Math.cos(g5) * Math.cos(D) + Math.sin(f7) * Math.sin(g5) * Math.sin(D);
      c5 = new Qe({
        x: p5,
        y: B3,
        z: k4,
        w: Y2
      }), c5.normalize();
    }
    return new pt({
      position: u3,
      orientation: c5
    });
  }
  function Ht(n2) {
    return n2.nodeType === 1;
  }
  function mr(n2) {
    let t4 = null;
    for (const s3 of n2.childNodes)
      if (Ht(s3)) {
        t4 = s3;
        break;
      }
    if (!t4)
      return null;
    const u3 = {
      xml: t4
    };
    switch (t4.nodeName) {
      case "sphere":
        return new fr(u3);
      case "box":
        return new cr(u3);
      case "cylinder":
        return new hr(u3);
      case "mesh":
        return new pr(u3);
      default:
        return console.warn(`Unknown geometry type ${t4.nodeName}`), null;
    }
  }
  function st() {
    if (wt) return ae2;
    wt = 1;
    function n2(M2, q3, W) {
      if (W === void 0 && (W = Array.prototype), M2 && typeof W.find == "function")
        return W.find.call(M2, q3);
      for (var ue2 = 0; ue2 < M2.length; ue2++)
        if (u3(M2, ue2)) {
          var me = M2[ue2];
          if (q3.call(void 0, me, ue2, M2))
            return me;
        }
    }
    function t4(M2, q3) {
      return q3 === void 0 && (q3 = Object), q3 && typeof q3.getOwnPropertyDescriptors == "function" && (M2 = q3.create(null, q3.getOwnPropertyDescriptors(M2))), q3 && typeof q3.freeze == "function" ? q3.freeze(M2) : M2;
    }
    function u3(M2, q3) {
      return Object.prototype.hasOwnProperty.call(M2, q3);
    }
    function s3(M2, q3) {
      if (M2 === null || typeof M2 != "object")
        throw new TypeError("target is not an object");
      for (var W in q3)
        u3(q3, W) && (M2[W] = q3[W]);
      return M2;
    }
    var c5 = t4({
      allowfullscreen: true,
      async: true,
      autofocus: true,
      autoplay: true,
      checked: true,
      controls: true,
      default: true,
      defer: true,
      disabled: true,
      formnovalidate: true,
      hidden: true,
      ismap: true,
      itemscope: true,
      loop: true,
      multiple: true,
      muted: true,
      nomodule: true,
      novalidate: true,
      open: true,
      playsinline: true,
      readonly: true,
      required: true,
      reversed: true,
      selected: true
    });
    function o5(M2) {
      return u3(c5, M2.toLowerCase());
    }
    var h4 = t4({
      area: true,
      base: true,
      br: true,
      col: true,
      embed: true,
      hr: true,
      img: true,
      input: true,
      link: true,
      meta: true,
      param: true,
      source: true,
      track: true,
      wbr: true
    });
    function C2(M2) {
      return u3(h4, M2.toLowerCase());
    }
    var f7 = t4({
      script: false,
      style: false,
      textarea: true,
      title: true
    });
    function g5(M2) {
      var q3 = M2.toLowerCase();
      return u3(f7, q3) && !f7[q3];
    }
    function D(M2) {
      var q3 = M2.toLowerCase();
      return u3(f7, q3) && f7[q3];
    }
    function p5(M2) {
      return M2 === k4.HTML;
    }
    function B3(M2) {
      return p5(M2) || M2 === k4.XML_XHTML_APPLICATION;
    }
    var k4 = t4({
      /**
       * `text/html`, the only mime type that triggers treating an XML document as HTML.
       *
       * @see https://www.iana.org/assignments/media-types/text/html IANA MimeType registration
       * @see https://en.wikipedia.org/wiki/HTML Wikipedia
       * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMParser/parseFromString MDN
       * @see https://html.spec.whatwg.org/multipage/dynamic-markup-insertion.html#dom-domparser-parsefromstring
       *      WHATWG HTML Spec
       */
      HTML: "text/html",
      /**
       * `application/xml`, the standard mime type for XML documents.
       *
       * @see https://www.iana.org/assignments/media-types/application/xml IANA MimeType
       *      registration
       * @see https://tools.ietf.org/html/rfc7303#section-9.1 RFC 7303
       * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
       */
      XML_APPLICATION: "application/xml",
      /**
       * `text/xml`, an alias for `application/xml`.
       *
       * @see https://tools.ietf.org/html/rfc7303#section-9.2 RFC 7303
       * @see https://www.iana.org/assignments/media-types/text/xml IANA MimeType registration
       * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
       */
      XML_TEXT: "text/xml",
      /**
       * `application/xhtml+xml`, indicates an XML document that has the default HTML namespace,
       * but is parsed as an XML document.
       *
       * @see https://www.iana.org/assignments/media-types/application/xhtml+xml IANA MimeType
       *      registration
       * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument WHATWG DOM Spec
       * @see https://en.wikipedia.org/wiki/XHTML Wikipedia
       */
      XML_XHTML_APPLICATION: "application/xhtml+xml",
      /**
       * `image/svg+xml`,
       *
       * @see https://www.iana.org/assignments/media-types/image/svg+xml IANA MimeType registration
       * @see https://www.w3.org/TR/SVG11/ W3C SVG 1.1
       * @see https://en.wikipedia.org/wiki/Scalable_Vector_Graphics Wikipedia
       */
      XML_SVG_IMAGE: "image/svg+xml"
    }), Y2 = Object.keys(k4).map(function(M2) {
      return k4[M2];
    });
    function X2(M2) {
      return Y2.indexOf(M2) > -1;
    }
    var te = t4({
      /**
       * The XHTML namespace.
       *
       * @see http://www.w3.org/1999/xhtml
       */
      HTML: "http://www.w3.org/1999/xhtml",
      /**
       * The SVG namespace.
       *
       * @see http://www.w3.org/2000/svg
       */
      SVG: "http://www.w3.org/2000/svg",
      /**
       * The `xml:` namespace.
       *
       * @see http://www.w3.org/XML/1998/namespace
       */
      XML: "http://www.w3.org/XML/1998/namespace",
      /**
       * The `xmlns:` namespace.
       *
       * @see https://www.w3.org/2000/xmlns/
       */
      XMLNS: "http://www.w3.org/2000/xmlns/"
    });
    return ae2.assign = s3, ae2.find = n2, ae2.freeze = t4, ae2.HTML_BOOLEAN_ATTRIBUTES = c5, ae2.HTML_RAW_TEXT_ELEMENTS = f7, ae2.HTML_VOID_ELEMENTS = h4, ae2.hasDefaultHTMLNamespace = B3, ae2.hasOwn = u3, ae2.isHTMLBooleanAttribute = o5, ae2.isHTMLRawTextElement = g5, ae2.isHTMLEscapableRawTextElement = D, ae2.isHTMLMimeType = p5, ae2.isHTMLVoidElement = C2, ae2.isValidMimeType = X2, ae2.MIME_TYPE = k4, ae2.NAMESPACE = te, ae2;
  }
  function dt() {
    if (yt) return it;
    yt = 1;
    var n2 = st();
    function t4(B3, k4) {
      B3.prototype = Object.create(Error.prototype, {
        constructor: { value: B3 },
        name: { value: B3.name, enumerable: true, writable: k4 }
      });
    }
    var u3 = n2.freeze({
      /**
       * the default value as defined by the spec
       */
      Error: "Error",
      /**
       * @deprecated
       * Use RangeError instead.
       */
      IndexSizeError: "IndexSizeError",
      /**
       * @deprecated
       * Just to match the related static code, not part of the spec.
       */
      DomstringSizeError: "DomstringSizeError",
      HierarchyRequestError: "HierarchyRequestError",
      WrongDocumentError: "WrongDocumentError",
      InvalidCharacterError: "InvalidCharacterError",
      /**
       * @deprecated
       * Just to match the related static code, not part of the spec.
       */
      NoDataAllowedError: "NoDataAllowedError",
      NoModificationAllowedError: "NoModificationAllowedError",
      NotFoundError: "NotFoundError",
      NotSupportedError: "NotSupportedError",
      InUseAttributeError: "InUseAttributeError",
      InvalidStateError: "InvalidStateError",
      SyntaxError: "SyntaxError",
      InvalidModificationError: "InvalidModificationError",
      NamespaceError: "NamespaceError",
      /**
       * @deprecated
       * Use TypeError for invalid arguments,
       * "NotSupportedError" DOMException for unsupported operations,
       * and "NotAllowedError" DOMException for denied requests instead.
       */
      InvalidAccessError: "InvalidAccessError",
      /**
       * @deprecated
       * Just to match the related static code, not part of the spec.
       */
      ValidationError: "ValidationError",
      /**
       * @deprecated
       * Use TypeError instead.
       */
      TypeMismatchError: "TypeMismatchError",
      SecurityError: "SecurityError",
      NetworkError: "NetworkError",
      AbortError: "AbortError",
      /**
       * @deprecated
       * Just to match the related static code, not part of the spec.
       */
      URLMismatchError: "URLMismatchError",
      QuotaExceededError: "QuotaExceededError",
      TimeoutError: "TimeoutError",
      InvalidNodeTypeError: "InvalidNodeTypeError",
      DataCloneError: "DataCloneError",
      EncodingError: "EncodingError",
      NotReadableError: "NotReadableError",
      UnknownError: "UnknownError",
      ConstraintError: "ConstraintError",
      DataError: "DataError",
      TransactionInactiveError: "TransactionInactiveError",
      ReadOnlyError: "ReadOnlyError",
      VersionError: "VersionError",
      OperationError: "OperationError",
      NotAllowedError: "NotAllowedError",
      OptOutError: "OptOutError"
    }), s3 = Object.keys(u3);
    function c5(B3) {
      return typeof B3 == "number" && B3 >= 1 && B3 <= 25;
    }
    function o5(B3) {
      return typeof B3 == "string" && B3.substring(B3.length - u3.Error.length) === u3.Error;
    }
    function h4(B3, k4) {
      c5(B3) ? (this.name = s3[B3], this.message = k4 || "") : (this.message = B3, this.name = o5(k4) ? k4 : u3.Error), Error.captureStackTrace && Error.captureStackTrace(this, h4);
    }
    t4(h4, true), Object.defineProperties(h4.prototype, {
      code: {
        enumerable: true,
        get: function() {
          var B3 = s3.indexOf(this.name);
          return c5(B3) ? B3 : 0;
        }
      }
    });
    for (var C2 = {
      INDEX_SIZE_ERR: 1,
      DOMSTRING_SIZE_ERR: 2,
      HIERARCHY_REQUEST_ERR: 3,
      WRONG_DOCUMENT_ERR: 4,
      INVALID_CHARACTER_ERR: 5,
      NO_DATA_ALLOWED_ERR: 6,
      NO_MODIFICATION_ALLOWED_ERR: 7,
      NOT_FOUND_ERR: 8,
      NOT_SUPPORTED_ERR: 9,
      INUSE_ATTRIBUTE_ERR: 10,
      INVALID_STATE_ERR: 11,
      SYNTAX_ERR: 12,
      INVALID_MODIFICATION_ERR: 13,
      NAMESPACE_ERR: 14,
      INVALID_ACCESS_ERR: 15,
      VALIDATION_ERR: 16,
      TYPE_MISMATCH_ERR: 17,
      SECURITY_ERR: 18,
      NETWORK_ERR: 19,
      ABORT_ERR: 20,
      URL_MISMATCH_ERR: 21,
      QUOTA_EXCEEDED_ERR: 22,
      TIMEOUT_ERR: 23,
      INVALID_NODE_TYPE_ERR: 24,
      DATA_CLONE_ERR: 25
    }, f7 = Object.entries(C2), g5 = 0; g5 < f7.length; g5++) {
      var D = f7[g5][0];
      h4[D] = f7[g5][1];
    }
    function p5(B3, k4) {
      this.message = B3, this.locator = k4, Error.captureStackTrace && Error.captureStackTrace(this, p5);
    }
    return t4(p5), it.DOMException = h4, it.DOMExceptionName = u3, it.ExceptionCode = C2, it.ParseError = p5, it;
  }
  function Yt() {
    if (_t) return L4;
    _t = 1;
    function n2(ee) {
      try {
        typeof ee != "function" && (ee = RegExp);
        var le = new ee("\u{1D306}", "u").exec("\u{1D306}");
        return !!le && le[0].length === 2;
      } catch {
      }
      return false;
    }
    var t4 = n2();
    function u3(ee) {
      if (ee.source[0] !== "[")
        throw new Error(ee + " can not be used with chars");
      return ee.source.slice(1, ee.source.lastIndexOf("]"));
    }
    function s3(ee, le) {
      if (ee.source[0] !== "[")
        throw new Error("/" + ee.source + "/ can not be used with chars_without");
      if (!le || typeof le != "string")
        throw new Error(JSON.stringify(le) + " is not a valid search");
      if (ee.source.indexOf(le) === -1)
        throw new Error('"' + le + '" is not is /' + ee.source + "/");
      if (le === "-" && ee.source.indexOf(le) !== 1)
        throw new Error('"' + le + '" is not at the first postion of /' + ee.source + "/");
      return new RegExp(ee.source.replace(le, ""), t4 ? "u" : "");
    }
    function c5(ee) {
      var le = this;
      return new RegExp(
        Array.prototype.slice.call(arguments).map(function(Me) {
          var Ie = typeof Me == "string";
          if (Ie && le === void 0 && Me === "|")
            throw new Error("use regg instead of reg to wrap expressions with `|`!");
          return Ie ? Me : Me.source;
        }).join(""),
        t4 ? "mu" : "m"
      );
    }
    function o5(ee) {
      if (arguments.length === 0)
        throw new Error("no parameters provided");
      return c5.apply(o5, ["(?:"].concat(Array.prototype.slice.call(arguments), [")"]));
    }
    var h4 = "\uFFFD", C2 = /[-\x09\x0A\x0D\x20-\x2C\x2E-\uD7FF\uE000-\uFFFD]/;
    t4 && (C2 = c5("[", u3(C2), "\\u{10000}-\\u{10FFFF}", "]"));
    var f7 = /[\x20\x09\x0D\x0A]/, g5 = u3(f7), D = c5(f7, "+"), p5 = c5(f7, "*"), B3 = /[:_a-zA-Z\xC0-\xD6\xD8-\xF6\xF8-\u02FF\u0370-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD]/;
    t4 && (B3 = c5("[", u3(B3), "\\u{10000}-\\u{10FFFF}", "]"));
    var k4 = u3(B3), Y2 = c5("[", k4, u3(/[-.0-9\xB7]/), u3(/[\u0300-\u036F\u203F-\u2040]/), "]"), X2 = c5(B3, Y2, "*"), te = c5(Y2, "+"), M2 = c5("&", X2, ";"), q3 = o5(/&#[0-9]+;|&#x[0-9a-fA-F]+;/), W = o5(M2, "|", q3), ue2 = c5("%", X2, ";"), me = o5(
      c5('"', o5(/[^%&"]/, "|", ue2, "|", W), "*", '"'),
      "|",
      c5("'", o5(/[^%&']/, "|", ue2, "|", W), "*", "'")
    ), d6 = o5('"', o5(/[^<&"]/, "|", W), "*", '"', "|", "'", o5(/[^<&']/, "|", W), "*", "'"), _3 = s3(B3, ":"), x5 = s3(Y2, ":"), U5 = c5(_3, x5, "*"), $3 = c5(U5, o5(":", U5), "?"), J2 = c5("^", $3, "$"), Ne = c5("(", $3, ")"), Z3 = o5(/"[^"]*"|'[^']*'/), we = c5(/^<\?/, "(", X2, ")", o5(D, "(", C2, "*?)"), "?", /\?>/), l6 = /[\x20\x0D\x0Aa-zA-Z0-9-'()+,./:=?;!*#@$_%]/, A6 = o5('"', l6, '*"', "|", "'", s3(l6, "'"), "*'"), b5 = "<!--", v3 = "-->", y7 = c5(b5, o5(s3(C2, "-"), "|", c5("-", s3(C2, "-"))), "*", v3), E3 = "#PCDATA", S4 = o5(
      c5(/\(/, p5, E3, o5(p5, /\|/, p5, $3), "*", p5, /\)\*/),
      "|",
      c5(/\(/, p5, E3, p5, /\)/)
    ), V2 = /[?*+]?/, O6 = c5(
      /\([^>]+\)/,
      V2
      /*regg(choice, '|', seq), _children_quantity*/
    ), T2 = o5("EMPTY", "|", "ANY", "|", S4, "|", O6), w3 = "<!ELEMENT", I4 = c5(w3, D, o5($3, "|", ue2), D, o5(T2, "|", ue2), p5, ">"), P = c5("NOTATION", D, /\(/, p5, X2, o5(p5, /\|/, p5, X2), "*", p5, /\)/), oe = c5(/\(/, p5, te, o5(p5, /\|/, p5, te), "*", p5, /\)/), Te = o5(P, "|", oe), Ee = o5(/CDATA|ID|IDREF|IDREFS|ENTITY|ENTITIES|NMTOKEN|NMTOKENS/, "|", Te), ie = o5(/#REQUIRED|#IMPLIED/, "|", o5(o5("#FIXED", D), "?", d6)), R5 = o5(D, X2, D, Ee, D, ie), Fe = "<!ATTLIST", ye = c5(Fe, D, X2, R5, "*", p5, ">"), ce2 = "about:legacy-compat", Le = o5('"' + ce2 + '"', "|", "'" + ce2 + "'"), _e = "SYSTEM", Ce = "PUBLIC", Re = o5(o5(_e, D, Z3), "|", o5(Ce, D, A6, D, Z3)), Ue = c5(
      "^",
      o5(
        o5(_e, D, "(?<SystemLiteralOnly>", Z3, ")"),
        "|",
        o5(Ce, D, "(?<PubidLiteral>", A6, ")", D, "(?<SystemLiteral>", Z3, ")")
      )
    ), qe = o5(D, "NDATA", D, X2), Ae = o5(me, "|", o5(Re, qe, "?")), j2 = "<!ENTITY", Pe = c5(j2, D, X2, D, Ae, p5, ">"), ne = o5(me, "|", Re), Ge = c5(j2, D, "%", D, X2, D, ne, p5, ">"), at = o5(Pe, "|", Ge), Ve = c5(Ce, D, A6), ze = c5("<!NOTATION", D, X2, D, o5(Re, "|", Ve), p5, ">"), F4 = c5(p5, "=", p5), Q2 = /1[.]\d+/, De = c5(D, "version", F4, o5("'", Q2, "'", "|", '"', Q2, '"')), ge = /[A-Za-z][-A-Za-z0-9._]*/, He = o5(D, "encoding", F4, o5('"', ge, '"', "|", "'", ge, "'")), Je = o5(D, "standalone", F4, o5("'", o5("yes", "|", "no"), "'", "|", '"', o5("yes", "|", "no"), '"')), Ze = c5(/^<\?xml/, De, He, "?", Je, "?", p5, /\?>/), Ke = "<!DOCTYPE", ot = "<![CDATA[", ct = "]]>", et2 = /<!\[CDATA\[/, Ye = /\]\]>/, tt = c5(C2, "*?", Ye), mt = c5(et2, tt);
    return L4.chars = u3, L4.chars_without = s3, L4.detectUnicodeSupport = n2, L4.reg = c5, L4.regg = o5, L4.ABOUT_LEGACY_COMPAT = ce2, L4.ABOUT_LEGACY_COMPAT_SystemLiteral = Le, L4.AttlistDecl = ye, L4.CDATA_START = ot, L4.CDATA_END = ct, L4.CDSect = mt, L4.Char = C2, L4.Comment = y7, L4.COMMENT_START = b5, L4.COMMENT_END = v3, L4.DOCTYPE_DECL_START = Ke, L4.elementdecl = I4, L4.EntityDecl = at, L4.EntityValue = me, L4.ExternalID = Re, L4.ExternalID_match = Ue, L4.Name = X2, L4.NotationDecl = ze, L4.Reference = W, L4.PEReference = ue2, L4.PI = we, L4.PUBLIC = Ce, L4.PubidLiteral = A6, L4.QName = $3, L4.QName_exact = J2, L4.QName_group = Ne, L4.S = D, L4.SChar_s = g5, L4.S_OPT = p5, L4.SYSTEM = _e, L4.SystemLiteral = Z3, L4.UNICODE_REPLACEMENT_CHARACTER = h4, L4.UNICODE_SUPPORT = t4, L4.XMLDecl = Ze, L4;
  }
  function Xt() {
    if (St) return K2;
    St = 1;
    var n2 = st(), t4 = n2.find, u3 = n2.hasDefaultHTMLNamespace, s3 = n2.hasOwn, c5 = n2.isHTMLMimeType, o5 = n2.isHTMLRawTextElement, h4 = n2.isHTMLVoidElement, C2 = n2.MIME_TYPE, f7 = n2.NAMESPACE, g5 = /* @__PURE__ */ Symbol(), D = dt(), p5 = D.DOMException, B3 = D.DOMExceptionName, k4 = Yt();
    function Y2(e2) {
      if (e2 !== g5)
        throw new TypeError("Illegal constructor");
    }
    function X2(e2) {
      return e2 !== "";
    }
    function te(e2) {
      return e2 ? e2.split(/[\t\n\f\r ]+/).filter(X2) : [];
    }
    function M2(e2, r3) {
      return s3(e2, r3) || (e2[r3] = true), e2;
    }
    function q3(e2) {
      if (!e2) return [];
      var r3 = te(e2);
      return Object.keys(r3.reduce(M2, {}));
    }
    function W(e2) {
      return function(r3) {
        return e2 && e2.indexOf(r3) !== -1;
      };
    }
    function ue2(e2) {
      if (!k4.QName_exact.test(e2))
        throw new p5(p5.INVALID_CHARACTER_ERR, 'invalid character in qualified name "' + e2 + '"');
    }
    function me(e2, r3) {
      ue2(r3), e2 = e2 || null;
      var i2 = null, a4 = r3;
      if (r3.indexOf(":") >= 0) {
        var m4 = r3.split(":");
        i2 = m4[0], a4 = m4[1];
      }
      if (i2 !== null && e2 === null)
        throw new p5(p5.NAMESPACE_ERR, "prefix is non-null and namespace is null");
      if (i2 === "xml" && e2 !== n2.NAMESPACE.XML)
        throw new p5(p5.NAMESPACE_ERR, 'prefix is "xml" and namespace is not the XML namespace');
      if ((i2 === "xmlns" || r3 === "xmlns") && e2 !== n2.NAMESPACE.XMLNS)
        throw new p5(
          p5.NAMESPACE_ERR,
          'either qualifiedName or prefix is "xmlns" and namespace is not the XMLNS namespace'
        );
      if (e2 === n2.NAMESPACE.XMLNS && i2 !== "xmlns" && r3 !== "xmlns")
        throw new p5(
          p5.NAMESPACE_ERR,
          'namespace is the XMLNS namespace and neither qualifiedName nor prefix is "xmlns"'
        );
      return [e2, i2, a4];
    }
    function d6(e2, r3) {
      for (var i2 in e2)
        s3(e2, i2) && (r3[i2] = e2[i2]);
    }
    function _3(e2, r3) {
      var i2 = e2.prototype;
      if (!(i2 instanceof r3)) {
        let a4 = function() {
        };
        a4.prototype = r3.prototype, a4 = new a4(), d6(i2, a4), e2.prototype = i2 = a4;
      }
      i2.constructor != e2 && (typeof e2 != "function" && console.error("unknown Class:" + e2), i2.constructor = e2);
    }
    var x5 = {}, U5 = x5.ELEMENT_NODE = 1, $3 = x5.ATTRIBUTE_NODE = 2, J2 = x5.TEXT_NODE = 3, Ne = x5.CDATA_SECTION_NODE = 4, Z3 = x5.ENTITY_REFERENCE_NODE = 5, we = x5.ENTITY_NODE = 6, l6 = x5.PROCESSING_INSTRUCTION_NODE = 7, A6 = x5.COMMENT_NODE = 8, b5 = x5.DOCUMENT_NODE = 9, v3 = x5.DOCUMENT_TYPE_NODE = 10, y7 = x5.DOCUMENT_FRAGMENT_NODE = 11, E3 = x5.NOTATION_NODE = 12, S4 = n2.freeze({
      DOCUMENT_POSITION_DISCONNECTED: 1,
      DOCUMENT_POSITION_PRECEDING: 2,
      DOCUMENT_POSITION_FOLLOWING: 4,
      DOCUMENT_POSITION_CONTAINS: 8,
      DOCUMENT_POSITION_CONTAINED_BY: 16,
      DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC: 32
    });
    function V2(e2, r3) {
      if (r3.length < e2.length) return V2(r3, e2);
      var i2 = null;
      for (var a4 in e2) {
        if (e2[a4] !== r3[a4]) return i2;
        i2 = e2[a4];
      }
      return i2;
    }
    function O6(e2) {
      return e2.guid || (e2.guid = Math.random()), e2.guid;
    }
    function T2() {
    }
    T2.prototype = {
      /**
       * The number of nodes in the list. The range of valid child node indices is 0 to length-1
       * inclusive.
       *
       * @type {number}
       */
      length: 0,
      /**
       * Returns the item at `index`. If index is greater than or equal to the number of nodes in
       * the list, this returns null.
       *
       * @param index
       * Unsigned long Index into the collection.
       * @returns {Node | null}
       * The node at position `index` in the NodeList,
       * or null if that is not a valid index.
       */
      item: function(e2) {
        return e2 >= 0 && e2 < this.length ? this[e2] : null;
      },
      /**
       * Returns a string representation of the NodeList.
       *
       * @param {unknown} nodeFilter
       * __A filter function? Not implemented according to the spec?__.
       * @returns {string}
       * A string representation of the NodeList.
       */
      toString: function(e2) {
        for (var r3 = [], i2 = 0; i2 < this.length; i2++)
          Ie(this[i2], r3, e2);
        return r3.join("");
      },
      /**
       * Filters the NodeList based on a predicate.
       *
       * @param {function(Node): boolean} predicate
       * - A predicate function to filter the NodeList.
       * @returns {Node[]}
       * An array of nodes that satisfy the predicate.
       * @private
       */
      filter: function(e2) {
        return Array.prototype.filter.call(this, e2);
      },
      /**
       * Returns the first index at which a given node can be found in the NodeList, or -1 if it is
       * not present.
       *
       * @param {Node} item
       * - The Node item to locate in the NodeList.
       * @returns {number}
       * The first index of the node in the NodeList; -1 if not found.
       * @private
       */
      indexOf: function(e2) {
        return Array.prototype.indexOf.call(this, e2);
      }
    }, T2.prototype[Symbol.iterator] = function() {
      var e2 = this, r3 = 0;
      return {
        next: function() {
          return r3 < e2.length ? {
            value: e2[r3++],
            done: false
          } : {
            done: true
          };
        },
        return: function() {
          return {
            done: true
          };
        }
      };
    };
    function w3(e2, r3) {
      this._node = e2, this._refresh = r3, I4(this);
    }
    function I4(e2) {
      var r3 = e2._node._inc || e2._node.ownerDocument._inc;
      if (e2._inc !== r3) {
        var i2 = e2._refresh(e2._node);
        if (Ct(e2, "length", i2.length), !e2.$$length || i2.length < e2.$$length)
          for (var a4 = i2.length; a4 in e2; a4++)
            s3(e2, a4) && delete e2[a4];
        d6(i2, e2), e2._inc = r3;
      }
    }
    w3.prototype.item = function(e2) {
      return I4(this), this[e2] || null;
    }, _3(w3, T2);
    function P() {
    }
    function oe(e2, r3) {
      for (var i2 = 0; i2 < e2.length; ) {
        if (e2[i2] === r3)
          return i2;
        i2++;
      }
    }
    function Te(e2, r3, i2, a4) {
      if (a4 ? r3[oe(r3, a4)] = i2 : (r3[r3.length] = i2, r3.length++), e2) {
        i2.ownerElement = e2;
        var m4 = e2.ownerDocument;
        m4 && (a4 && _e(m4, e2, a4), Le(m4, e2, i2));
      }
    }
    function Ee(e2, r3, i2) {
      var a4 = oe(r3, i2);
      if (a4 >= 0) {
        for (var m4 = r3.length - 1; a4 <= m4; )
          r3[a4] = r3[++a4];
        if (r3.length = m4, e2) {
          var N4 = e2.ownerDocument;
          N4 && _e(N4, e2, i2), i2.ownerElement = null;
        }
      }
    }
    P.prototype = {
      length: 0,
      item: T2.prototype.item,
      /**
       * Get an attribute by name. Note: Name is in lower case in case of HTML namespace and
       * document.
       *
       * @param {string} localName
       * The local name of the attribute.
       * @returns {Attr | null}
       * The attribute with the given local name, or null if no such attribute exists.
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-name
       */
      getNamedItem: function(e2) {
        this._ownerElement && this._ownerElement._isInHTMLDocumentAndNamespace() && (e2 = e2.toLowerCase());
        for (var r3 = 0; r3 < this.length; ) {
          var i2 = this[r3];
          if (i2.nodeName === e2)
            return i2;
          r3++;
        }
        return null;
      },
      /**
       * Set an attribute.
       *
       * @param {Attr} attr
       * The attribute to set.
       * @returns {Attr | null}
       * The old attribute with the same local name and namespace URI as the new one, or null if no
       * such attribute exists.
       * @throws {DOMException}
       * With code:
       * - {@link INUSE_ATTRIBUTE_ERR} - If the attribute is already an attribute of another
       * element.
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
       */
      setNamedItem: function(e2) {
        var r3 = e2.ownerElement;
        if (r3 && r3 !== this._ownerElement)
          throw new p5(p5.INUSE_ATTRIBUTE_ERR);
        var i2 = this.getNamedItemNS(e2.namespaceURI, e2.localName);
        return i2 === e2 ? e2 : (Te(this._ownerElement, this, e2, i2), i2);
      },
      /**
       * Set an attribute, replacing an existing attribute with the same local name and namespace
       * URI if one exists.
       *
       * @param {Attr} attr
       * The attribute to set.
       * @returns {Attr | null}
       * The old attribute with the same local name and namespace URI as the new one, or null if no
       * such attribute exists.
       * @throws {DOMException}
       * Throws a DOMException with the name "InUseAttributeError" if the attribute is already an
       * attribute of another element.
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
       */
      setNamedItemNS: function(e2) {
        return this.setNamedItem(e2);
      },
      /**
       * Removes an attribute specified by the local name.
       *
       * @param {string} localName
       * The local name of the attribute to be removed.
       * @returns {Attr}
       * The attribute node that was removed.
       * @throws {DOMException}
       * With code:
       * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given name is found.
       * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditem
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-name
       */
      removeNamedItem: function(e2) {
        var r3 = this.getNamedItem(e2);
        if (!r3)
          throw new p5(p5.NOT_FOUND_ERR, e2);
        return Ee(this._ownerElement, this, r3), r3;
      },
      /**
       * Removes an attribute specified by the namespace and local name.
       *
       * @param {string | null} namespaceURI
       * The namespace URI of the attribute to be removed.
       * @param {string} localName
       * The local name of the attribute to be removed.
       * @returns {Attr}
       * The attribute node that was removed.
       * @throws {DOMException}
       * With code:
       * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given namespace URI and local
       * name is found.
       * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditemns
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-namespace
       */
      removeNamedItemNS: function(e2, r3) {
        var i2 = this.getNamedItemNS(e2, r3);
        if (!i2)
          throw new p5(p5.NOT_FOUND_ERR, e2 ? e2 + " : " + r3 : r3);
        return Ee(this._ownerElement, this, i2), i2;
      },
      /**
       * Get an attribute by namespace and local name.
       *
       * @param {string | null} namespaceURI
       * The namespace URI of the attribute.
       * @param {string} localName
       * The local name of the attribute.
       * @returns {Attr | null}
       * The attribute with the given namespace URI and local name, or null if no such attribute
       * exists.
       * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-namespace
       */
      getNamedItemNS: function(e2, r3) {
        e2 || (e2 = null);
        for (var i2 = 0; i2 < this.length; ) {
          var a4 = this[i2];
          if (a4.localName === r3 && a4.namespaceURI === e2)
            return a4;
          i2++;
        }
        return null;
      }
    }, P.prototype[Symbol.iterator] = function() {
      var e2 = this, r3 = 0;
      return {
        next: function() {
          return r3 < e2.length ? {
            value: e2[r3++],
            done: false
          } : {
            done: true
          };
        },
        return: function() {
          return {
            done: true
          };
        }
      };
    };
    function ie() {
    }
    ie.prototype = {
      /**
       * Test if the DOM implementation implements a specific feature and version, as specified in
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/core.html#DOMFeatures DOM Features}.
       *
       * The DOMImplementation.hasFeature() method returns a Boolean flag indicating if a given
       * feature is supported. The different implementations fairly diverged in what kind of
       * features were reported. The latest version of the spec settled to force this method to
       * always return true, where the functionality was accurate and in use.
       *
       * @deprecated
       * It is deprecated and modern browsers return true in all cases.
       * @function DOMImplementation#hasFeature
       * @param {string} feature
       * The name of the feature to test.
       * @param {string} [version]
       * This is the version number of the feature to test.
       * @returns {boolean}
       * Always returns true.
       * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/hasFeature MDN
       * @see https://www.w3.org/TR/REC-DOM-Level-1/level-one-core.html#ID-5CED94D7 DOM Level 1 Core
       * @see https://dom.spec.whatwg.org/#dom-domimplementation-hasfeature DOM Living Standard
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-5CED94D7 DOM Level 3 Core
       */
      hasFeature: function(e2, r3) {
        return true;
      },
      /**
       * Creates a DOM Document object of the specified type with its document element. Note that
       * based on the {@link DocumentType}
       * given to create the document, the implementation may instantiate specialized
       * {@link Document} objects that support additional features than the "Core", such as "HTML"
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML}.
       * On the other hand, setting the {@link DocumentType} after the document was created makes
       * this very unlikely to happen. Alternatively, specialized {@link Document} creation methods,
       * such as createHTMLDocument
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML},
       * can be used to obtain specific types of {@link Document} objects.
       *
       * __It behaves slightly different from the description in the living standard__:
       * - There is no interface/class `XMLDocument`, it returns a `Document`
       * instance (with it's `type` set to `'xml'`).
       * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
       *
       * @function DOMImplementation.createDocument
       * @param {string | null} namespaceURI
       * The
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-namespaceURI namespace URI}
       * of the document element to create or null.
       * @param {string | null} qualifiedName
       * The
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified name}
       * of the document element to be created or null.
       * @param {DocumentType | null} [doctype=null]
       * The type of document to be created or null. When doctype is not null, its
       * {@link Node#ownerDocument} attribute is set to the document being created. Default is
       * `null`
       * @returns {Document}
       * A new {@link Document} object with its document element. If the NamespaceURI,
       * qualifiedName, and doctype are null, the returned {@link Document} is empty with no
       * document element.
       * @throws {DOMException}
       * With code:
       *
       * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
       * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
       * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed, if the qualifiedName has a
       * prefix and the namespaceURI is null, or if the qualifiedName is null and the namespaceURI
       * is different from null, or if the qualifiedName has a prefix that is "xml" and the
       * namespaceURI is different from "{@link http://www.w3.org/XML/1998/namespace}"
       * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#Namespaces XML Namespaces},
       * or if the DOM implementation does not support the "XML" feature but a non-null namespace
       * URI was provided, since namespaces were defined by XML.
       * - `WRONG_DOCUMENT_ERR`: Raised if doctype has already been used with a different document
       * or was created from a different implementation.
       * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
       * "XML" and the language exposed through the Document does not support XML Namespaces (such
       * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
       * @since DOM Level 2.
       * @see {@link #createHTMLDocument}
       * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocument MDN
       * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument DOM Living Standard
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-2-Core-DOM-createDocument DOM
       *      Level 3 Core
       * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocument DOM
       *      Level 2 Core (initial)
       */
      createDocument: function(e2, r3, i2) {
        var a4 = C2.XML_APPLICATION;
        e2 === f7.HTML ? a4 = C2.XML_XHTML_APPLICATION : e2 === f7.SVG && (a4 = C2.XML_SVG_IMAGE);
        var m4 = new ce2(g5, { contentType: a4 });
        if (m4.implementation = this, m4.childNodes = new T2(), m4.doctype = i2 || null, i2 && m4.appendChild(i2), r3) {
          var N4 = m4.createElementNS(e2, r3);
          m4.appendChild(N4);
        }
        return m4;
      },
      /**
       * Creates an empty DocumentType node. Entity declarations and notations are not made
       * available. Entity reference expansions and default attribute additions do not occur.
       *
       * **This behavior is slightly different from the one in the specs**:
       * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
       * - `publicId` and `systemId` contain the raw data including any possible quotes,
       *   so they can always be serialized back to the original value
       * - `internalSubset` contains the raw string between `[` and `]` if present,
       *   but is not parsed or validated in any form.
       *
       * @function DOMImplementation#createDocumentType
       * @param {string} qualifiedName
       * The {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified
       * name} of the document type to be created.
       * @param {string} [publicId]
       * The external subset public identifier.
       * @param {string} [systemId]
       * The external subset system identifier.
       * @param {string} [internalSubset]
       * the internal subset or an empty string if it is not present
       * @returns {DocumentType}
       * A new {@link DocumentType} node with {@link Node#ownerDocument} set to null.
       * @throws {DOMException}
       * With code:
       *
       * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
       * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
       * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed.
       * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
       * "XML" and the language exposed through the Document does not support XML Namespaces (such
       * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
       * @since DOM Level 2.
       * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocumentType
       *      MDN
       * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocumenttype DOM Living
       *      Standard
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-3-Core-DOM-createDocType DOM
       *      Level 3 Core
       * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocType DOM
       *      Level 2 Core
       * @see https://github.com/xmldom/xmldom/blob/master/CHANGELOG.md#050
       * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-Core-DocType-internalSubset
       * @prettierignore
       */
      createDocumentType: function(e2, r3, i2, a4) {
        ue2(e2);
        var m4 = new Ke(g5);
        return m4.name = e2, m4.nodeName = e2, m4.publicId = r3 || "", m4.systemId = i2 || "", m4.internalSubset = a4 || "", m4.childNodes = new T2(), m4;
      },
      /**
       * Returns an HTML document, that might already have a basic DOM structure.
       *
       * __It behaves slightly different from the description in the living standard__:
       * - If the first argument is `false` no initial nodes are added (steps 3-7 in the specs are
       * omitted)
       * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
       *
       * @param {string | false} [title]
       * A string containing the title to give the new HTML document.
       * @returns {Document}
       * The HTML document.
       * @since WHATWG Living Standard.
       * @see {@link #createDocument}
       * @see https://dom.spec.whatwg.org/#dom-domimplementation-createhtmldocument
       * @see https://dom.spec.whatwg.org/#html-document
       */
      createHTMLDocument: function(e2) {
        var r3 = new ce2(g5, { contentType: C2.HTML });
        if (r3.implementation = this, r3.childNodes = new T2(), e2 !== false) {
          r3.doctype = this.createDocumentType("html"), r3.doctype.ownerDocument = r3, r3.appendChild(r3.doctype);
          var i2 = r3.createElement("html");
          r3.appendChild(i2);
          var a4 = r3.createElement("head");
          if (i2.appendChild(a4), typeof e2 == "string") {
            var m4 = r3.createElement("title");
            m4.appendChild(r3.createTextNode(e2)), a4.appendChild(m4);
          }
          i2.appendChild(r3.createElement("body"));
        }
        return r3;
      }
    };
    function R5(e2) {
      Y2(e2);
    }
    R5.prototype = {
      /**
       * The first child of this node.
       *
       * @type {Node | null}
       */
      firstChild: null,
      /**
       * The last child of this node.
       *
       * @type {Node | null}
       */
      lastChild: null,
      /**
       * The previous sibling of this node.
       *
       * @type {Node | null}
       */
      previousSibling: null,
      /**
       * The next sibling of this node.
       *
       * @type {Node | null}
       */
      nextSibling: null,
      /**
       * The parent node of this node.
       *
       * @type {Node | null}
       */
      parentNode: null,
      /**
       * The parent element of this node.
       *
       * @type {Element | null}
       */
      get parentElement() {
        return this.parentNode && this.parentNode.nodeType === this.ELEMENT_NODE ? this.parentNode : null;
      },
      /**
       * The child nodes of this node.
       *
       * @type {NodeList}
       */
      childNodes: null,
      /**
       * The document object associated with this node.
       *
       * @type {Document | null}
       */
      ownerDocument: null,
      /**
       * The value of this node.
       *
       * @type {string | null}
       */
      nodeValue: null,
      /**
       * The namespace URI of this node.
       *
       * @type {string | null}
       */
      namespaceURI: null,
      /**
       * The prefix of the namespace for this node.
       *
       * @type {string | null}
       */
      prefix: null,
      /**
       * The local part of the qualified name of this node.
       *
       * @type {string | null}
       */
      localName: null,
      /**
       * The baseURI is currently always `about:blank`,
       * since that's what happens when you create a document from scratch.
       *
       * @type {'about:blank'}
       */
      baseURI: "about:blank",
      /**
       * Is true if this node is part of a document.
       *
       * @type {boolean}
       */
      get isConnected() {
        var e2 = this.getRootNode();
        return e2 && e2.nodeType === e2.DOCUMENT_NODE;
      },
      /**
       * Checks whether `other` is an inclusive descendant of this node.
       *
       * @param {Node | null | undefined} other
       * The node to check.
       * @returns {boolean}
       * True if `other` is an inclusive descendant of this node; false otherwise.
       * @see https://dom.spec.whatwg.org/#dom-node-contains
       */
      contains: function(e2) {
        if (!e2) return false;
        var r3 = e2;
        do {
          if (this === r3) return true;
          r3 = e2.parentNode;
        } while (r3);
        return false;
      },
      /**
       * @typedef GetRootNodeOptions
       * @property {boolean} [composed=false]
       */
      /**
       * Searches for the root node of this node.
       *
       * **This behavior is slightly different from the in the specs**:
       * - ignores `options.composed`, since `ShadowRoot`s are unsupported, always returns root.
       *
       * @param {GetRootNodeOptions} [options]
       * @returns {Node}
       * Root node.
       * @see https://dom.spec.whatwg.org/#dom-node-getrootnode
       * @see https://dom.spec.whatwg.org/#concept-shadow-including-root
       */
      getRootNode: function(e2) {
        var r3 = this;
        do {
          if (!r3.parentNode)
            return r3;
          r3 = r3.parentNode;
        } while (r3);
      },
      /**
       * Checks whether the given node is equal to this node.
       *
       * @param {Node} [otherNode]
       * @see https://dom.spec.whatwg.org/#concept-node-equals
       */
      isEqualNode: function(e2) {
        if (!e2 || this.nodeType !== e2.nodeType) return false;
        switch (this.nodeType) {
          case this.DOCUMENT_TYPE_NODE:
            if (this.name !== e2.name || this.publicId !== e2.publicId || this.systemId !== e2.systemId) return false;
            break;
          case this.ELEMENT_NODE:
            if (this.namespaceURI !== e2.namespaceURI || this.prefix !== e2.prefix || this.localName !== e2.localName || this.attributes.length !== e2.attributes.length) return false;
            for (var r3 = 0; r3 < this.attributes.length; r3++) {
              var i2 = this.attributes.item(r3);
              if (!i2.isEqualNode(e2.getAttributeNodeNS(i2.namespaceURI, i2.localName)))
                return false;
            }
            break;
          case this.ATTRIBUTE_NODE:
            if (this.namespaceURI !== e2.namespaceURI || this.localName !== e2.localName || this.value !== e2.value) return false;
            break;
          case this.PROCESSING_INSTRUCTION_NODE:
            if (this.target !== e2.target || this.data !== e2.data)
              return false;
            break;
          case this.TEXT_NODE:
          case this.COMMENT_NODE:
            if (this.data !== e2.data) return false;
            break;
        }
        if (this.childNodes.length !== e2.childNodes.length)
          return false;
        for (var r3 = 0; r3 < this.childNodes.length; r3++)
          if (!this.childNodes[r3].isEqualNode(e2.childNodes[r3]))
            return false;
        return true;
      },
      /**
       * Checks whether or not the given node is this node.
       *
       * @param {Node} [otherNode]
       */
      isSameNode: function(e2) {
        return this === e2;
      },
      /**
       * Inserts a node before a reference node as a child of this node.
       *
       * @param {Node} newChild
       * The new child node to be inserted.
       * @param {Node | null} refChild
       * The reference node before which newChild will be inserted.
       * @returns {Node}
       * The new child node successfully inserted.
       * @throws {DOMException}
       * Throws a DOMException if inserting the node would result in a DOM tree that is not
       * well-formed, or if `child` is provided but is not a child of `parent`.
       * See {@link _insertBefore} for more details.
       * @since Modified in DOM L2
       */
      insertBefore: function(e2, r3) {
        return F4(this, e2, r3);
      },
      /**
       * Replaces an old child node with a new child node within this node.
       *
       * @param {Node} newChild
       * The new node that is to replace the old node.
       * If it already exists in the DOM, it is removed from its original position.
       * @param {Node} oldChild
       * The existing child node to be replaced.
       * @returns {Node}
       * Returns the replaced child node.
       * @throws {DOMException}
       * Throws a DOMException if replacing the node would result in a DOM tree that is not
       * well-formed, or if `oldChild` is not a child of `this`.
       * This can also occur if the pre-replacement validity assertion fails.
       * See {@link _insertBefore}, {@link Node.removeChild}, and
       * {@link assertPreReplacementValidityInDocument} for more details.
       * @see https://dom.spec.whatwg.org/#concept-node-replace
       */
      replaceChild: function(e2, r3) {
        F4(this, e2, r3, ze), r3 && this.removeChild(r3);
      },
      /**
       * Removes an existing child node from this node.
       *
       * @param {Node} oldChild
       * The child node to be removed.
       * @returns {Node}
       * Returns the removed child node.
       * @throws {DOMException}
       * Throws a DOMException if `oldChild` is not a child of `this`.
       * See {@link _removeChild} for more details.
       */
      removeChild: function(e2) {
        return Re(this, e2);
      },
      /**
       * Appends a child node to this node.
       *
       * @param {Node} newChild
       * The child node to be appended to this node.
       * If it already exists in the DOM, it is removed from its original position.
       * @returns {Node}
       * Returns the appended child node.
       * @throws {DOMException}
       * Throws a DOMException if appending the node would result in a DOM tree that is not
       * well-formed, or if `newChild` is not a valid Node.
       * See {@link insertBefore} for more details.
       */
      appendChild: function(e2) {
        return this.insertBefore(e2, null);
      },
      /**
       * Determines whether this node has any child nodes.
       *
       * @returns {boolean}
       * Returns true if this node has any child nodes, and false otherwise.
       */
      hasChildNodes: function() {
        return this.firstChild != null;
      },
      /**
       * Creates a copy of the calling node.
       *
       * @param {boolean} deep
       * If true, the contents of the node are recursively copied.
       * If false, only the node itself (and its attributes, if it is an element) are copied.
       * @returns {Node}
       * Returns the newly created copy of the node.
       * @throws {DOMException}
       * May throw a DOMException if operations within {@link Element#setAttributeNode} or
       * {@link Node#appendChild} (which are potentially invoked in this method) do not meet their
       * specific constraints.
       * @see {@link cloneNode}
       */
      cloneNode: function(e2) {
        return Et(this.ownerDocument || this, this, e2);
      },
      /**
       * Puts the specified node and all of its subtree into a "normalized" form. In a normalized
       * subtree, no text nodes in the subtree are empty and there are no adjacent text nodes.
       *
       * Specifically, this method merges any adjacent text nodes (i.e., nodes for which `nodeType`
       * is `TEXT_NODE`) into a single node with the combined data. It also removes any empty text
       * nodes.
       *
       * This method operates recursively, so it also normalizes any and all descendent nodes within
       * the subtree.
       *
       * @throws {DOMException}
       * May throw a DOMException if operations within removeChild or appendData (which are
       * potentially invoked in this method) do not meet their specific constraints.
       * @since Modified in DOM Level 2
       * @see {@link Node.removeChild}
       * @see {@link CharacterData.appendData}
       */
      normalize: function() {
        for (var e2 = this.firstChild; e2; ) {
          var r3 = e2.nextSibling;
          r3 && r3.nodeType == J2 && e2.nodeType == J2 ? (this.removeChild(r3), e2.appendData(r3.data)) : (e2.normalize(), e2 = r3);
        }
      },
      /**
       * Checks whether the DOM implementation implements a specific feature and its version.
       *
       * @deprecated
       * Since `DOMImplementation.hasFeature` is deprecated and always returns true.
       * @param {string} feature
       * The package name of the feature to test. This is the same name that can be passed to the
       * method `hasFeature` on `DOMImplementation`.
       * @param {string} version
       * This is the version number of the package name to test.
       * @returns {boolean}
       * Returns true in all cases in the current implementation.
       * @since Introduced in DOM Level 2
       * @see {@link DOMImplementation.hasFeature}
       */
      isSupported: function(e2, r3) {
        return this.ownerDocument.implementation.hasFeature(e2, r3);
      },
      /**
       * Look up the prefix associated to the given namespace URI, starting from this node.
       * **The default namespace declarations are ignored by this method.**
       * See Namespace Prefix Lookup for details on the algorithm used by this method.
       *
       * **This behavior is different from the in the specs**:
       * - no node type specific handling
       * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
       *
       * @param {string | null} namespaceURI
       * The namespace URI for which to find the associated prefix.
       * @returns {string | null}
       * The associated prefix, if found; otherwise, null.
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespacePrefix
       * @see https://www.w3.org/TR/DOM-Level-3-Core/namespaces-algorithms.html#lookupNamespacePrefixAlgo
       * @see https://dom.spec.whatwg.org/#dom-node-lookupprefix
       * @see https://github.com/xmldom/xmldom/issues/322
       * @prettierignore
       */
      lookupPrefix: function(e2) {
        for (var r3 = this; r3; ) {
          var i2 = r3._nsMap;
          if (i2) {
            for (var a4 in i2)
              if (s3(i2, a4) && i2[a4] === e2)
                return a4;
          }
          r3 = r3.nodeType == $3 ? r3.ownerDocument : r3.parentNode;
        }
        return null;
      },
      /**
       * This function is used to look up the namespace URI associated with the given prefix,
       * starting from this node.
       *
       * **This behavior is different from the in the specs**:
       * - no node type specific handling
       * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
       *
       * @param {string | null} prefix
       * The prefix for which to find the associated namespace URI.
       * @returns {string | null}
       * The associated namespace URI, if found; otherwise, null.
       * @since DOM Level 3
       * @see https://dom.spec.whatwg.org/#dom-node-lookupnamespaceuri
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespaceURI
       * @prettierignore
       */
      lookupNamespaceURI: function(e2) {
        for (var r3 = this; r3; ) {
          var i2 = r3._nsMap;
          if (i2 && s3(i2, e2))
            return i2[e2];
          r3 = r3.nodeType == $3 ? r3.ownerDocument : r3.parentNode;
        }
        return null;
      },
      /**
       * Determines whether the given namespace URI is the default namespace.
       *
       * The function works by looking up the prefix associated with the given namespace URI. If no
       * prefix is found (i.e., the namespace URI is not registered in the namespace map of this
       * node or any of its ancestors), it returns `true`, implying the namespace URI is considered
       * the default.
       *
       * **This behavior is different from the in the specs**:
       * - no node type specific handling
       * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
       *
       * @param {string | null} namespaceURI
       * The namespace URI to be checked.
       * @returns {boolean}
       * Returns true if the given namespace URI is the default namespace, false otherwise.
       * @since DOM Level 3
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-isDefaultNamespace
       * @see https://dom.spec.whatwg.org/#dom-node-isdefaultnamespace
       * @prettierignore
       */
      isDefaultNamespace: function(e2) {
        var r3 = this.lookupPrefix(e2);
        return r3 == null;
      },
      /**
       * Compares the reference node with a node with regard to their position in the document and
       * according to the document order.
       *
       * @param {Node} other
       * The node to compare the reference node to.
       * @returns {number}
       * Returns how the node is positioned relatively to the reference node according to the
       * bitmask. 0 if reference node and given node are the same.
       * @since DOM Level 3
       * @see https://www.w3.org/TR/2004/REC-DOM-Level-3-Core-20040407/core.html#Node3-compare
       * @see https://dom.spec.whatwg.org/#dom-node-comparedocumentposition
       */
      compareDocumentPosition: function(e2) {
        if (this === e2) return 0;
        var r3 = e2, i2 = this, a4 = null, m4 = null;
        if (r3 instanceof De && (a4 = r3, r3 = a4.ownerElement), i2 instanceof De && (m4 = i2, i2 = m4.ownerElement, a4 && r3 && i2 === r3))
          for (var N4 = 0, z2; z2 = i2.attributes[N4]; N4++) {
            if (z2 === a4)
              return S4.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + S4.DOCUMENT_POSITION_PRECEDING;
            if (z2 === m4)
              return S4.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + S4.DOCUMENT_POSITION_FOLLOWING;
          }
        if (!r3 || !i2 || i2.ownerDocument !== r3.ownerDocument)
          return S4.DOCUMENT_POSITION_DISCONNECTED + S4.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + (O6(i2.ownerDocument) > O6(r3.ownerDocument) ? S4.DOCUMENT_POSITION_FOLLOWING : S4.DOCUMENT_POSITION_PRECEDING);
        if (m4 && r3 === i2)
          return S4.DOCUMENT_POSITION_CONTAINS + S4.DOCUMENT_POSITION_PRECEDING;
        if (a4 && r3 === i2)
          return S4.DOCUMENT_POSITION_CONTAINED_BY + S4.DOCUMENT_POSITION_FOLLOWING;
        for (var se = [], he = r3.parentNode; he; ) {
          if (!m4 && he === i2)
            return S4.DOCUMENT_POSITION_CONTAINED_BY + S4.DOCUMENT_POSITION_FOLLOWING;
          se.push(he), he = he.parentNode;
        }
        se.reverse();
        for (var ve = [], de = i2.parentNode; de; ) {
          if (!a4 && de === r3)
            return S4.DOCUMENT_POSITION_CONTAINS + S4.DOCUMENT_POSITION_PRECEDING;
          ve.push(de), de = de.parentNode;
        }
        ve.reverse();
        var Xe = V2(se, ve);
        for (var xe in Xe.childNodes) {
          var be = Xe.childNodes[xe];
          if (be === i2) return S4.DOCUMENT_POSITION_FOLLOWING;
          if (be === r3) return S4.DOCUMENT_POSITION_PRECEDING;
          if (ve.indexOf(be) >= 0) return S4.DOCUMENT_POSITION_FOLLOWING;
          if (se.indexOf(be) >= 0) return S4.DOCUMENT_POSITION_PRECEDING;
        }
        return 0;
      }
    };
    function Fe(e2) {
      return e2 == "<" && "&lt;" || e2 == ">" && "&gt;" || e2 == "&" && "&amp;" || e2 == '"' && "&quot;" || "&#" + e2.charCodeAt() + ";";
    }
    d6(x5, R5), d6(x5, R5.prototype), d6(S4, R5), d6(S4, R5.prototype);
    function ye(e2, r3) {
      if (r3(e2))
        return true;
      if (e2 = e2.firstChild)
        do
          if (ye(e2, r3))
            return true;
        while (e2 = e2.nextSibling);
    }
    function ce2(e2, r3) {
      Y2(e2);
      var i2 = r3 || {};
      this.ownerDocument = this, this.contentType = i2.contentType || C2.XML_APPLICATION, this.type = c5(this.contentType) ? "html" : "xml";
    }
    function Le(e2, r3, i2) {
      e2 && e2._inc++;
      var a4 = i2.namespaceURI;
      a4 === f7.XMLNS && (r3._nsMap[i2.prefix ? i2.localName : ""] = i2.value);
    }
    function _e(e2, r3, i2, a4) {
      e2 && e2._inc++;
      var m4 = i2.namespaceURI;
      m4 === f7.XMLNS && delete r3._nsMap[i2.prefix ? i2.localName : ""];
    }
    function Ce(e2, r3, i2) {
      if (e2 && e2._inc) {
        e2._inc++;
        var a4 = r3.childNodes;
        if (i2 && !i2.nextSibling)
          a4[a4.length++] = i2;
        else {
          for (var m4 = r3.firstChild, N4 = 0; m4; )
            a4[N4++] = m4, m4 = m4.nextSibling;
          a4.length = N4, delete a4[a4.length];
        }
      }
    }
    function Re(e2, r3) {
      if (e2 !== r3.parentNode)
        throw new p5(p5.NOT_FOUND_ERR, "child's parent is not parent");
      var i2 = r3.previousSibling, a4 = r3.nextSibling;
      return i2 ? i2.nextSibling = a4 : e2.firstChild = a4, a4 ? a4.previousSibling = i2 : e2.lastChild = i2, Ce(e2.ownerDocument, e2), r3.parentNode = null, r3.previousSibling = null, r3.nextSibling = null, r3;
    }
    function Ue(e2) {
      return e2 && (e2.nodeType === R5.DOCUMENT_NODE || e2.nodeType === R5.DOCUMENT_FRAGMENT_NODE || e2.nodeType === R5.ELEMENT_NODE);
    }
    function qe(e2) {
      return e2 && (e2.nodeType === R5.CDATA_SECTION_NODE || e2.nodeType === R5.COMMENT_NODE || e2.nodeType === R5.DOCUMENT_FRAGMENT_NODE || e2.nodeType === R5.DOCUMENT_TYPE_NODE || e2.nodeType === R5.ELEMENT_NODE || e2.nodeType === R5.PROCESSING_INSTRUCTION_NODE || e2.nodeType === R5.TEXT_NODE);
    }
    function Ae(e2) {
      return e2 && e2.nodeType === R5.DOCUMENT_TYPE_NODE;
    }
    function j2(e2) {
      return e2 && e2.nodeType === R5.ELEMENT_NODE;
    }
    function Pe(e2) {
      return e2 && e2.nodeType === R5.TEXT_NODE;
    }
    function ne(e2, r3) {
      var i2 = e2.childNodes || [];
      if (t4(i2, j2) || Ae(r3))
        return false;
      var a4 = t4(i2, Ae);
      return !(r3 && a4 && i2.indexOf(a4) > i2.indexOf(r3));
    }
    function Ge(e2, r3) {
      var i2 = e2.childNodes || [];
      function a4(N4) {
        return j2(N4) && N4 !== r3;
      }
      if (t4(i2, a4))
        return false;
      var m4 = t4(i2, Ae);
      return !(r3 && m4 && i2.indexOf(m4) > i2.indexOf(r3));
    }
    function at(e2, r3, i2) {
      if (!Ue(e2))
        throw new p5(p5.HIERARCHY_REQUEST_ERR, "Unexpected parent node type " + e2.nodeType);
      if (i2 && i2.parentNode !== e2)
        throw new p5(p5.NOT_FOUND_ERR, "child not in parent");
      if (
        // 4. If `node` is not a DocumentFragment, DocumentType, Element, or CharacterData node, then throw a "HierarchyRequestError" DOMException.
        !qe(r3) || // 5. If either `node` is a Text node and `parent` is a document,
        // the sax parser currently adds top level text nodes, this will be fixed in 0.9.0
        // || (node.nodeType === Node.TEXT_NODE && parent.nodeType === Node.DOCUMENT_NODE)
        // or `node` is a doctype and `parent` is not a document, then throw a "HierarchyRequestError" DOMException.
        Ae(r3) && e2.nodeType !== R5.DOCUMENT_NODE
      )
        throw new p5(
          p5.HIERARCHY_REQUEST_ERR,
          "Unexpected node type " + r3.nodeType + " for parent node type " + e2.nodeType
        );
    }
    function Ve(e2, r3, i2) {
      var a4 = e2.childNodes || [], m4 = r3.childNodes || [];
      if (r3.nodeType === R5.DOCUMENT_FRAGMENT_NODE) {
        var N4 = m4.filter(j2);
        if (N4.length > 1 || t4(m4, Pe))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
        if (N4.length === 1 && !ne(e2, i2))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
      }
      if (j2(r3) && !ne(e2, i2))
        throw new p5(p5.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
      if (Ae(r3)) {
        if (t4(a4, Ae))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
        var z2 = t4(a4, j2);
        if (i2 && a4.indexOf(z2) < a4.indexOf(i2))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
        if (!i2 && z2)
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Doctype can not be appended since element is present");
      }
    }
    function ze(e2, r3, i2) {
      var a4 = e2.childNodes || [], m4 = r3.childNodes || [];
      if (r3.nodeType === R5.DOCUMENT_FRAGMENT_NODE) {
        var N4 = m4.filter(j2);
        if (N4.length > 1 || t4(m4, Pe))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
        if (N4.length === 1 && !Ge(e2, i2))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
      }
      if (j2(r3) && !Ge(e2, i2))
        throw new p5(p5.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
      if (Ae(r3)) {
        if (t4(a4, function(he) {
          return Ae(he) && he !== i2;
        }))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
        var z2 = t4(a4, j2);
        if (i2 && a4.indexOf(z2) < a4.indexOf(i2))
          throw new p5(p5.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
      }
    }
    function F4(e2, r3, i2, a4) {
      at(e2, r3, i2), e2.nodeType === R5.DOCUMENT_NODE && (a4 || Ve)(e2, r3, i2);
      var m4 = r3.parentNode;
      if (m4 && m4.removeChild(r3), r3.nodeType === y7) {
        var N4 = r3.firstChild;
        if (N4 == null)
          return r3;
        var z2 = r3.lastChild;
      } else
        N4 = z2 = r3;
      var se = i2 ? i2.previousSibling : e2.lastChild;
      N4.previousSibling = se, z2.nextSibling = i2, se ? se.nextSibling = N4 : e2.firstChild = N4, i2 == null ? e2.lastChild = z2 : i2.previousSibling = z2;
      do
        N4.parentNode = e2;
      while (N4 !== z2 && (N4 = N4.nextSibling));
      return Ce(e2.ownerDocument || e2, e2, r3), r3.nodeType == y7 && (r3.firstChild = r3.lastChild = null), r3;
    }
    ce2.prototype = {
      /**
       * The implementation that created this document.
       *
       * @type DOMImplementation
       * @readonly
       */
      implementation: null,
      nodeName: "#document",
      nodeType: b5,
      /**
       * The DocumentType node of the document.
       *
       * @type DocumentType
       * @readonly
       */
      doctype: null,
      documentElement: null,
      _inc: 1,
      insertBefore: function(e2, r3) {
        if (e2.nodeType === y7) {
          for (var i2 = e2.firstChild; i2; ) {
            var a4 = i2.nextSibling;
            this.insertBefore(i2, r3), i2 = a4;
          }
          return e2;
        }
        return F4(this, e2, r3), e2.ownerDocument = this, this.documentElement === null && e2.nodeType === U5 && (this.documentElement = e2), e2;
      },
      removeChild: function(e2) {
        var r3 = Re(this, e2);
        return r3 === this.documentElement && (this.documentElement = null), r3;
      },
      replaceChild: function(e2, r3) {
        F4(this, e2, r3, ze), e2.ownerDocument = this, r3 && this.removeChild(r3), j2(e2) && (this.documentElement = e2);
      },
      // Introduced in DOM Level 2:
      importNode: function(e2, r3) {
        return Tt(this, e2, r3);
      },
      // Introduced in DOM Level 2:
      getElementById: function(e2) {
        var r3 = null;
        return ye(this.documentElement, function(i2) {
          if (i2.nodeType == U5 && i2.getAttribute("id") == e2)
            return r3 = i2, true;
        }), r3;
      },
      /**
       * Creates a new `Element` that is owned by this `Document`.
       * In HTML Documents `localName` is the lower cased `tagName`,
       * otherwise no transformation is being applied.
       * When `contentType` implies the HTML namespace, it will be set as `namespaceURI`.
       *
       * __This implementation differs from the specification:__ - The provided name is not checked
       * against the `Name` production,
       * so no related error will be thrown.
       * - There is no interface `HTMLElement`, it is always an `Element`.
       * - There is no support for a second argument to indicate using custom elements.
       *
       * @param {string} tagName
       * @returns {Element}
       * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createElement
       * @see https://dom.spec.whatwg.org/#dom-document-createelement
       * @see https://dom.spec.whatwg.org/#concept-create-element
       */
      createElement: function(e2) {
        var r3 = new Q2(g5);
        r3.ownerDocument = this, this.type === "html" && (e2 = e2.toLowerCase()), u3(this.contentType) && (r3.namespaceURI = f7.HTML), r3.nodeName = e2, r3.tagName = e2, r3.localName = e2, r3.childNodes = new T2();
        var i2 = r3.attributes = new P();
        return i2._ownerElement = r3, r3;
      },
      /**
       * @returns {DocumentFragment}
       */
      createDocumentFragment: function() {
        var e2 = new Ye(g5);
        return e2.ownerDocument = this, e2.childNodes = new T2(), e2;
      },
      /**
       * @param {string} data
       * @returns {Text}
       */
      createTextNode: function(e2) {
        var r3 = new He(g5);
        return r3.ownerDocument = this, r3.childNodes = new T2(), r3.appendData(e2), r3;
      },
      /**
       * @param {string} data
       * @returns {Comment}
       */
      createComment: function(e2) {
        var r3 = new Je(g5);
        return r3.ownerDocument = this, r3.childNodes = new T2(), r3.appendData(e2), r3;
      },
      /**
       * @param {string} data
       * @returns {CDATASection}
       */
      createCDATASection: function(e2) {
        var r3 = new Ze(g5);
        return r3.ownerDocument = this, r3.childNodes = new T2(), r3.appendData(e2), r3;
      },
      /**
       * @param {string} target
       * @param {string} data
       * @returns {ProcessingInstruction}
       */
      createProcessingInstruction: function(e2, r3) {
        var i2 = new tt(g5);
        return i2.ownerDocument = this, i2.childNodes = new T2(), i2.nodeName = i2.target = e2, i2.nodeValue = i2.data = r3, i2;
      },
      /**
       * Creates an `Attr` node that is owned by this document.
       * In HTML Documents `localName` is the lower cased `name`,
       * otherwise no transformation is being applied.
       *
       * __This implementation differs from the specification:__ - The provided name is not checked
       * against the `Name` production,
       * so no related error will be thrown.
       *
       * @param {string} name
       * @returns {Attr}
       * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createAttribute
       * @see https://dom.spec.whatwg.org/#dom-document-createattribute
       */
      createAttribute: function(e2) {
        if (!k4.QName_exact.test(e2))
          throw new p5(p5.INVALID_CHARACTER_ERR, 'invalid character in name "' + e2 + '"');
        return this.type === "html" && (e2 = e2.toLowerCase()), this._createAttribute(e2);
      },
      _createAttribute: function(e2) {
        var r3 = new De(g5);
        return r3.ownerDocument = this, r3.childNodes = new T2(), r3.name = e2, r3.nodeName = e2, r3.localName = e2, r3.specified = true, r3;
      },
      /**
       * Creates an EntityReference object.
       * The current implementation does not fill the `childNodes` with those of the corresponding
       * `Entity`
       *
       * @deprecated
       * In DOM Level 4.
       * @param {string} name
       * The name of the entity to reference. No namespace well-formedness checks are performed.
       * @returns {EntityReference}
       * @throws {DOMException}
       * With code `INVALID_CHARACTER_ERR` when `name` is not valid.
       * @throws {DOMException}
       * with code `NOT_SUPPORTED_ERR` when the document is of type `html`
       * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-392B75AE
       */
      createEntityReference: function(e2) {
        if (!k4.Name.test(e2))
          throw new p5(p5.INVALID_CHARACTER_ERR, 'not a valid xml name "' + e2 + '"');
        if (this.type === "html")
          throw new p5("document is an html document", B3.NotSupportedError);
        var r3 = new et2(g5);
        return r3.ownerDocument = this, r3.childNodes = new T2(), r3.nodeName = e2, r3;
      },
      // Introduced in DOM Level 2:
      /**
       * @param {string} namespaceURI
       * @param {string} qualifiedName
       * @returns {Element}
       */
      createElementNS: function(e2, r3) {
        var i2 = me(e2, r3), a4 = new Q2(g5), m4 = a4.attributes = new P();
        return a4.childNodes = new T2(), a4.ownerDocument = this, a4.nodeName = r3, a4.tagName = r3, a4.namespaceURI = i2[0], a4.prefix = i2[1], a4.localName = i2[2], m4._ownerElement = a4, a4;
      },
      // Introduced in DOM Level 2:
      /**
       * @param {string} namespaceURI
       * @param {string} qualifiedName
       * @returns {Attr}
       */
      createAttributeNS: function(e2, r3) {
        var i2 = me(e2, r3), a4 = new De(g5);
        return a4.ownerDocument = this, a4.childNodes = new T2(), a4.nodeName = r3, a4.name = r3, a4.specified = true, a4.namespaceURI = i2[0], a4.prefix = i2[1], a4.localName = i2[2], a4;
      }
    }, _3(ce2, R5);
    function Q2(e2) {
      Y2(e2), this._nsMap = /* @__PURE__ */ Object.create(null);
    }
    Q2.prototype = {
      nodeType: U5,
      /**
       * The attributes of this element.
       *
       * @type {NamedNodeMap | null}
       */
      attributes: null,
      getQualifiedName: function() {
        return this.prefix ? this.prefix + ":" + this.localName : this.localName;
      },
      _isInHTMLDocumentAndNamespace: function() {
        return this.ownerDocument.type === "html" && this.namespaceURI === f7.HTML;
      },
      /**
       * Implementaton of Level2 Core function hasAttributes.
       *
       * @returns {boolean}
       * True if attribute list is not empty.
       * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-NodeHasAttrs
       */
      hasAttributes: function() {
        return !!(this.attributes && this.attributes.length);
      },
      hasAttribute: function(e2) {
        return !!this.getAttributeNode(e2);
      },
      /**
       * Returns element’s first attribute whose qualified name is `name`, and `null`
       * if there is no such attribute.
       *
       * @param {string} name
       * @returns {string | null}
       */
      getAttribute: function(e2) {
        var r3 = this.getAttributeNode(e2);
        return r3 ? r3.value : null;
      },
      getAttributeNode: function(e2) {
        return this._isInHTMLDocumentAndNamespace() && (e2 = e2.toLowerCase()), this.attributes.getNamedItem(e2);
      },
      /**
       * Sets the value of element’s first attribute whose qualified name is qualifiedName to value.
       *
       * @param {string} name
       * @param {string} value
       */
      setAttribute: function(e2, r3) {
        this._isInHTMLDocumentAndNamespace() && (e2 = e2.toLowerCase());
        var i2 = this.getAttributeNode(e2);
        i2 ? i2.value = i2.nodeValue = "" + r3 : (i2 = this.ownerDocument._createAttribute(e2), i2.value = i2.nodeValue = "" + r3, this.setAttributeNode(i2));
      },
      removeAttribute: function(e2) {
        var r3 = this.getAttributeNode(e2);
        r3 && this.removeAttributeNode(r3);
      },
      setAttributeNode: function(e2) {
        return this.attributes.setNamedItem(e2);
      },
      setAttributeNodeNS: function(e2) {
        return this.attributes.setNamedItemNS(e2);
      },
      removeAttributeNode: function(e2) {
        return this.attributes.removeNamedItem(e2.nodeName);
      },
      //get real attribute name,and remove it by removeAttributeNode
      removeAttributeNS: function(e2, r3) {
        var i2 = this.getAttributeNodeNS(e2, r3);
        i2 && this.removeAttributeNode(i2);
      },
      hasAttributeNS: function(e2, r3) {
        return this.getAttributeNodeNS(e2, r3) != null;
      },
      /**
       * Returns element’s attribute whose namespace is `namespaceURI` and local name is
       * `localName`,
       * or `null` if there is no such attribute.
       *
       * @param {string} namespaceURI
       * @param {string} localName
       * @returns {string | null}
       */
      getAttributeNS: function(e2, r3) {
        var i2 = this.getAttributeNodeNS(e2, r3);
        return i2 ? i2.value : null;
      },
      /**
       * Sets the value of element’s attribute whose namespace is `namespaceURI` and local name is
       * `localName` to value.
       *
       * @param {string} namespaceURI
       * @param {string} qualifiedName
       * @param {string} value
       * @see https://dom.spec.whatwg.org/#dom-element-setattributens
       */
      setAttributeNS: function(e2, r3, i2) {
        var a4 = me(e2, r3), m4 = a4[2], N4 = this.getAttributeNodeNS(e2, m4);
        N4 ? N4.value = N4.nodeValue = "" + i2 : (N4 = this.ownerDocument.createAttributeNS(e2, r3), N4.value = N4.nodeValue = "" + i2, this.setAttributeNode(N4));
      },
      getAttributeNodeNS: function(e2, r3) {
        return this.attributes.getNamedItemNS(e2, r3);
      },
      /**
       * Returns a LiveNodeList of all child elements which have **all** of the given class name(s).
       *
       * Returns an empty list if `classNames` is an empty string or only contains HTML white space
       * characters.
       *
       * Warning: This returns a live LiveNodeList.
       * Changes in the DOM will reflect in the array as the changes occur.
       * If an element selected by this array no longer qualifies for the selector,
       * it will automatically be removed. Be aware of this for iteration purposes.
       *
       * @param {string} classNames
       * Is a string representing the class name(s) to match; multiple class names are separated by
       * (ASCII-)whitespace.
       * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByClassName
       * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/getElementsByClassName
       * @see https://dom.spec.whatwg.org/#concept-getelementsbyclassname
       */
      getElementsByClassName: function(e2) {
        var r3 = q3(e2);
        return new w3(this, function(i2) {
          var a4 = [];
          return r3.length > 0 && ye(i2, function(m4) {
            if (m4 !== i2 && m4.nodeType === U5) {
              var N4 = m4.getAttribute("class");
              if (N4) {
                var z2 = e2 === N4;
                if (!z2) {
                  var se = q3(N4);
                  z2 = r3.every(W(se));
                }
                z2 && a4.push(m4);
              }
            }
          }), a4;
        });
      },
      /**
       * Returns a LiveNodeList of elements with the given qualifiedName.
       * Searching for all descendants can be done by passing `*` as `qualifiedName`.
       *
       * All descendants of the specified element are searched, but not the element itself.
       * The returned list is live, which means it updates itself with the DOM tree automatically.
       * Therefore, there is no need to call `Element.getElementsByTagName()`
       * with the same element and arguments repeatedly if the DOM changes in between calls.
       *
       * When called on an HTML element in an HTML document,
       * `getElementsByTagName` lower-cases the argument before searching for it.
       * This is undesirable when trying to match camel-cased SVG elements (such as
       * `<linearGradient>`) in an HTML document.
       * Instead, use `Element.getElementsByTagNameNS()`,
       * which preserves the capitalization of the tag name.
       *
       * `Element.getElementsByTagName` is similar to `Document.getElementsByTagName()`,
       * except that it only searches for elements that are descendants of the specified element.
       *
       * @param {string} qualifiedName
       * @returns {LiveNodeList}
       * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByTagName
       * @see https://dom.spec.whatwg.org/#concept-getelementsbytagname
       */
      getElementsByTagName: function(e2) {
        var r3 = (this.nodeType === b5 ? this : this.ownerDocument).type === "html", i2 = e2.toLowerCase();
        return new w3(this, function(a4) {
          var m4 = [];
          return ye(a4, function(N4) {
            if (!(N4 === a4 || N4.nodeType !== U5))
              if (e2 === "*")
                m4.push(N4);
              else {
                var z2 = N4.getQualifiedName(), se = r3 && N4.namespaceURI === f7.HTML ? i2 : e2;
                z2 === se && m4.push(N4);
              }
          }), m4;
        });
      },
      getElementsByTagNameNS: function(e2, r3) {
        return new w3(this, function(i2) {
          var a4 = [];
          return ye(i2, function(m4) {
            m4 !== i2 && m4.nodeType === U5 && (e2 === "*" || m4.namespaceURI === e2) && (r3 === "*" || m4.localName == r3) && a4.push(m4);
          }), a4;
        });
      }
    }, ce2.prototype.getElementsByClassName = Q2.prototype.getElementsByClassName, ce2.prototype.getElementsByTagName = Q2.prototype.getElementsByTagName, ce2.prototype.getElementsByTagNameNS = Q2.prototype.getElementsByTagNameNS, _3(Q2, R5);
    function De(e2) {
      Y2(e2), this.namespaceURI = null, this.prefix = null, this.ownerElement = null;
    }
    De.prototype.nodeType = $3, _3(De, R5);
    function ge(e2) {
      Y2(e2);
    }
    ge.prototype = {
      data: "",
      substringData: function(e2, r3) {
        return this.data.substring(e2, e2 + r3);
      },
      appendData: function(e2) {
        e2 = this.data + e2, this.nodeValue = this.data = e2, this.length = e2.length;
      },
      insertData: function(e2, r3) {
        this.replaceData(e2, 0, r3);
      },
      deleteData: function(e2, r3) {
        this.replaceData(e2, r3, "");
      },
      replaceData: function(e2, r3, i2) {
        var a4 = this.data.substring(0, e2), m4 = this.data.substring(e2 + r3);
        i2 = a4 + i2 + m4, this.nodeValue = this.data = i2, this.length = i2.length;
      }
    }, _3(ge, R5);
    function He(e2) {
      Y2(e2);
    }
    He.prototype = {
      nodeName: "#text",
      nodeType: J2,
      splitText: function(e2) {
        var r3 = this.data, i2 = r3.substring(e2);
        r3 = r3.substring(0, e2), this.data = this.nodeValue = r3, this.length = r3.length;
        var a4 = this.ownerDocument.createTextNode(i2);
        return this.parentNode && this.parentNode.insertBefore(a4, this.nextSibling), a4;
      }
    }, _3(He, ge);
    function Je(e2) {
      Y2(e2);
    }
    Je.prototype = {
      nodeName: "#comment",
      nodeType: A6
    }, _3(Je, ge);
    function Ze(e2) {
      Y2(e2);
    }
    Ze.prototype = {
      nodeName: "#cdata-section",
      nodeType: Ne
    }, _3(Ze, He);
    function Ke(e2) {
      Y2(e2);
    }
    Ke.prototype.nodeType = v3, _3(Ke, R5);
    function ot(e2) {
      Y2(e2);
    }
    ot.prototype.nodeType = E3, _3(ot, R5);
    function ct(e2) {
      Y2(e2);
    }
    ct.prototype.nodeType = we, _3(ct, R5);
    function et2(e2) {
      Y2(e2);
    }
    et2.prototype.nodeType = Z3, _3(et2, R5);
    function Ye(e2) {
      Y2(e2);
    }
    Ye.prototype.nodeName = "#document-fragment", Ye.prototype.nodeType = y7, _3(Ye, R5);
    function tt(e2) {
      Y2(e2);
    }
    tt.prototype.nodeType = l6, _3(tt, ge);
    function mt() {
    }
    mt.prototype.serializeToString = function(e2, r3) {
      return ee.call(e2, r3);
    }, R5.prototype.toString = ee;
    function ee(e2) {
      var r3 = [], i2 = this.nodeType === b5 && this.documentElement || this, a4 = i2.prefix, m4 = i2.namespaceURI;
      if (m4 && a4 == null) {
        var a4 = i2.lookupPrefix(m4);
        if (a4 == null)
          var N4 = [
            { namespace: m4, prefix: null }
            //{namespace:uri,prefix:''}
          ];
      }
      return Ie(this, r3, e2, N4), r3.join("");
    }
    function le(e2, r3, i2) {
      var a4 = e2.prefix || "", m4 = e2.namespaceURI;
      if (!m4 || a4 === "xml" && m4 === f7.XML || m4 === f7.XMLNS)
        return false;
      for (var N4 = i2.length; N4--; ) {
        var z2 = i2[N4];
        if (z2.prefix === a4)
          return z2.namespace !== m4;
      }
      return true;
    }
    function Me(e2, r3, i2) {
      e2.push(" ", r3, '="', i2.replace(/[<>&"\t\n\r]/g, Fe), '"');
    }
    function Ie(e2, r3, i2, a4) {
      a4 || (a4 = []);
      var m4 = e2.nodeType === b5 ? e2 : e2.ownerDocument, N4 = m4.type === "html";
      if (i2)
        if (e2 = i2(e2), e2) {
          if (typeof e2 == "string") {
            r3.push(e2);
            return;
          }
        } else
          return;
      switch (e2.nodeType) {
        case U5:
          var z2 = e2.attributes, se = z2.length, pe = e2.firstChild, he = e2.tagName, ve = he;
          if (!N4 && !e2.prefix && e2.namespaceURI) {
            for (var de, Xe = 0; Xe < z2.length; Xe++)
              if (z2.item(Xe).name === "xmlns") {
                de = z2.item(Xe).value;
                break;
              }
            if (!de)
              for (var xe = a4.length - 1; xe >= 0; xe--) {
                var be = a4[xe];
                if (be.prefix === "" && be.namespace === e2.namespaceURI) {
                  de = be.namespace;
                  break;
                }
              }
            if (de !== e2.namespaceURI)
              for (var xe = a4.length - 1; xe >= 0; xe--) {
                var be = a4[xe];
                if (be.namespace === e2.namespaceURI) {
                  be.prefix && (ve = be.prefix + ":" + he);
                  break;
                }
              }
          }
          r3.push("<", ve);
          for (var $e = 0; $e < se; $e++) {
            var Se = z2.item($e);
            Se.prefix == "xmlns" ? a4.push({
              prefix: Se.localName,
              namespace: Se.value
            }) : Se.nodeName == "xmlns" && a4.push({ prefix: "", namespace: Se.value });
          }
          for (var $e = 0; $e < se; $e++) {
            var Se = z2.item($e);
            if (le(Se, N4, a4)) {
              var je = Se.prefix || "", lt = Se.namespaceURI;
              Me(r3, je ? "xmlns:" + je : "xmlns", lt), a4.push({ prefix: je, namespace: lt });
            }
            Ie(Se, r3, i2, a4);
          }
          if (he === ve && le(e2, N4, a4)) {
            var je = e2.prefix || "", lt = e2.namespaceURI;
            Me(r3, je ? "xmlns:" + je : "xmlns", lt), a4.push({ prefix: je, namespace: lt });
          }
          var Dt = !pe;
          if (Dt && (N4 || e2.namespaceURI === f7.HTML) && (Dt = h4(he)), Dt)
            r3.push("/>");
          else {
            if (r3.push(">"), N4 && o5(he))
              for (; pe; )
                pe.data ? r3.push(pe.data) : Ie(pe, r3, i2, a4.slice()), pe = pe.nextSibling;
            else
              for (; pe; )
                Ie(pe, r3, i2, a4.slice()), pe = pe.nextSibling;
            r3.push("</", ve, ">");
          }
          return;
        case b5:
        case y7:
          for (var pe = e2.firstChild; pe; )
            Ie(pe, r3, i2, a4.slice()), pe = pe.nextSibling;
          return;
        case $3:
          return Me(r3, e2.name, e2.value);
        case J2:
          return r3.push(e2.data.replace(/[<&>]/g, Fe));
        case Ne:
          return r3.push(k4.CDATA_START, e2.data, k4.CDATA_END);
        case A6:
          return r3.push(k4.COMMENT_START, e2.data, k4.COMMENT_END);
        case v3:
          var bt = e2.publicId, rt = e2.systemId;
          r3.push(k4.DOCTYPE_DECL_START, " ", e2.name), bt ? (r3.push(" ", k4.PUBLIC, " ", bt), rt && rt !== "." && r3.push(" ", rt)) : rt && rt !== "." && r3.push(" ", k4.SYSTEM, " ", rt), e2.internalSubset && r3.push(" [", e2.internalSubset, "]"), r3.push(">");
          return;
        case l6:
          return r3.push("<?", e2.target, " ", e2.data, "?>");
        case Z3:
          return r3.push("&", e2.nodeName, ";");
        //case ENTITY_NODE:
        //case NOTATION_NODE:
        default:
          r3.push("??", e2.nodeName);
      }
    }
    function Tt(e2, r3, i2) {
      var a4;
      switch (r3.nodeType) {
        case U5:
          a4 = r3.cloneNode(false), a4.ownerDocument = e2;
        //var attrs = node2.attributes;
        //var len = attrs.length;
        //for(var i=0;i<len;i++){
        //node2.setAttributeNodeNS(importNode(doc,attrs.item(i),deep));
        //}
        case y7:
          break;
        case $3:
          i2 = true;
          break;
      }
      if (a4 || (a4 = r3.cloneNode(false)), a4.ownerDocument = e2, a4.parentNode = null, i2)
        for (var m4 = r3.firstChild; m4; )
          a4.appendChild(Tt(e2, m4, i2)), m4 = m4.nextSibling;
      return a4;
    }
    function Et(e2, r3, i2) {
      var a4 = new r3.constructor(g5);
      for (var m4 in r3)
        if (s3(r3, m4)) {
          var N4 = r3[m4];
          typeof N4 != "object" && N4 != a4[m4] && (a4[m4] = N4);
        }
      switch (r3.childNodes && (a4.childNodes = new T2()), a4.ownerDocument = e2, a4.nodeType) {
        case U5:
          var z2 = r3.attributes, se = a4.attributes = new P(), he = z2.length;
          se._ownerElement = a4;
          for (var ve = 0; ve < he; ve++)
            a4.setAttributeNode(Et(e2, z2.item(ve), true));
          break;
        case $3:
          i2 = true;
      }
      if (i2)
        for (var de = r3.firstChild; de; )
          a4.appendChild(Et(e2, de, i2)), de = de.nextSibling;
      return a4;
    }
    function Ct(e2, r3, i2) {
      e2[r3] = i2;
    }
    try {
      if (Object.defineProperty) {
        let e2 = function(r3) {
          switch (r3.nodeType) {
            case U5:
            case y7:
              var i2 = [];
              for (r3 = r3.firstChild; r3; )
                r3.nodeType !== 7 && r3.nodeType !== 8 && i2.push(e2(r3)), r3 = r3.nextSibling;
              return i2.join("");
            default:
              return r3.nodeValue;
          }
        };
        Object.defineProperty(w3.prototype, "length", {
          get: function() {
            return I4(this), this.$$length;
          }
        }), Object.defineProperty(R5.prototype, "textContent", {
          get: function() {
            return e2(this);
          },
          set: function(r3) {
            switch (this.nodeType) {
              case U5:
              case y7:
                for (; this.firstChild; )
                  this.removeChild(this.firstChild);
                (r3 || String(r3)) && this.appendChild(this.ownerDocument.createTextNode(r3));
                break;
              default:
                this.data = r3, this.value = r3, this.nodeValue = r3;
            }
          }
        }), Ct = function(r3, i2, a4) {
          r3["$$" + i2] = a4;
        };
      }
    } catch {
    }
    return K2._updateLiveList = I4, K2.Attr = De, K2.CDATASection = Ze, K2.CharacterData = ge, K2.Comment = Je, K2.Document = ce2, K2.DocumentFragment = Ye, K2.DocumentType = Ke, K2.DOMImplementation = ie, K2.Element = Q2, K2.Entity = ct, K2.EntityReference = et2, K2.LiveNodeList = w3, K2.NamedNodeMap = P, K2.Node = R5, K2.NodeList = T2, K2.Notation = ot, K2.Text = He, K2.ProcessingInstruction = tt, K2.XMLSerializer = mt, K2;
  }
  function Dr() {
    return Ot || (Ot = 1, (function(n2) {
      var t4 = st().freeze;
      n2.XML_ENTITIES = t4({
        amp: "&",
        apos: "'",
        gt: ">",
        lt: "<",
        quot: '"'
      }), n2.HTML_ENTITIES = t4({
        Aacute: "\xC1",
        aacute: "\xE1",
        Abreve: "\u0102",
        abreve: "\u0103",
        ac: "\u223E",
        acd: "\u223F",
        acE: "\u223E\u0333",
        Acirc: "\xC2",
        acirc: "\xE2",
        acute: "\xB4",
        Acy: "\u0410",
        acy: "\u0430",
        AElig: "\xC6",
        aelig: "\xE6",
        af: "\u2061",
        Afr: "\u{1D504}",
        afr: "\u{1D51E}",
        Agrave: "\xC0",
        agrave: "\xE0",
        alefsym: "\u2135",
        aleph: "\u2135",
        Alpha: "\u0391",
        alpha: "\u03B1",
        Amacr: "\u0100",
        amacr: "\u0101",
        amalg: "\u2A3F",
        AMP: "&",
        amp: "&",
        And: "\u2A53",
        and: "\u2227",
        andand: "\u2A55",
        andd: "\u2A5C",
        andslope: "\u2A58",
        andv: "\u2A5A",
        ang: "\u2220",
        ange: "\u29A4",
        angle: "\u2220",
        angmsd: "\u2221",
        angmsdaa: "\u29A8",
        angmsdab: "\u29A9",
        angmsdac: "\u29AA",
        angmsdad: "\u29AB",
        angmsdae: "\u29AC",
        angmsdaf: "\u29AD",
        angmsdag: "\u29AE",
        angmsdah: "\u29AF",
        angrt: "\u221F",
        angrtvb: "\u22BE",
        angrtvbd: "\u299D",
        angsph: "\u2222",
        angst: "\xC5",
        angzarr: "\u237C",
        Aogon: "\u0104",
        aogon: "\u0105",
        Aopf: "\u{1D538}",
        aopf: "\u{1D552}",
        ap: "\u2248",
        apacir: "\u2A6F",
        apE: "\u2A70",
        ape: "\u224A",
        apid: "\u224B",
        apos: "'",
        ApplyFunction: "\u2061",
        approx: "\u2248",
        approxeq: "\u224A",
        Aring: "\xC5",
        aring: "\xE5",
        Ascr: "\u{1D49C}",
        ascr: "\u{1D4B6}",
        Assign: "\u2254",
        ast: "*",
        asymp: "\u2248",
        asympeq: "\u224D",
        Atilde: "\xC3",
        atilde: "\xE3",
        Auml: "\xC4",
        auml: "\xE4",
        awconint: "\u2233",
        awint: "\u2A11",
        backcong: "\u224C",
        backepsilon: "\u03F6",
        backprime: "\u2035",
        backsim: "\u223D",
        backsimeq: "\u22CD",
        Backslash: "\u2216",
        Barv: "\u2AE7",
        barvee: "\u22BD",
        Barwed: "\u2306",
        barwed: "\u2305",
        barwedge: "\u2305",
        bbrk: "\u23B5",
        bbrktbrk: "\u23B6",
        bcong: "\u224C",
        Bcy: "\u0411",
        bcy: "\u0431",
        bdquo: "\u201E",
        becaus: "\u2235",
        Because: "\u2235",
        because: "\u2235",
        bemptyv: "\u29B0",
        bepsi: "\u03F6",
        bernou: "\u212C",
        Bernoullis: "\u212C",
        Beta: "\u0392",
        beta: "\u03B2",
        beth: "\u2136",
        between: "\u226C",
        Bfr: "\u{1D505}",
        bfr: "\u{1D51F}",
        bigcap: "\u22C2",
        bigcirc: "\u25EF",
        bigcup: "\u22C3",
        bigodot: "\u2A00",
        bigoplus: "\u2A01",
        bigotimes: "\u2A02",
        bigsqcup: "\u2A06",
        bigstar: "\u2605",
        bigtriangledown: "\u25BD",
        bigtriangleup: "\u25B3",
        biguplus: "\u2A04",
        bigvee: "\u22C1",
        bigwedge: "\u22C0",
        bkarow: "\u290D",
        blacklozenge: "\u29EB",
        blacksquare: "\u25AA",
        blacktriangle: "\u25B4",
        blacktriangledown: "\u25BE",
        blacktriangleleft: "\u25C2",
        blacktriangleright: "\u25B8",
        blank: "\u2423",
        blk12: "\u2592",
        blk14: "\u2591",
        blk34: "\u2593",
        block: "\u2588",
        bne: "=\u20E5",
        bnequiv: "\u2261\u20E5",
        bNot: "\u2AED",
        bnot: "\u2310",
        Bopf: "\u{1D539}",
        bopf: "\u{1D553}",
        bot: "\u22A5",
        bottom: "\u22A5",
        bowtie: "\u22C8",
        boxbox: "\u29C9",
        boxDL: "\u2557",
        boxDl: "\u2556",
        boxdL: "\u2555",
        boxdl: "\u2510",
        boxDR: "\u2554",
        boxDr: "\u2553",
        boxdR: "\u2552",
        boxdr: "\u250C",
        boxH: "\u2550",
        boxh: "\u2500",
        boxHD: "\u2566",
        boxHd: "\u2564",
        boxhD: "\u2565",
        boxhd: "\u252C",
        boxHU: "\u2569",
        boxHu: "\u2567",
        boxhU: "\u2568",
        boxhu: "\u2534",
        boxminus: "\u229F",
        boxplus: "\u229E",
        boxtimes: "\u22A0",
        boxUL: "\u255D",
        boxUl: "\u255C",
        boxuL: "\u255B",
        boxul: "\u2518",
        boxUR: "\u255A",
        boxUr: "\u2559",
        boxuR: "\u2558",
        boxur: "\u2514",
        boxV: "\u2551",
        boxv: "\u2502",
        boxVH: "\u256C",
        boxVh: "\u256B",
        boxvH: "\u256A",
        boxvh: "\u253C",
        boxVL: "\u2563",
        boxVl: "\u2562",
        boxvL: "\u2561",
        boxvl: "\u2524",
        boxVR: "\u2560",
        boxVr: "\u255F",
        boxvR: "\u255E",
        boxvr: "\u251C",
        bprime: "\u2035",
        Breve: "\u02D8",
        breve: "\u02D8",
        brvbar: "\xA6",
        Bscr: "\u212C",
        bscr: "\u{1D4B7}",
        bsemi: "\u204F",
        bsim: "\u223D",
        bsime: "\u22CD",
        bsol: "\\",
        bsolb: "\u29C5",
        bsolhsub: "\u27C8",
        bull: "\u2022",
        bullet: "\u2022",
        bump: "\u224E",
        bumpE: "\u2AAE",
        bumpe: "\u224F",
        Bumpeq: "\u224E",
        bumpeq: "\u224F",
        Cacute: "\u0106",
        cacute: "\u0107",
        Cap: "\u22D2",
        cap: "\u2229",
        capand: "\u2A44",
        capbrcup: "\u2A49",
        capcap: "\u2A4B",
        capcup: "\u2A47",
        capdot: "\u2A40",
        CapitalDifferentialD: "\u2145",
        caps: "\u2229\uFE00",
        caret: "\u2041",
        caron: "\u02C7",
        Cayleys: "\u212D",
        ccaps: "\u2A4D",
        Ccaron: "\u010C",
        ccaron: "\u010D",
        Ccedil: "\xC7",
        ccedil: "\xE7",
        Ccirc: "\u0108",
        ccirc: "\u0109",
        Cconint: "\u2230",
        ccups: "\u2A4C",
        ccupssm: "\u2A50",
        Cdot: "\u010A",
        cdot: "\u010B",
        cedil: "\xB8",
        Cedilla: "\xB8",
        cemptyv: "\u29B2",
        cent: "\xA2",
        CenterDot: "\xB7",
        centerdot: "\xB7",
        Cfr: "\u212D",
        cfr: "\u{1D520}",
        CHcy: "\u0427",
        chcy: "\u0447",
        check: "\u2713",
        checkmark: "\u2713",
        Chi: "\u03A7",
        chi: "\u03C7",
        cir: "\u25CB",
        circ: "\u02C6",
        circeq: "\u2257",
        circlearrowleft: "\u21BA",
        circlearrowright: "\u21BB",
        circledast: "\u229B",
        circledcirc: "\u229A",
        circleddash: "\u229D",
        CircleDot: "\u2299",
        circledR: "\xAE",
        circledS: "\u24C8",
        CircleMinus: "\u2296",
        CirclePlus: "\u2295",
        CircleTimes: "\u2297",
        cirE: "\u29C3",
        cire: "\u2257",
        cirfnint: "\u2A10",
        cirmid: "\u2AEF",
        cirscir: "\u29C2",
        ClockwiseContourIntegral: "\u2232",
        CloseCurlyDoubleQuote: "\u201D",
        CloseCurlyQuote: "\u2019",
        clubs: "\u2663",
        clubsuit: "\u2663",
        Colon: "\u2237",
        colon: ":",
        Colone: "\u2A74",
        colone: "\u2254",
        coloneq: "\u2254",
        comma: ",",
        commat: "@",
        comp: "\u2201",
        compfn: "\u2218",
        complement: "\u2201",
        complexes: "\u2102",
        cong: "\u2245",
        congdot: "\u2A6D",
        Congruent: "\u2261",
        Conint: "\u222F",
        conint: "\u222E",
        ContourIntegral: "\u222E",
        Copf: "\u2102",
        copf: "\u{1D554}",
        coprod: "\u2210",
        Coproduct: "\u2210",
        COPY: "\xA9",
        copy: "\xA9",
        copysr: "\u2117",
        CounterClockwiseContourIntegral: "\u2233",
        crarr: "\u21B5",
        Cross: "\u2A2F",
        cross: "\u2717",
        Cscr: "\u{1D49E}",
        cscr: "\u{1D4B8}",
        csub: "\u2ACF",
        csube: "\u2AD1",
        csup: "\u2AD0",
        csupe: "\u2AD2",
        ctdot: "\u22EF",
        cudarrl: "\u2938",
        cudarrr: "\u2935",
        cuepr: "\u22DE",
        cuesc: "\u22DF",
        cularr: "\u21B6",
        cularrp: "\u293D",
        Cup: "\u22D3",
        cup: "\u222A",
        cupbrcap: "\u2A48",
        CupCap: "\u224D",
        cupcap: "\u2A46",
        cupcup: "\u2A4A",
        cupdot: "\u228D",
        cupor: "\u2A45",
        cups: "\u222A\uFE00",
        curarr: "\u21B7",
        curarrm: "\u293C",
        curlyeqprec: "\u22DE",
        curlyeqsucc: "\u22DF",
        curlyvee: "\u22CE",
        curlywedge: "\u22CF",
        curren: "\xA4",
        curvearrowleft: "\u21B6",
        curvearrowright: "\u21B7",
        cuvee: "\u22CE",
        cuwed: "\u22CF",
        cwconint: "\u2232",
        cwint: "\u2231",
        cylcty: "\u232D",
        Dagger: "\u2021",
        dagger: "\u2020",
        daleth: "\u2138",
        Darr: "\u21A1",
        dArr: "\u21D3",
        darr: "\u2193",
        dash: "\u2010",
        Dashv: "\u2AE4",
        dashv: "\u22A3",
        dbkarow: "\u290F",
        dblac: "\u02DD",
        Dcaron: "\u010E",
        dcaron: "\u010F",
        Dcy: "\u0414",
        dcy: "\u0434",
        DD: "\u2145",
        dd: "\u2146",
        ddagger: "\u2021",
        ddarr: "\u21CA",
        DDotrahd: "\u2911",
        ddotseq: "\u2A77",
        deg: "\xB0",
        Del: "\u2207",
        Delta: "\u0394",
        delta: "\u03B4",
        demptyv: "\u29B1",
        dfisht: "\u297F",
        Dfr: "\u{1D507}",
        dfr: "\u{1D521}",
        dHar: "\u2965",
        dharl: "\u21C3",
        dharr: "\u21C2",
        DiacriticalAcute: "\xB4",
        DiacriticalDot: "\u02D9",
        DiacriticalDoubleAcute: "\u02DD",
        DiacriticalGrave: "`",
        DiacriticalTilde: "\u02DC",
        diam: "\u22C4",
        Diamond: "\u22C4",
        diamond: "\u22C4",
        diamondsuit: "\u2666",
        diams: "\u2666",
        die: "\xA8",
        DifferentialD: "\u2146",
        digamma: "\u03DD",
        disin: "\u22F2",
        div: "\xF7",
        divide: "\xF7",
        divideontimes: "\u22C7",
        divonx: "\u22C7",
        DJcy: "\u0402",
        djcy: "\u0452",
        dlcorn: "\u231E",
        dlcrop: "\u230D",
        dollar: "$",
        Dopf: "\u{1D53B}",
        dopf: "\u{1D555}",
        Dot: "\xA8",
        dot: "\u02D9",
        DotDot: "\u20DC",
        doteq: "\u2250",
        doteqdot: "\u2251",
        DotEqual: "\u2250",
        dotminus: "\u2238",
        dotplus: "\u2214",
        dotsquare: "\u22A1",
        doublebarwedge: "\u2306",
        DoubleContourIntegral: "\u222F",
        DoubleDot: "\xA8",
        DoubleDownArrow: "\u21D3",
        DoubleLeftArrow: "\u21D0",
        DoubleLeftRightArrow: "\u21D4",
        DoubleLeftTee: "\u2AE4",
        DoubleLongLeftArrow: "\u27F8",
        DoubleLongLeftRightArrow: "\u27FA",
        DoubleLongRightArrow: "\u27F9",
        DoubleRightArrow: "\u21D2",
        DoubleRightTee: "\u22A8",
        DoubleUpArrow: "\u21D1",
        DoubleUpDownArrow: "\u21D5",
        DoubleVerticalBar: "\u2225",
        DownArrow: "\u2193",
        Downarrow: "\u21D3",
        downarrow: "\u2193",
        DownArrowBar: "\u2913",
        DownArrowUpArrow: "\u21F5",
        DownBreve: "\u0311",
        downdownarrows: "\u21CA",
        downharpoonleft: "\u21C3",
        downharpoonright: "\u21C2",
        DownLeftRightVector: "\u2950",
        DownLeftTeeVector: "\u295E",
        DownLeftVector: "\u21BD",
        DownLeftVectorBar: "\u2956",
        DownRightTeeVector: "\u295F",
        DownRightVector: "\u21C1",
        DownRightVectorBar: "\u2957",
        DownTee: "\u22A4",
        DownTeeArrow: "\u21A7",
        drbkarow: "\u2910",
        drcorn: "\u231F",
        drcrop: "\u230C",
        Dscr: "\u{1D49F}",
        dscr: "\u{1D4B9}",
        DScy: "\u0405",
        dscy: "\u0455",
        dsol: "\u29F6",
        Dstrok: "\u0110",
        dstrok: "\u0111",
        dtdot: "\u22F1",
        dtri: "\u25BF",
        dtrif: "\u25BE",
        duarr: "\u21F5",
        duhar: "\u296F",
        dwangle: "\u29A6",
        DZcy: "\u040F",
        dzcy: "\u045F",
        dzigrarr: "\u27FF",
        Eacute: "\xC9",
        eacute: "\xE9",
        easter: "\u2A6E",
        Ecaron: "\u011A",
        ecaron: "\u011B",
        ecir: "\u2256",
        Ecirc: "\xCA",
        ecirc: "\xEA",
        ecolon: "\u2255",
        Ecy: "\u042D",
        ecy: "\u044D",
        eDDot: "\u2A77",
        Edot: "\u0116",
        eDot: "\u2251",
        edot: "\u0117",
        ee: "\u2147",
        efDot: "\u2252",
        Efr: "\u{1D508}",
        efr: "\u{1D522}",
        eg: "\u2A9A",
        Egrave: "\xC8",
        egrave: "\xE8",
        egs: "\u2A96",
        egsdot: "\u2A98",
        el: "\u2A99",
        Element: "\u2208",
        elinters: "\u23E7",
        ell: "\u2113",
        els: "\u2A95",
        elsdot: "\u2A97",
        Emacr: "\u0112",
        emacr: "\u0113",
        empty: "\u2205",
        emptyset: "\u2205",
        EmptySmallSquare: "\u25FB",
        emptyv: "\u2205",
        EmptyVerySmallSquare: "\u25AB",
        emsp: "\u2003",
        emsp13: "\u2004",
        emsp14: "\u2005",
        ENG: "\u014A",
        eng: "\u014B",
        ensp: "\u2002",
        Eogon: "\u0118",
        eogon: "\u0119",
        Eopf: "\u{1D53C}",
        eopf: "\u{1D556}",
        epar: "\u22D5",
        eparsl: "\u29E3",
        eplus: "\u2A71",
        epsi: "\u03B5",
        Epsilon: "\u0395",
        epsilon: "\u03B5",
        epsiv: "\u03F5",
        eqcirc: "\u2256",
        eqcolon: "\u2255",
        eqsim: "\u2242",
        eqslantgtr: "\u2A96",
        eqslantless: "\u2A95",
        Equal: "\u2A75",
        equals: "=",
        EqualTilde: "\u2242",
        equest: "\u225F",
        Equilibrium: "\u21CC",
        equiv: "\u2261",
        equivDD: "\u2A78",
        eqvparsl: "\u29E5",
        erarr: "\u2971",
        erDot: "\u2253",
        Escr: "\u2130",
        escr: "\u212F",
        esdot: "\u2250",
        Esim: "\u2A73",
        esim: "\u2242",
        Eta: "\u0397",
        eta: "\u03B7",
        ETH: "\xD0",
        eth: "\xF0",
        Euml: "\xCB",
        euml: "\xEB",
        euro: "\u20AC",
        excl: "!",
        exist: "\u2203",
        Exists: "\u2203",
        expectation: "\u2130",
        ExponentialE: "\u2147",
        exponentiale: "\u2147",
        fallingdotseq: "\u2252",
        Fcy: "\u0424",
        fcy: "\u0444",
        female: "\u2640",
        ffilig: "\uFB03",
        fflig: "\uFB00",
        ffllig: "\uFB04",
        Ffr: "\u{1D509}",
        ffr: "\u{1D523}",
        filig: "\uFB01",
        FilledSmallSquare: "\u25FC",
        FilledVerySmallSquare: "\u25AA",
        fjlig: "fj",
        flat: "\u266D",
        fllig: "\uFB02",
        fltns: "\u25B1",
        fnof: "\u0192",
        Fopf: "\u{1D53D}",
        fopf: "\u{1D557}",
        ForAll: "\u2200",
        forall: "\u2200",
        fork: "\u22D4",
        forkv: "\u2AD9",
        Fouriertrf: "\u2131",
        fpartint: "\u2A0D",
        frac12: "\xBD",
        frac13: "\u2153",
        frac14: "\xBC",
        frac15: "\u2155",
        frac16: "\u2159",
        frac18: "\u215B",
        frac23: "\u2154",
        frac25: "\u2156",
        frac34: "\xBE",
        frac35: "\u2157",
        frac38: "\u215C",
        frac45: "\u2158",
        frac56: "\u215A",
        frac58: "\u215D",
        frac78: "\u215E",
        frasl: "\u2044",
        frown: "\u2322",
        Fscr: "\u2131",
        fscr: "\u{1D4BB}",
        gacute: "\u01F5",
        Gamma: "\u0393",
        gamma: "\u03B3",
        Gammad: "\u03DC",
        gammad: "\u03DD",
        gap: "\u2A86",
        Gbreve: "\u011E",
        gbreve: "\u011F",
        Gcedil: "\u0122",
        Gcirc: "\u011C",
        gcirc: "\u011D",
        Gcy: "\u0413",
        gcy: "\u0433",
        Gdot: "\u0120",
        gdot: "\u0121",
        gE: "\u2267",
        ge: "\u2265",
        gEl: "\u2A8C",
        gel: "\u22DB",
        geq: "\u2265",
        geqq: "\u2267",
        geqslant: "\u2A7E",
        ges: "\u2A7E",
        gescc: "\u2AA9",
        gesdot: "\u2A80",
        gesdoto: "\u2A82",
        gesdotol: "\u2A84",
        gesl: "\u22DB\uFE00",
        gesles: "\u2A94",
        Gfr: "\u{1D50A}",
        gfr: "\u{1D524}",
        Gg: "\u22D9",
        gg: "\u226B",
        ggg: "\u22D9",
        gimel: "\u2137",
        GJcy: "\u0403",
        gjcy: "\u0453",
        gl: "\u2277",
        gla: "\u2AA5",
        glE: "\u2A92",
        glj: "\u2AA4",
        gnap: "\u2A8A",
        gnapprox: "\u2A8A",
        gnE: "\u2269",
        gne: "\u2A88",
        gneq: "\u2A88",
        gneqq: "\u2269",
        gnsim: "\u22E7",
        Gopf: "\u{1D53E}",
        gopf: "\u{1D558}",
        grave: "`",
        GreaterEqual: "\u2265",
        GreaterEqualLess: "\u22DB",
        GreaterFullEqual: "\u2267",
        GreaterGreater: "\u2AA2",
        GreaterLess: "\u2277",
        GreaterSlantEqual: "\u2A7E",
        GreaterTilde: "\u2273",
        Gscr: "\u{1D4A2}",
        gscr: "\u210A",
        gsim: "\u2273",
        gsime: "\u2A8E",
        gsiml: "\u2A90",
        Gt: "\u226B",
        GT: ">",
        gt: ">",
        gtcc: "\u2AA7",
        gtcir: "\u2A7A",
        gtdot: "\u22D7",
        gtlPar: "\u2995",
        gtquest: "\u2A7C",
        gtrapprox: "\u2A86",
        gtrarr: "\u2978",
        gtrdot: "\u22D7",
        gtreqless: "\u22DB",
        gtreqqless: "\u2A8C",
        gtrless: "\u2277",
        gtrsim: "\u2273",
        gvertneqq: "\u2269\uFE00",
        gvnE: "\u2269\uFE00",
        Hacek: "\u02C7",
        hairsp: "\u200A",
        half: "\xBD",
        hamilt: "\u210B",
        HARDcy: "\u042A",
        hardcy: "\u044A",
        hArr: "\u21D4",
        harr: "\u2194",
        harrcir: "\u2948",
        harrw: "\u21AD",
        Hat: "^",
        hbar: "\u210F",
        Hcirc: "\u0124",
        hcirc: "\u0125",
        hearts: "\u2665",
        heartsuit: "\u2665",
        hellip: "\u2026",
        hercon: "\u22B9",
        Hfr: "\u210C",
        hfr: "\u{1D525}",
        HilbertSpace: "\u210B",
        hksearow: "\u2925",
        hkswarow: "\u2926",
        hoarr: "\u21FF",
        homtht: "\u223B",
        hookleftarrow: "\u21A9",
        hookrightarrow: "\u21AA",
        Hopf: "\u210D",
        hopf: "\u{1D559}",
        horbar: "\u2015",
        HorizontalLine: "\u2500",
        Hscr: "\u210B",
        hscr: "\u{1D4BD}",
        hslash: "\u210F",
        Hstrok: "\u0126",
        hstrok: "\u0127",
        HumpDownHump: "\u224E",
        HumpEqual: "\u224F",
        hybull: "\u2043",
        hyphen: "\u2010",
        Iacute: "\xCD",
        iacute: "\xED",
        ic: "\u2063",
        Icirc: "\xCE",
        icirc: "\xEE",
        Icy: "\u0418",
        icy: "\u0438",
        Idot: "\u0130",
        IEcy: "\u0415",
        iecy: "\u0435",
        iexcl: "\xA1",
        iff: "\u21D4",
        Ifr: "\u2111",
        ifr: "\u{1D526}",
        Igrave: "\xCC",
        igrave: "\xEC",
        ii: "\u2148",
        iiiint: "\u2A0C",
        iiint: "\u222D",
        iinfin: "\u29DC",
        iiota: "\u2129",
        IJlig: "\u0132",
        ijlig: "\u0133",
        Im: "\u2111",
        Imacr: "\u012A",
        imacr: "\u012B",
        image: "\u2111",
        ImaginaryI: "\u2148",
        imagline: "\u2110",
        imagpart: "\u2111",
        imath: "\u0131",
        imof: "\u22B7",
        imped: "\u01B5",
        Implies: "\u21D2",
        in: "\u2208",
        incare: "\u2105",
        infin: "\u221E",
        infintie: "\u29DD",
        inodot: "\u0131",
        Int: "\u222C",
        int: "\u222B",
        intcal: "\u22BA",
        integers: "\u2124",
        Integral: "\u222B",
        intercal: "\u22BA",
        Intersection: "\u22C2",
        intlarhk: "\u2A17",
        intprod: "\u2A3C",
        InvisibleComma: "\u2063",
        InvisibleTimes: "\u2062",
        IOcy: "\u0401",
        iocy: "\u0451",
        Iogon: "\u012E",
        iogon: "\u012F",
        Iopf: "\u{1D540}",
        iopf: "\u{1D55A}",
        Iota: "\u0399",
        iota: "\u03B9",
        iprod: "\u2A3C",
        iquest: "\xBF",
        Iscr: "\u2110",
        iscr: "\u{1D4BE}",
        isin: "\u2208",
        isindot: "\u22F5",
        isinE: "\u22F9",
        isins: "\u22F4",
        isinsv: "\u22F3",
        isinv: "\u2208",
        it: "\u2062",
        Itilde: "\u0128",
        itilde: "\u0129",
        Iukcy: "\u0406",
        iukcy: "\u0456",
        Iuml: "\xCF",
        iuml: "\xEF",
        Jcirc: "\u0134",
        jcirc: "\u0135",
        Jcy: "\u0419",
        jcy: "\u0439",
        Jfr: "\u{1D50D}",
        jfr: "\u{1D527}",
        jmath: "\u0237",
        Jopf: "\u{1D541}",
        jopf: "\u{1D55B}",
        Jscr: "\u{1D4A5}",
        jscr: "\u{1D4BF}",
        Jsercy: "\u0408",
        jsercy: "\u0458",
        Jukcy: "\u0404",
        jukcy: "\u0454",
        Kappa: "\u039A",
        kappa: "\u03BA",
        kappav: "\u03F0",
        Kcedil: "\u0136",
        kcedil: "\u0137",
        Kcy: "\u041A",
        kcy: "\u043A",
        Kfr: "\u{1D50E}",
        kfr: "\u{1D528}",
        kgreen: "\u0138",
        KHcy: "\u0425",
        khcy: "\u0445",
        KJcy: "\u040C",
        kjcy: "\u045C",
        Kopf: "\u{1D542}",
        kopf: "\u{1D55C}",
        Kscr: "\u{1D4A6}",
        kscr: "\u{1D4C0}",
        lAarr: "\u21DA",
        Lacute: "\u0139",
        lacute: "\u013A",
        laemptyv: "\u29B4",
        lagran: "\u2112",
        Lambda: "\u039B",
        lambda: "\u03BB",
        Lang: "\u27EA",
        lang: "\u27E8",
        langd: "\u2991",
        langle: "\u27E8",
        lap: "\u2A85",
        Laplacetrf: "\u2112",
        laquo: "\xAB",
        Larr: "\u219E",
        lArr: "\u21D0",
        larr: "\u2190",
        larrb: "\u21E4",
        larrbfs: "\u291F",
        larrfs: "\u291D",
        larrhk: "\u21A9",
        larrlp: "\u21AB",
        larrpl: "\u2939",
        larrsim: "\u2973",
        larrtl: "\u21A2",
        lat: "\u2AAB",
        lAtail: "\u291B",
        latail: "\u2919",
        late: "\u2AAD",
        lates: "\u2AAD\uFE00",
        lBarr: "\u290E",
        lbarr: "\u290C",
        lbbrk: "\u2772",
        lbrace: "{",
        lbrack: "[",
        lbrke: "\u298B",
        lbrksld: "\u298F",
        lbrkslu: "\u298D",
        Lcaron: "\u013D",
        lcaron: "\u013E",
        Lcedil: "\u013B",
        lcedil: "\u013C",
        lceil: "\u2308",
        lcub: "{",
        Lcy: "\u041B",
        lcy: "\u043B",
        ldca: "\u2936",
        ldquo: "\u201C",
        ldquor: "\u201E",
        ldrdhar: "\u2967",
        ldrushar: "\u294B",
        ldsh: "\u21B2",
        lE: "\u2266",
        le: "\u2264",
        LeftAngleBracket: "\u27E8",
        LeftArrow: "\u2190",
        Leftarrow: "\u21D0",
        leftarrow: "\u2190",
        LeftArrowBar: "\u21E4",
        LeftArrowRightArrow: "\u21C6",
        leftarrowtail: "\u21A2",
        LeftCeiling: "\u2308",
        LeftDoubleBracket: "\u27E6",
        LeftDownTeeVector: "\u2961",
        LeftDownVector: "\u21C3",
        LeftDownVectorBar: "\u2959",
        LeftFloor: "\u230A",
        leftharpoondown: "\u21BD",
        leftharpoonup: "\u21BC",
        leftleftarrows: "\u21C7",
        LeftRightArrow: "\u2194",
        Leftrightarrow: "\u21D4",
        leftrightarrow: "\u2194",
        leftrightarrows: "\u21C6",
        leftrightharpoons: "\u21CB",
        leftrightsquigarrow: "\u21AD",
        LeftRightVector: "\u294E",
        LeftTee: "\u22A3",
        LeftTeeArrow: "\u21A4",
        LeftTeeVector: "\u295A",
        leftthreetimes: "\u22CB",
        LeftTriangle: "\u22B2",
        LeftTriangleBar: "\u29CF",
        LeftTriangleEqual: "\u22B4",
        LeftUpDownVector: "\u2951",
        LeftUpTeeVector: "\u2960",
        LeftUpVector: "\u21BF",
        LeftUpVectorBar: "\u2958",
        LeftVector: "\u21BC",
        LeftVectorBar: "\u2952",
        lEg: "\u2A8B",
        leg: "\u22DA",
        leq: "\u2264",
        leqq: "\u2266",
        leqslant: "\u2A7D",
        les: "\u2A7D",
        lescc: "\u2AA8",
        lesdot: "\u2A7F",
        lesdoto: "\u2A81",
        lesdotor: "\u2A83",
        lesg: "\u22DA\uFE00",
        lesges: "\u2A93",
        lessapprox: "\u2A85",
        lessdot: "\u22D6",
        lesseqgtr: "\u22DA",
        lesseqqgtr: "\u2A8B",
        LessEqualGreater: "\u22DA",
        LessFullEqual: "\u2266",
        LessGreater: "\u2276",
        lessgtr: "\u2276",
        LessLess: "\u2AA1",
        lesssim: "\u2272",
        LessSlantEqual: "\u2A7D",
        LessTilde: "\u2272",
        lfisht: "\u297C",
        lfloor: "\u230A",
        Lfr: "\u{1D50F}",
        lfr: "\u{1D529}",
        lg: "\u2276",
        lgE: "\u2A91",
        lHar: "\u2962",
        lhard: "\u21BD",
        lharu: "\u21BC",
        lharul: "\u296A",
        lhblk: "\u2584",
        LJcy: "\u0409",
        ljcy: "\u0459",
        Ll: "\u22D8",
        ll: "\u226A",
        llarr: "\u21C7",
        llcorner: "\u231E",
        Lleftarrow: "\u21DA",
        llhard: "\u296B",
        lltri: "\u25FA",
        Lmidot: "\u013F",
        lmidot: "\u0140",
        lmoust: "\u23B0",
        lmoustache: "\u23B0",
        lnap: "\u2A89",
        lnapprox: "\u2A89",
        lnE: "\u2268",
        lne: "\u2A87",
        lneq: "\u2A87",
        lneqq: "\u2268",
        lnsim: "\u22E6",
        loang: "\u27EC",
        loarr: "\u21FD",
        lobrk: "\u27E6",
        LongLeftArrow: "\u27F5",
        Longleftarrow: "\u27F8",
        longleftarrow: "\u27F5",
        LongLeftRightArrow: "\u27F7",
        Longleftrightarrow: "\u27FA",
        longleftrightarrow: "\u27F7",
        longmapsto: "\u27FC",
        LongRightArrow: "\u27F6",
        Longrightarrow: "\u27F9",
        longrightarrow: "\u27F6",
        looparrowleft: "\u21AB",
        looparrowright: "\u21AC",
        lopar: "\u2985",
        Lopf: "\u{1D543}",
        lopf: "\u{1D55D}",
        loplus: "\u2A2D",
        lotimes: "\u2A34",
        lowast: "\u2217",
        lowbar: "_",
        LowerLeftArrow: "\u2199",
        LowerRightArrow: "\u2198",
        loz: "\u25CA",
        lozenge: "\u25CA",
        lozf: "\u29EB",
        lpar: "(",
        lparlt: "\u2993",
        lrarr: "\u21C6",
        lrcorner: "\u231F",
        lrhar: "\u21CB",
        lrhard: "\u296D",
        lrm: "\u200E",
        lrtri: "\u22BF",
        lsaquo: "\u2039",
        Lscr: "\u2112",
        lscr: "\u{1D4C1}",
        Lsh: "\u21B0",
        lsh: "\u21B0",
        lsim: "\u2272",
        lsime: "\u2A8D",
        lsimg: "\u2A8F",
        lsqb: "[",
        lsquo: "\u2018",
        lsquor: "\u201A",
        Lstrok: "\u0141",
        lstrok: "\u0142",
        Lt: "\u226A",
        LT: "<",
        lt: "<",
        ltcc: "\u2AA6",
        ltcir: "\u2A79",
        ltdot: "\u22D6",
        lthree: "\u22CB",
        ltimes: "\u22C9",
        ltlarr: "\u2976",
        ltquest: "\u2A7B",
        ltri: "\u25C3",
        ltrie: "\u22B4",
        ltrif: "\u25C2",
        ltrPar: "\u2996",
        lurdshar: "\u294A",
        luruhar: "\u2966",
        lvertneqq: "\u2268\uFE00",
        lvnE: "\u2268\uFE00",
        macr: "\xAF",
        male: "\u2642",
        malt: "\u2720",
        maltese: "\u2720",
        Map: "\u2905",
        map: "\u21A6",
        mapsto: "\u21A6",
        mapstodown: "\u21A7",
        mapstoleft: "\u21A4",
        mapstoup: "\u21A5",
        marker: "\u25AE",
        mcomma: "\u2A29",
        Mcy: "\u041C",
        mcy: "\u043C",
        mdash: "\u2014",
        mDDot: "\u223A",
        measuredangle: "\u2221",
        MediumSpace: "\u205F",
        Mellintrf: "\u2133",
        Mfr: "\u{1D510}",
        mfr: "\u{1D52A}",
        mho: "\u2127",
        micro: "\xB5",
        mid: "\u2223",
        midast: "*",
        midcir: "\u2AF0",
        middot: "\xB7",
        minus: "\u2212",
        minusb: "\u229F",
        minusd: "\u2238",
        minusdu: "\u2A2A",
        MinusPlus: "\u2213",
        mlcp: "\u2ADB",
        mldr: "\u2026",
        mnplus: "\u2213",
        models: "\u22A7",
        Mopf: "\u{1D544}",
        mopf: "\u{1D55E}",
        mp: "\u2213",
        Mscr: "\u2133",
        mscr: "\u{1D4C2}",
        mstpos: "\u223E",
        Mu: "\u039C",
        mu: "\u03BC",
        multimap: "\u22B8",
        mumap: "\u22B8",
        nabla: "\u2207",
        Nacute: "\u0143",
        nacute: "\u0144",
        nang: "\u2220\u20D2",
        nap: "\u2249",
        napE: "\u2A70\u0338",
        napid: "\u224B\u0338",
        napos: "\u0149",
        napprox: "\u2249",
        natur: "\u266E",
        natural: "\u266E",
        naturals: "\u2115",
        nbsp: "\xA0",
        nbump: "\u224E\u0338",
        nbumpe: "\u224F\u0338",
        ncap: "\u2A43",
        Ncaron: "\u0147",
        ncaron: "\u0148",
        Ncedil: "\u0145",
        ncedil: "\u0146",
        ncong: "\u2247",
        ncongdot: "\u2A6D\u0338",
        ncup: "\u2A42",
        Ncy: "\u041D",
        ncy: "\u043D",
        ndash: "\u2013",
        ne: "\u2260",
        nearhk: "\u2924",
        neArr: "\u21D7",
        nearr: "\u2197",
        nearrow: "\u2197",
        nedot: "\u2250\u0338",
        NegativeMediumSpace: "\u200B",
        NegativeThickSpace: "\u200B",
        NegativeThinSpace: "\u200B",
        NegativeVeryThinSpace: "\u200B",
        nequiv: "\u2262",
        nesear: "\u2928",
        nesim: "\u2242\u0338",
        NestedGreaterGreater: "\u226B",
        NestedLessLess: "\u226A",
        NewLine: `
`,
        nexist: "\u2204",
        nexists: "\u2204",
        Nfr: "\u{1D511}",
        nfr: "\u{1D52B}",
        ngE: "\u2267\u0338",
        nge: "\u2271",
        ngeq: "\u2271",
        ngeqq: "\u2267\u0338",
        ngeqslant: "\u2A7E\u0338",
        nges: "\u2A7E\u0338",
        nGg: "\u22D9\u0338",
        ngsim: "\u2275",
        nGt: "\u226B\u20D2",
        ngt: "\u226F",
        ngtr: "\u226F",
        nGtv: "\u226B\u0338",
        nhArr: "\u21CE",
        nharr: "\u21AE",
        nhpar: "\u2AF2",
        ni: "\u220B",
        nis: "\u22FC",
        nisd: "\u22FA",
        niv: "\u220B",
        NJcy: "\u040A",
        njcy: "\u045A",
        nlArr: "\u21CD",
        nlarr: "\u219A",
        nldr: "\u2025",
        nlE: "\u2266\u0338",
        nle: "\u2270",
        nLeftarrow: "\u21CD",
        nleftarrow: "\u219A",
        nLeftrightarrow: "\u21CE",
        nleftrightarrow: "\u21AE",
        nleq: "\u2270",
        nleqq: "\u2266\u0338",
        nleqslant: "\u2A7D\u0338",
        nles: "\u2A7D\u0338",
        nless: "\u226E",
        nLl: "\u22D8\u0338",
        nlsim: "\u2274",
        nLt: "\u226A\u20D2",
        nlt: "\u226E",
        nltri: "\u22EA",
        nltrie: "\u22EC",
        nLtv: "\u226A\u0338",
        nmid: "\u2224",
        NoBreak: "\u2060",
        NonBreakingSpace: "\xA0",
        Nopf: "\u2115",
        nopf: "\u{1D55F}",
        Not: "\u2AEC",
        not: "\xAC",
        NotCongruent: "\u2262",
        NotCupCap: "\u226D",
        NotDoubleVerticalBar: "\u2226",
        NotElement: "\u2209",
        NotEqual: "\u2260",
        NotEqualTilde: "\u2242\u0338",
        NotExists: "\u2204",
        NotGreater: "\u226F",
        NotGreaterEqual: "\u2271",
        NotGreaterFullEqual: "\u2267\u0338",
        NotGreaterGreater: "\u226B\u0338",
        NotGreaterLess: "\u2279",
        NotGreaterSlantEqual: "\u2A7E\u0338",
        NotGreaterTilde: "\u2275",
        NotHumpDownHump: "\u224E\u0338",
        NotHumpEqual: "\u224F\u0338",
        notin: "\u2209",
        notindot: "\u22F5\u0338",
        notinE: "\u22F9\u0338",
        notinva: "\u2209",
        notinvb: "\u22F7",
        notinvc: "\u22F6",
        NotLeftTriangle: "\u22EA",
        NotLeftTriangleBar: "\u29CF\u0338",
        NotLeftTriangleEqual: "\u22EC",
        NotLess: "\u226E",
        NotLessEqual: "\u2270",
        NotLessGreater: "\u2278",
        NotLessLess: "\u226A\u0338",
        NotLessSlantEqual: "\u2A7D\u0338",
        NotLessTilde: "\u2274",
        NotNestedGreaterGreater: "\u2AA2\u0338",
        NotNestedLessLess: "\u2AA1\u0338",
        notni: "\u220C",
        notniva: "\u220C",
        notnivb: "\u22FE",
        notnivc: "\u22FD",
        NotPrecedes: "\u2280",
        NotPrecedesEqual: "\u2AAF\u0338",
        NotPrecedesSlantEqual: "\u22E0",
        NotReverseElement: "\u220C",
        NotRightTriangle: "\u22EB",
        NotRightTriangleBar: "\u29D0\u0338",
        NotRightTriangleEqual: "\u22ED",
        NotSquareSubset: "\u228F\u0338",
        NotSquareSubsetEqual: "\u22E2",
        NotSquareSuperset: "\u2290\u0338",
        NotSquareSupersetEqual: "\u22E3",
        NotSubset: "\u2282\u20D2",
        NotSubsetEqual: "\u2288",
        NotSucceeds: "\u2281",
        NotSucceedsEqual: "\u2AB0\u0338",
        NotSucceedsSlantEqual: "\u22E1",
        NotSucceedsTilde: "\u227F\u0338",
        NotSuperset: "\u2283\u20D2",
        NotSupersetEqual: "\u2289",
        NotTilde: "\u2241",
        NotTildeEqual: "\u2244",
        NotTildeFullEqual: "\u2247",
        NotTildeTilde: "\u2249",
        NotVerticalBar: "\u2224",
        npar: "\u2226",
        nparallel: "\u2226",
        nparsl: "\u2AFD\u20E5",
        npart: "\u2202\u0338",
        npolint: "\u2A14",
        npr: "\u2280",
        nprcue: "\u22E0",
        npre: "\u2AAF\u0338",
        nprec: "\u2280",
        npreceq: "\u2AAF\u0338",
        nrArr: "\u21CF",
        nrarr: "\u219B",
        nrarrc: "\u2933\u0338",
        nrarrw: "\u219D\u0338",
        nRightarrow: "\u21CF",
        nrightarrow: "\u219B",
        nrtri: "\u22EB",
        nrtrie: "\u22ED",
        nsc: "\u2281",
        nsccue: "\u22E1",
        nsce: "\u2AB0\u0338",
        Nscr: "\u{1D4A9}",
        nscr: "\u{1D4C3}",
        nshortmid: "\u2224",
        nshortparallel: "\u2226",
        nsim: "\u2241",
        nsime: "\u2244",
        nsimeq: "\u2244",
        nsmid: "\u2224",
        nspar: "\u2226",
        nsqsube: "\u22E2",
        nsqsupe: "\u22E3",
        nsub: "\u2284",
        nsubE: "\u2AC5\u0338",
        nsube: "\u2288",
        nsubset: "\u2282\u20D2",
        nsubseteq: "\u2288",
        nsubseteqq: "\u2AC5\u0338",
        nsucc: "\u2281",
        nsucceq: "\u2AB0\u0338",
        nsup: "\u2285",
        nsupE: "\u2AC6\u0338",
        nsupe: "\u2289",
        nsupset: "\u2283\u20D2",
        nsupseteq: "\u2289",
        nsupseteqq: "\u2AC6\u0338",
        ntgl: "\u2279",
        Ntilde: "\xD1",
        ntilde: "\xF1",
        ntlg: "\u2278",
        ntriangleleft: "\u22EA",
        ntrianglelefteq: "\u22EC",
        ntriangleright: "\u22EB",
        ntrianglerighteq: "\u22ED",
        Nu: "\u039D",
        nu: "\u03BD",
        num: "#",
        numero: "\u2116",
        numsp: "\u2007",
        nvap: "\u224D\u20D2",
        nVDash: "\u22AF",
        nVdash: "\u22AE",
        nvDash: "\u22AD",
        nvdash: "\u22AC",
        nvge: "\u2265\u20D2",
        nvgt: ">\u20D2",
        nvHarr: "\u2904",
        nvinfin: "\u29DE",
        nvlArr: "\u2902",
        nvle: "\u2264\u20D2",
        nvlt: "<\u20D2",
        nvltrie: "\u22B4\u20D2",
        nvrArr: "\u2903",
        nvrtrie: "\u22B5\u20D2",
        nvsim: "\u223C\u20D2",
        nwarhk: "\u2923",
        nwArr: "\u21D6",
        nwarr: "\u2196",
        nwarrow: "\u2196",
        nwnear: "\u2927",
        Oacute: "\xD3",
        oacute: "\xF3",
        oast: "\u229B",
        ocir: "\u229A",
        Ocirc: "\xD4",
        ocirc: "\xF4",
        Ocy: "\u041E",
        ocy: "\u043E",
        odash: "\u229D",
        Odblac: "\u0150",
        odblac: "\u0151",
        odiv: "\u2A38",
        odot: "\u2299",
        odsold: "\u29BC",
        OElig: "\u0152",
        oelig: "\u0153",
        ofcir: "\u29BF",
        Ofr: "\u{1D512}",
        ofr: "\u{1D52C}",
        ogon: "\u02DB",
        Ograve: "\xD2",
        ograve: "\xF2",
        ogt: "\u29C1",
        ohbar: "\u29B5",
        ohm: "\u03A9",
        oint: "\u222E",
        olarr: "\u21BA",
        olcir: "\u29BE",
        olcross: "\u29BB",
        oline: "\u203E",
        olt: "\u29C0",
        Omacr: "\u014C",
        omacr: "\u014D",
        Omega: "\u03A9",
        omega: "\u03C9",
        Omicron: "\u039F",
        omicron: "\u03BF",
        omid: "\u29B6",
        ominus: "\u2296",
        Oopf: "\u{1D546}",
        oopf: "\u{1D560}",
        opar: "\u29B7",
        OpenCurlyDoubleQuote: "\u201C",
        OpenCurlyQuote: "\u2018",
        operp: "\u29B9",
        oplus: "\u2295",
        Or: "\u2A54",
        or: "\u2228",
        orarr: "\u21BB",
        ord: "\u2A5D",
        order: "\u2134",
        orderof: "\u2134",
        ordf: "\xAA",
        ordm: "\xBA",
        origof: "\u22B6",
        oror: "\u2A56",
        orslope: "\u2A57",
        orv: "\u2A5B",
        oS: "\u24C8",
        Oscr: "\u{1D4AA}",
        oscr: "\u2134",
        Oslash: "\xD8",
        oslash: "\xF8",
        osol: "\u2298",
        Otilde: "\xD5",
        otilde: "\xF5",
        Otimes: "\u2A37",
        otimes: "\u2297",
        otimesas: "\u2A36",
        Ouml: "\xD6",
        ouml: "\xF6",
        ovbar: "\u233D",
        OverBar: "\u203E",
        OverBrace: "\u23DE",
        OverBracket: "\u23B4",
        OverParenthesis: "\u23DC",
        par: "\u2225",
        para: "\xB6",
        parallel: "\u2225",
        parsim: "\u2AF3",
        parsl: "\u2AFD",
        part: "\u2202",
        PartialD: "\u2202",
        Pcy: "\u041F",
        pcy: "\u043F",
        percnt: "%",
        period: ".",
        permil: "\u2030",
        perp: "\u22A5",
        pertenk: "\u2031",
        Pfr: "\u{1D513}",
        pfr: "\u{1D52D}",
        Phi: "\u03A6",
        phi: "\u03C6",
        phiv: "\u03D5",
        phmmat: "\u2133",
        phone: "\u260E",
        Pi: "\u03A0",
        pi: "\u03C0",
        pitchfork: "\u22D4",
        piv: "\u03D6",
        planck: "\u210F",
        planckh: "\u210E",
        plankv: "\u210F",
        plus: "+",
        plusacir: "\u2A23",
        plusb: "\u229E",
        pluscir: "\u2A22",
        plusdo: "\u2214",
        plusdu: "\u2A25",
        pluse: "\u2A72",
        PlusMinus: "\xB1",
        plusmn: "\xB1",
        plussim: "\u2A26",
        plustwo: "\u2A27",
        pm: "\xB1",
        Poincareplane: "\u210C",
        pointint: "\u2A15",
        Popf: "\u2119",
        popf: "\u{1D561}",
        pound: "\xA3",
        Pr: "\u2ABB",
        pr: "\u227A",
        prap: "\u2AB7",
        prcue: "\u227C",
        prE: "\u2AB3",
        pre: "\u2AAF",
        prec: "\u227A",
        precapprox: "\u2AB7",
        preccurlyeq: "\u227C",
        Precedes: "\u227A",
        PrecedesEqual: "\u2AAF",
        PrecedesSlantEqual: "\u227C",
        PrecedesTilde: "\u227E",
        preceq: "\u2AAF",
        precnapprox: "\u2AB9",
        precneqq: "\u2AB5",
        precnsim: "\u22E8",
        precsim: "\u227E",
        Prime: "\u2033",
        prime: "\u2032",
        primes: "\u2119",
        prnap: "\u2AB9",
        prnE: "\u2AB5",
        prnsim: "\u22E8",
        prod: "\u220F",
        Product: "\u220F",
        profalar: "\u232E",
        profline: "\u2312",
        profsurf: "\u2313",
        prop: "\u221D",
        Proportion: "\u2237",
        Proportional: "\u221D",
        propto: "\u221D",
        prsim: "\u227E",
        prurel: "\u22B0",
        Pscr: "\u{1D4AB}",
        pscr: "\u{1D4C5}",
        Psi: "\u03A8",
        psi: "\u03C8",
        puncsp: "\u2008",
        Qfr: "\u{1D514}",
        qfr: "\u{1D52E}",
        qint: "\u2A0C",
        Qopf: "\u211A",
        qopf: "\u{1D562}",
        qprime: "\u2057",
        Qscr: "\u{1D4AC}",
        qscr: "\u{1D4C6}",
        quaternions: "\u210D",
        quatint: "\u2A16",
        quest: "?",
        questeq: "\u225F",
        QUOT: '"',
        quot: '"',
        rAarr: "\u21DB",
        race: "\u223D\u0331",
        Racute: "\u0154",
        racute: "\u0155",
        radic: "\u221A",
        raemptyv: "\u29B3",
        Rang: "\u27EB",
        rang: "\u27E9",
        rangd: "\u2992",
        range: "\u29A5",
        rangle: "\u27E9",
        raquo: "\xBB",
        Rarr: "\u21A0",
        rArr: "\u21D2",
        rarr: "\u2192",
        rarrap: "\u2975",
        rarrb: "\u21E5",
        rarrbfs: "\u2920",
        rarrc: "\u2933",
        rarrfs: "\u291E",
        rarrhk: "\u21AA",
        rarrlp: "\u21AC",
        rarrpl: "\u2945",
        rarrsim: "\u2974",
        Rarrtl: "\u2916",
        rarrtl: "\u21A3",
        rarrw: "\u219D",
        rAtail: "\u291C",
        ratail: "\u291A",
        ratio: "\u2236",
        rationals: "\u211A",
        RBarr: "\u2910",
        rBarr: "\u290F",
        rbarr: "\u290D",
        rbbrk: "\u2773",
        rbrace: "}",
        rbrack: "]",
        rbrke: "\u298C",
        rbrksld: "\u298E",
        rbrkslu: "\u2990",
        Rcaron: "\u0158",
        rcaron: "\u0159",
        Rcedil: "\u0156",
        rcedil: "\u0157",
        rceil: "\u2309",
        rcub: "}",
        Rcy: "\u0420",
        rcy: "\u0440",
        rdca: "\u2937",
        rdldhar: "\u2969",
        rdquo: "\u201D",
        rdquor: "\u201D",
        rdsh: "\u21B3",
        Re: "\u211C",
        real: "\u211C",
        realine: "\u211B",
        realpart: "\u211C",
        reals: "\u211D",
        rect: "\u25AD",
        REG: "\xAE",
        reg: "\xAE",
        ReverseElement: "\u220B",
        ReverseEquilibrium: "\u21CB",
        ReverseUpEquilibrium: "\u296F",
        rfisht: "\u297D",
        rfloor: "\u230B",
        Rfr: "\u211C",
        rfr: "\u{1D52F}",
        rHar: "\u2964",
        rhard: "\u21C1",
        rharu: "\u21C0",
        rharul: "\u296C",
        Rho: "\u03A1",
        rho: "\u03C1",
        rhov: "\u03F1",
        RightAngleBracket: "\u27E9",
        RightArrow: "\u2192",
        Rightarrow: "\u21D2",
        rightarrow: "\u2192",
        RightArrowBar: "\u21E5",
        RightArrowLeftArrow: "\u21C4",
        rightarrowtail: "\u21A3",
        RightCeiling: "\u2309",
        RightDoubleBracket: "\u27E7",
        RightDownTeeVector: "\u295D",
        RightDownVector: "\u21C2",
        RightDownVectorBar: "\u2955",
        RightFloor: "\u230B",
        rightharpoondown: "\u21C1",
        rightharpoonup: "\u21C0",
        rightleftarrows: "\u21C4",
        rightleftharpoons: "\u21CC",
        rightrightarrows: "\u21C9",
        rightsquigarrow: "\u219D",
        RightTee: "\u22A2",
        RightTeeArrow: "\u21A6",
        RightTeeVector: "\u295B",
        rightthreetimes: "\u22CC",
        RightTriangle: "\u22B3",
        RightTriangleBar: "\u29D0",
        RightTriangleEqual: "\u22B5",
        RightUpDownVector: "\u294F",
        RightUpTeeVector: "\u295C",
        RightUpVector: "\u21BE",
        RightUpVectorBar: "\u2954",
        RightVector: "\u21C0",
        RightVectorBar: "\u2953",
        ring: "\u02DA",
        risingdotseq: "\u2253",
        rlarr: "\u21C4",
        rlhar: "\u21CC",
        rlm: "\u200F",
        rmoust: "\u23B1",
        rmoustache: "\u23B1",
        rnmid: "\u2AEE",
        roang: "\u27ED",
        roarr: "\u21FE",
        robrk: "\u27E7",
        ropar: "\u2986",
        Ropf: "\u211D",
        ropf: "\u{1D563}",
        roplus: "\u2A2E",
        rotimes: "\u2A35",
        RoundImplies: "\u2970",
        rpar: ")",
        rpargt: "\u2994",
        rppolint: "\u2A12",
        rrarr: "\u21C9",
        Rrightarrow: "\u21DB",
        rsaquo: "\u203A",
        Rscr: "\u211B",
        rscr: "\u{1D4C7}",
        Rsh: "\u21B1",
        rsh: "\u21B1",
        rsqb: "]",
        rsquo: "\u2019",
        rsquor: "\u2019",
        rthree: "\u22CC",
        rtimes: "\u22CA",
        rtri: "\u25B9",
        rtrie: "\u22B5",
        rtrif: "\u25B8",
        rtriltri: "\u29CE",
        RuleDelayed: "\u29F4",
        ruluhar: "\u2968",
        rx: "\u211E",
        Sacute: "\u015A",
        sacute: "\u015B",
        sbquo: "\u201A",
        Sc: "\u2ABC",
        sc: "\u227B",
        scap: "\u2AB8",
        Scaron: "\u0160",
        scaron: "\u0161",
        sccue: "\u227D",
        scE: "\u2AB4",
        sce: "\u2AB0",
        Scedil: "\u015E",
        scedil: "\u015F",
        Scirc: "\u015C",
        scirc: "\u015D",
        scnap: "\u2ABA",
        scnE: "\u2AB6",
        scnsim: "\u22E9",
        scpolint: "\u2A13",
        scsim: "\u227F",
        Scy: "\u0421",
        scy: "\u0441",
        sdot: "\u22C5",
        sdotb: "\u22A1",
        sdote: "\u2A66",
        searhk: "\u2925",
        seArr: "\u21D8",
        searr: "\u2198",
        searrow: "\u2198",
        sect: "\xA7",
        semi: ";",
        seswar: "\u2929",
        setminus: "\u2216",
        setmn: "\u2216",
        sext: "\u2736",
        Sfr: "\u{1D516}",
        sfr: "\u{1D530}",
        sfrown: "\u2322",
        sharp: "\u266F",
        SHCHcy: "\u0429",
        shchcy: "\u0449",
        SHcy: "\u0428",
        shcy: "\u0448",
        ShortDownArrow: "\u2193",
        ShortLeftArrow: "\u2190",
        shortmid: "\u2223",
        shortparallel: "\u2225",
        ShortRightArrow: "\u2192",
        ShortUpArrow: "\u2191",
        shy: "\xAD",
        Sigma: "\u03A3",
        sigma: "\u03C3",
        sigmaf: "\u03C2",
        sigmav: "\u03C2",
        sim: "\u223C",
        simdot: "\u2A6A",
        sime: "\u2243",
        simeq: "\u2243",
        simg: "\u2A9E",
        simgE: "\u2AA0",
        siml: "\u2A9D",
        simlE: "\u2A9F",
        simne: "\u2246",
        simplus: "\u2A24",
        simrarr: "\u2972",
        slarr: "\u2190",
        SmallCircle: "\u2218",
        smallsetminus: "\u2216",
        smashp: "\u2A33",
        smeparsl: "\u29E4",
        smid: "\u2223",
        smile: "\u2323",
        smt: "\u2AAA",
        smte: "\u2AAC",
        smtes: "\u2AAC\uFE00",
        SOFTcy: "\u042C",
        softcy: "\u044C",
        sol: "/",
        solb: "\u29C4",
        solbar: "\u233F",
        Sopf: "\u{1D54A}",
        sopf: "\u{1D564}",
        spades: "\u2660",
        spadesuit: "\u2660",
        spar: "\u2225",
        sqcap: "\u2293",
        sqcaps: "\u2293\uFE00",
        sqcup: "\u2294",
        sqcups: "\u2294\uFE00",
        Sqrt: "\u221A",
        sqsub: "\u228F",
        sqsube: "\u2291",
        sqsubset: "\u228F",
        sqsubseteq: "\u2291",
        sqsup: "\u2290",
        sqsupe: "\u2292",
        sqsupset: "\u2290",
        sqsupseteq: "\u2292",
        squ: "\u25A1",
        Square: "\u25A1",
        square: "\u25A1",
        SquareIntersection: "\u2293",
        SquareSubset: "\u228F",
        SquareSubsetEqual: "\u2291",
        SquareSuperset: "\u2290",
        SquareSupersetEqual: "\u2292",
        SquareUnion: "\u2294",
        squarf: "\u25AA",
        squf: "\u25AA",
        srarr: "\u2192",
        Sscr: "\u{1D4AE}",
        sscr: "\u{1D4C8}",
        ssetmn: "\u2216",
        ssmile: "\u2323",
        sstarf: "\u22C6",
        Star: "\u22C6",
        star: "\u2606",
        starf: "\u2605",
        straightepsilon: "\u03F5",
        straightphi: "\u03D5",
        strns: "\xAF",
        Sub: "\u22D0",
        sub: "\u2282",
        subdot: "\u2ABD",
        subE: "\u2AC5",
        sube: "\u2286",
        subedot: "\u2AC3",
        submult: "\u2AC1",
        subnE: "\u2ACB",
        subne: "\u228A",
        subplus: "\u2ABF",
        subrarr: "\u2979",
        Subset: "\u22D0",
        subset: "\u2282",
        subseteq: "\u2286",
        subseteqq: "\u2AC5",
        SubsetEqual: "\u2286",
        subsetneq: "\u228A",
        subsetneqq: "\u2ACB",
        subsim: "\u2AC7",
        subsub: "\u2AD5",
        subsup: "\u2AD3",
        succ: "\u227B",
        succapprox: "\u2AB8",
        succcurlyeq: "\u227D",
        Succeeds: "\u227B",
        SucceedsEqual: "\u2AB0",
        SucceedsSlantEqual: "\u227D",
        SucceedsTilde: "\u227F",
        succeq: "\u2AB0",
        succnapprox: "\u2ABA",
        succneqq: "\u2AB6",
        succnsim: "\u22E9",
        succsim: "\u227F",
        SuchThat: "\u220B",
        Sum: "\u2211",
        sum: "\u2211",
        sung: "\u266A",
        Sup: "\u22D1",
        sup: "\u2283",
        sup1: "\xB9",
        sup2: "\xB2",
        sup3: "\xB3",
        supdot: "\u2ABE",
        supdsub: "\u2AD8",
        supE: "\u2AC6",
        supe: "\u2287",
        supedot: "\u2AC4",
        Superset: "\u2283",
        SupersetEqual: "\u2287",
        suphsol: "\u27C9",
        suphsub: "\u2AD7",
        suplarr: "\u297B",
        supmult: "\u2AC2",
        supnE: "\u2ACC",
        supne: "\u228B",
        supplus: "\u2AC0",
        Supset: "\u22D1",
        supset: "\u2283",
        supseteq: "\u2287",
        supseteqq: "\u2AC6",
        supsetneq: "\u228B",
        supsetneqq: "\u2ACC",
        supsim: "\u2AC8",
        supsub: "\u2AD4",
        supsup: "\u2AD6",
        swarhk: "\u2926",
        swArr: "\u21D9",
        swarr: "\u2199",
        swarrow: "\u2199",
        swnwar: "\u292A",
        szlig: "\xDF",
        Tab: "	",
        target: "\u2316",
        Tau: "\u03A4",
        tau: "\u03C4",
        tbrk: "\u23B4",
        Tcaron: "\u0164",
        tcaron: "\u0165",
        Tcedil: "\u0162",
        tcedil: "\u0163",
        Tcy: "\u0422",
        tcy: "\u0442",
        tdot: "\u20DB",
        telrec: "\u2315",
        Tfr: "\u{1D517}",
        tfr: "\u{1D531}",
        there4: "\u2234",
        Therefore: "\u2234",
        therefore: "\u2234",
        Theta: "\u0398",
        theta: "\u03B8",
        thetasym: "\u03D1",
        thetav: "\u03D1",
        thickapprox: "\u2248",
        thicksim: "\u223C",
        ThickSpace: "\u205F\u200A",
        thinsp: "\u2009",
        ThinSpace: "\u2009",
        thkap: "\u2248",
        thksim: "\u223C",
        THORN: "\xDE",
        thorn: "\xFE",
        Tilde: "\u223C",
        tilde: "\u02DC",
        TildeEqual: "\u2243",
        TildeFullEqual: "\u2245",
        TildeTilde: "\u2248",
        times: "\xD7",
        timesb: "\u22A0",
        timesbar: "\u2A31",
        timesd: "\u2A30",
        tint: "\u222D",
        toea: "\u2928",
        top: "\u22A4",
        topbot: "\u2336",
        topcir: "\u2AF1",
        Topf: "\u{1D54B}",
        topf: "\u{1D565}",
        topfork: "\u2ADA",
        tosa: "\u2929",
        tprime: "\u2034",
        TRADE: "\u2122",
        trade: "\u2122",
        triangle: "\u25B5",
        triangledown: "\u25BF",
        triangleleft: "\u25C3",
        trianglelefteq: "\u22B4",
        triangleq: "\u225C",
        triangleright: "\u25B9",
        trianglerighteq: "\u22B5",
        tridot: "\u25EC",
        trie: "\u225C",
        triminus: "\u2A3A",
        TripleDot: "\u20DB",
        triplus: "\u2A39",
        trisb: "\u29CD",
        tritime: "\u2A3B",
        trpezium: "\u23E2",
        Tscr: "\u{1D4AF}",
        tscr: "\u{1D4C9}",
        TScy: "\u0426",
        tscy: "\u0446",
        TSHcy: "\u040B",
        tshcy: "\u045B",
        Tstrok: "\u0166",
        tstrok: "\u0167",
        twixt: "\u226C",
        twoheadleftarrow: "\u219E",
        twoheadrightarrow: "\u21A0",
        Uacute: "\xDA",
        uacute: "\xFA",
        Uarr: "\u219F",
        uArr: "\u21D1",
        uarr: "\u2191",
        Uarrocir: "\u2949",
        Ubrcy: "\u040E",
        ubrcy: "\u045E",
        Ubreve: "\u016C",
        ubreve: "\u016D",
        Ucirc: "\xDB",
        ucirc: "\xFB",
        Ucy: "\u0423",
        ucy: "\u0443",
        udarr: "\u21C5",
        Udblac: "\u0170",
        udblac: "\u0171",
        udhar: "\u296E",
        ufisht: "\u297E",
        Ufr: "\u{1D518}",
        ufr: "\u{1D532}",
        Ugrave: "\xD9",
        ugrave: "\xF9",
        uHar: "\u2963",
        uharl: "\u21BF",
        uharr: "\u21BE",
        uhblk: "\u2580",
        ulcorn: "\u231C",
        ulcorner: "\u231C",
        ulcrop: "\u230F",
        ultri: "\u25F8",
        Umacr: "\u016A",
        umacr: "\u016B",
        uml: "\xA8",
        UnderBar: "_",
        UnderBrace: "\u23DF",
        UnderBracket: "\u23B5",
        UnderParenthesis: "\u23DD",
        Union: "\u22C3",
        UnionPlus: "\u228E",
        Uogon: "\u0172",
        uogon: "\u0173",
        Uopf: "\u{1D54C}",
        uopf: "\u{1D566}",
        UpArrow: "\u2191",
        Uparrow: "\u21D1",
        uparrow: "\u2191",
        UpArrowBar: "\u2912",
        UpArrowDownArrow: "\u21C5",
        UpDownArrow: "\u2195",
        Updownarrow: "\u21D5",
        updownarrow: "\u2195",
        UpEquilibrium: "\u296E",
        upharpoonleft: "\u21BF",
        upharpoonright: "\u21BE",
        uplus: "\u228E",
        UpperLeftArrow: "\u2196",
        UpperRightArrow: "\u2197",
        Upsi: "\u03D2",
        upsi: "\u03C5",
        upsih: "\u03D2",
        Upsilon: "\u03A5",
        upsilon: "\u03C5",
        UpTee: "\u22A5",
        UpTeeArrow: "\u21A5",
        upuparrows: "\u21C8",
        urcorn: "\u231D",
        urcorner: "\u231D",
        urcrop: "\u230E",
        Uring: "\u016E",
        uring: "\u016F",
        urtri: "\u25F9",
        Uscr: "\u{1D4B0}",
        uscr: "\u{1D4CA}",
        utdot: "\u22F0",
        Utilde: "\u0168",
        utilde: "\u0169",
        utri: "\u25B5",
        utrif: "\u25B4",
        uuarr: "\u21C8",
        Uuml: "\xDC",
        uuml: "\xFC",
        uwangle: "\u29A7",
        vangrt: "\u299C",
        varepsilon: "\u03F5",
        varkappa: "\u03F0",
        varnothing: "\u2205",
        varphi: "\u03D5",
        varpi: "\u03D6",
        varpropto: "\u221D",
        vArr: "\u21D5",
        varr: "\u2195",
        varrho: "\u03F1",
        varsigma: "\u03C2",
        varsubsetneq: "\u228A\uFE00",
        varsubsetneqq: "\u2ACB\uFE00",
        varsupsetneq: "\u228B\uFE00",
        varsupsetneqq: "\u2ACC\uFE00",
        vartheta: "\u03D1",
        vartriangleleft: "\u22B2",
        vartriangleright: "\u22B3",
        Vbar: "\u2AEB",
        vBar: "\u2AE8",
        vBarv: "\u2AE9",
        Vcy: "\u0412",
        vcy: "\u0432",
        VDash: "\u22AB",
        Vdash: "\u22A9",
        vDash: "\u22A8",
        vdash: "\u22A2",
        Vdashl: "\u2AE6",
        Vee: "\u22C1",
        vee: "\u2228",
        veebar: "\u22BB",
        veeeq: "\u225A",
        vellip: "\u22EE",
        Verbar: "\u2016",
        verbar: "|",
        Vert: "\u2016",
        vert: "|",
        VerticalBar: "\u2223",
        VerticalLine: "|",
        VerticalSeparator: "\u2758",
        VerticalTilde: "\u2240",
        VeryThinSpace: "\u200A",
        Vfr: "\u{1D519}",
        vfr: "\u{1D533}",
        vltri: "\u22B2",
        vnsub: "\u2282\u20D2",
        vnsup: "\u2283\u20D2",
        Vopf: "\u{1D54D}",
        vopf: "\u{1D567}",
        vprop: "\u221D",
        vrtri: "\u22B3",
        Vscr: "\u{1D4B1}",
        vscr: "\u{1D4CB}",
        vsubnE: "\u2ACB\uFE00",
        vsubne: "\u228A\uFE00",
        vsupnE: "\u2ACC\uFE00",
        vsupne: "\u228B\uFE00",
        Vvdash: "\u22AA",
        vzigzag: "\u299A",
        Wcirc: "\u0174",
        wcirc: "\u0175",
        wedbar: "\u2A5F",
        Wedge: "\u22C0",
        wedge: "\u2227",
        wedgeq: "\u2259",
        weierp: "\u2118",
        Wfr: "\u{1D51A}",
        wfr: "\u{1D534}",
        Wopf: "\u{1D54E}",
        wopf: "\u{1D568}",
        wp: "\u2118",
        wr: "\u2240",
        wreath: "\u2240",
        Wscr: "\u{1D4B2}",
        wscr: "\u{1D4CC}",
        xcap: "\u22C2",
        xcirc: "\u25EF",
        xcup: "\u22C3",
        xdtri: "\u25BD",
        Xfr: "\u{1D51B}",
        xfr: "\u{1D535}",
        xhArr: "\u27FA",
        xharr: "\u27F7",
        Xi: "\u039E",
        xi: "\u03BE",
        xlArr: "\u27F8",
        xlarr: "\u27F5",
        xmap: "\u27FC",
        xnis: "\u22FB",
        xodot: "\u2A00",
        Xopf: "\u{1D54F}",
        xopf: "\u{1D569}",
        xoplus: "\u2A01",
        xotime: "\u2A02",
        xrArr: "\u27F9",
        xrarr: "\u27F6",
        Xscr: "\u{1D4B3}",
        xscr: "\u{1D4CD}",
        xsqcup: "\u2A06",
        xuplus: "\u2A04",
        xutri: "\u25B3",
        xvee: "\u22C1",
        xwedge: "\u22C0",
        Yacute: "\xDD",
        yacute: "\xFD",
        YAcy: "\u042F",
        yacy: "\u044F",
        Ycirc: "\u0176",
        ycirc: "\u0177",
        Ycy: "\u042B",
        ycy: "\u044B",
        yen: "\xA5",
        Yfr: "\u{1D51C}",
        yfr: "\u{1D536}",
        YIcy: "\u0407",
        yicy: "\u0457",
        Yopf: "\u{1D550}",
        yopf: "\u{1D56A}",
        Yscr: "\u{1D4B4}",
        yscr: "\u{1D4CE}",
        YUcy: "\u042E",
        yucy: "\u044E",
        Yuml: "\u0178",
        yuml: "\xFF",
        Zacute: "\u0179",
        zacute: "\u017A",
        Zcaron: "\u017D",
        zcaron: "\u017E",
        Zcy: "\u0417",
        zcy: "\u0437",
        Zdot: "\u017B",
        zdot: "\u017C",
        zeetrf: "\u2128",
        ZeroWidthSpace: "\u200B",
        Zeta: "\u0396",
        zeta: "\u03B6",
        Zfr: "\u2128",
        zfr: "\u{1D537}",
        ZHcy: "\u0416",
        zhcy: "\u0436",
        zigrarr: "\u21DD",
        Zopf: "\u2124",
        zopf: "\u{1D56B}",
        Zscr: "\u{1D4B5}",
        zscr: "\u{1D4CF}",
        zwj: "\u200D",
        zwnj: "\u200C"
      }), n2.entityMap = n2.HTML_ENTITIES;
    })(gt)), gt;
  }
  function gr() {
    if (Rt) return ht;
    Rt = 1;
    var n2 = st(), t4 = Yt(), u3 = dt(), s3 = n2.isHTMLEscapableRawTextElement, c5 = n2.isHTMLMimeType, o5 = n2.isHTMLRawTextElement, h4 = n2.hasOwn, C2 = n2.NAMESPACE, f7 = u3.ParseError, g5 = u3.DOMException, D = 0, p5 = 1, B3 = 2, k4 = 3, Y2 = 4, X2 = 5, te = 6, M2 = 7;
    function q3() {
    }
    q3.prototype = {
      parse: function(l6, A6, b5) {
        var v3 = this.domBuilder;
        v3.startDocument(), U5(A6, A6 = /* @__PURE__ */ Object.create(null)), ue2(l6, A6, b5, v3, this.errorHandler), v3.endDocument();
      }
    };
    var W = /&#?\w+;?/g;
    function ue2(l6, A6, b5, v3, y7) {
      var E3 = c5(v3.mimeType);
      l6.indexOf(t4.UNICODE_REPLACEMENT_CHARACTER) >= 0 && y7.warning("Unicode replacement character detected, source encoding issues?");
      function S4(F4) {
        if (F4 > 65535) {
          F4 -= 65536;
          var Q2 = 55296 + (F4 >> 10), De = 56320 + (F4 & 1023);
          return String.fromCharCode(Q2, De);
        } else
          return String.fromCharCode(F4);
      }
      function V2(F4) {
        var Q2 = F4[F4.length - 1] === ";" ? F4 : F4 + ";";
        if (!E3 && Q2 !== F4)
          return y7.error("EntityRef: expecting ;"), F4;
        var De = t4.Reference.exec(Q2);
        if (!De || De[0].length !== Q2.length)
          return y7.error("entity not matching Reference production: " + F4), F4;
        var ge = Q2.slice(1, -1);
        return h4(b5, ge) ? b5[ge] : ge.charAt(0) === "#" ? S4(parseInt(ge.substring(1).replace("x", "0x"))) : (y7.error("entity not found:" + F4), F4);
      }
      function O6(F4) {
        if (F4 > ie) {
          var Q2 = l6.substring(ie, F4).replace(W, V2);
          P && oe(ie), v3.characters(Q2, 0, F4 - ie), ie = F4;
        }
      }
      var T2 = 0, w3 = 0, I4 = /\r\n?|\n|$/g, P = v3.locator;
      function oe(F4, Q2) {
        for (; F4 >= w3 && (Q2 = I4.exec(l6)); )
          T2 = w3, w3 = Q2.index + Q2[0].length, P.lineNumber++;
        P.columnNumber = F4 - T2 + 1;
      }
      for (var Te = [{ currentNSMap: A6 }], Ee = [], ie = 0; ; ) {
        try {
          var R5 = l6.indexOf("<", ie);
          if (R5 < 0) {
            if (!E3 && Ee.length > 0)
              return y7.fatalError("unclosed xml tag(s): " + Ee.join(", "));
            if (!l6.substring(ie).match(/^\s*$/)) {
              var Fe = v3.doc, ye = Fe.createTextNode(l6.substring(ie));
              if (Fe.documentElement)
                return y7.error("Extra content at the end of the document");
              Fe.appendChild(ye), v3.currentElement = ye;
            }
            return;
          }
          if (R5 > ie) {
            var ce2 = l6.substring(ie, R5);
            !E3 && Ee.length === 0 && (ce2 = ce2.replace(new RegExp(t4.S_OPT.source, "g"), ""), ce2 && y7.error("Unexpected content outside root element: '" + ce2 + "'")), O6(R5);
          }
          switch (l6.charAt(R5 + 1)) {
            case "/":
              var ne = l6.indexOf(">", R5 + 2), Le = l6.substring(R5 + 2, ne > 0 ? ne : void 0);
              if (!Le)
                return y7.fatalError("end tag name missing");
              var _e = ne > 0 && t4.reg("^", t4.QName_group, t4.S_OPT, "$").exec(Le);
              if (!_e)
                return y7.fatalError('end tag name contains invalid characters: "' + Le + '"');
              if (!v3.currentElement && !v3.doc.documentElement)
                return;
              var Ce = Ee[Ee.length - 1] || v3.currentElement.tagName || v3.doc.documentElement.tagName || "";
              if (Ce !== _e[1]) {
                var Re = _e[1].toLowerCase();
                if (!E3 || Ce.toLowerCase() !== Re)
                  return y7.fatalError('Opening and ending tag mismatch: "' + Ce + '" != "' + Le + '"');
              }
              var Ue = Te.pop();
              Ee.pop();
              var qe = Ue.localNSMap;
              if (v3.endElement(Ue.uri, Ue.localName, Ce), qe)
                for (var Ae in qe)
                  h4(qe, Ae) && v3.endPrefixMapping(Ae);
              ne++;
              break;
            // end element
            case "?":
              P && oe(R5), ne = Z3(l6, R5, v3, y7);
              break;
            case "!":
              P && oe(R5), ne = Ne(l6, R5, v3, y7, E3);
              break;
            default:
              P && oe(R5);
              var j2 = new we(), Pe = Te[Te.length - 1].currentNSMap, ne = d6(l6, R5, j2, Pe, V2, y7, E3), Ge = j2.length;
              if (j2.closed || (E3 && n2.isHTMLVoidElement(j2.tagName) ? j2.closed = true : Ee.push(j2.tagName)), P && Ge) {
                for (var at = me(P, {}), Ve = 0; Ve < Ge; Ve++) {
                  var ze = j2[Ve];
                  oe(ze.offset), ze.locator = me(P, {});
                }
                v3.locator = at, _3(j2, v3, Pe) && Te.push(j2), v3.locator = P;
              } else
                _3(j2, v3, Pe) && Te.push(j2);
              E3 && !j2.closed ? ne = x5(l6, ne, j2.tagName, V2, v3) : ne++;
          }
        } catch (F4) {
          if (F4 instanceof f7)
            throw F4;
          if (F4 instanceof g5)
            throw new f7(F4.name + ": " + F4.message, v3.locator, F4);
          y7.error("element parse error: " + F4), ne = -1;
        }
        ne > ie ? ie = ne : O6(Math.max(R5, ie) + 1);
      }
    }
    function me(l6, A6) {
      return A6.lineNumber = l6.lineNumber, A6.columnNumber = l6.columnNumber, A6;
    }
    function d6(l6, A6, b5, v3, y7, E3, S4) {
      function V2(oe, Te, Ee) {
        if (h4(b5.attributeNames, oe))
          return E3.fatalError("Attribute " + oe + " redefined");
        if (!S4 && Te.indexOf("<") >= 0)
          return E3.fatalError("Unescaped '<' not allowed in attributes values");
        b5.addValue(
          oe,
          // @see https://www.w3.org/TR/xml/#AVNormalize
          // since the xmldom sax parser does not "interpret" DTD the following is not implemented:
          // - recursive replacement of (DTD) entity references
          // - trimming and collapsing multiple spaces into a single one for attributes that are not of type CDATA
          Te.replace(/[\t\n\r]/g, " ").replace(W, y7),
          Ee
        );
      }
      for (var O6, T2, w3 = ++A6, I4 = D; ; ) {
        var P = l6.charAt(w3);
        switch (P) {
          case "=":
            if (I4 === p5)
              O6 = l6.slice(A6, w3), I4 = k4;
            else if (I4 === B3)
              I4 = k4;
            else
              throw new Error("attribute equal must after attrName");
            break;
          case "'":
          case '"':
            if (I4 === k4 || I4 === p5)
              if (I4 === p5 && (E3.warning('attribute value must after "="'), O6 = l6.slice(A6, w3)), A6 = w3 + 1, w3 = l6.indexOf(P, A6), w3 > 0)
                T2 = l6.slice(A6, w3), V2(O6, T2, A6 - 1), I4 = X2;
              else
                throw new Error("attribute value no end '" + P + "' match");
            else if (I4 == Y2)
              T2 = l6.slice(A6, w3), V2(O6, T2, A6), E3.warning('attribute "' + O6 + '" missed start quot(' + P + ")!!"), A6 = w3 + 1, I4 = X2;
            else
              throw new Error('attribute value must after "="');
            break;
          case "/":
            switch (I4) {
              case D:
                b5.setTagName(l6.slice(A6, w3));
              case X2:
              case te:
              case M2:
                I4 = M2, b5.closed = true;
              case Y2:
              case p5:
                break;
              case B3:
                b5.closed = true;
                break;
              //case S_EQ:
              default:
                throw new Error("attribute invalid close char('/')");
            }
            break;
          case "":
            return E3.error("unexpected end of input"), I4 == D && b5.setTagName(l6.slice(A6, w3)), w3;
          case ">":
            switch (I4) {
              case D:
                b5.setTagName(l6.slice(A6, w3));
              case X2:
              case te:
              case M2:
                break;
              //normal
              case Y2:
              //Compatible state
              case p5:
                T2 = l6.slice(A6, w3), T2.slice(-1) === "/" && (b5.closed = true, T2 = T2.slice(0, -1));
              case B3:
                I4 === B3 && (T2 = O6), I4 == Y2 ? (E3.warning('attribute "' + T2 + '" missed quot(")!'), V2(O6, T2, A6)) : (S4 || E3.warning('attribute "' + T2 + '" missed value!! "' + T2 + '" instead!!'), V2(T2, T2, A6));
                break;
              case k4:
                if (!S4)
                  return E3.fatalError(`AttValue: ' or " expected`);
            }
            return w3;
          /*xml space '\x20' | #x9 | #xD | #xA; */
          case "\x80":
            P = " ";
          default:
            if (P <= " ")
              switch (I4) {
                case D:
                  b5.setTagName(l6.slice(A6, w3)), I4 = te;
                  break;
                case p5:
                  O6 = l6.slice(A6, w3), I4 = B3;
                  break;
                case Y2:
                  var T2 = l6.slice(A6, w3);
                  E3.warning('attribute "' + T2 + '" missed quot(")!!'), V2(O6, T2, A6);
                case X2:
                  I4 = te;
                  break;
              }
            else
              switch (I4) {
                //case S_TAG:void();break;
                //case S_ATTR:void();break;
                //case S_ATTR_NOQUOT_VALUE:void();break;
                case B3:
                  S4 || E3.warning('attribute "' + O6 + '" missed value!! "' + O6 + '" instead2!!'), V2(O6, O6, A6), A6 = w3, I4 = p5;
                  break;
                case X2:
                  E3.warning('attribute space is required"' + O6 + '"!!');
                case te:
                  I4 = p5, A6 = w3;
                  break;
                case k4:
                  I4 = Y2, A6 = w3;
                  break;
                case M2:
                  throw new Error("elements closed character '/' and '>' must be connected to");
              }
        }
        w3++;
      }
    }
    function _3(l6, A6, b5) {
      for (var v3 = l6.tagName, y7 = null, I4 = l6.length; I4--; ) {
        var E3 = l6[I4], S4 = E3.qName, V2 = E3.value, P = S4.indexOf(":");
        if (P > 0)
          var O6 = E3.prefix = S4.slice(0, P), T2 = S4.slice(P + 1), w3 = O6 === "xmlns" && T2;
        else
          T2 = S4, O6 = null, w3 = S4 === "xmlns" && "";
        E3.localName = T2, w3 !== false && (y7 == null && (y7 = /* @__PURE__ */ Object.create(null), U5(b5, b5 = /* @__PURE__ */ Object.create(null))), b5[w3] = y7[w3] = V2, E3.uri = C2.XMLNS, A6.startPrefixMapping(w3, V2));
      }
      for (var I4 = l6.length; I4--; )
        E3 = l6[I4], E3.prefix && (E3.prefix === "xml" && (E3.uri = C2.XML), E3.prefix !== "xmlns" && (E3.uri = b5[E3.prefix]));
      var P = v3.indexOf(":");
      P > 0 ? (O6 = l6.prefix = v3.slice(0, P), T2 = l6.localName = v3.slice(P + 1)) : (O6 = null, T2 = l6.localName = v3);
      var oe = l6.uri = b5[O6 || ""];
      if (A6.startElement(oe, T2, v3, l6), l6.closed) {
        if (A6.endElement(oe, T2, v3), y7)
          for (O6 in y7)
            h4(y7, O6) && A6.endPrefixMapping(O6);
      } else
        return l6.currentNSMap = b5, l6.localNSMap = y7, true;
    }
    function x5(l6, A6, b5, v3, y7) {
      var E3 = s3(b5);
      if (E3 || o5(b5)) {
        var S4 = l6.indexOf("</" + b5 + ">", A6), V2 = l6.substring(A6 + 1, S4);
        return E3 && (V2 = V2.replace(W, v3)), y7.characters(V2, 0, V2.length), S4;
      }
      return A6 + 1;
    }
    function U5(l6, A6) {
      for (var b5 in l6)
        h4(l6, b5) && (A6[b5] = l6[b5]);
    }
    function $3(l6, A6) {
      var b5 = A6;
      function v3(w3) {
        return w3 = w3 || 0, l6.charAt(b5 + w3);
      }
      function y7(w3) {
        w3 = w3 || 1, b5 += w3;
      }
      function E3() {
        for (var w3 = 0; b5 < l6.length; ) {
          var I4 = v3();
          if (I4 !== " " && I4 !== `
` && I4 !== "	" && I4 !== "\r")
            return w3;
          w3++, y7();
        }
        return -1;
      }
      function S4() {
        return l6.substring(b5);
      }
      function V2(w3) {
        return l6.substring(b5, b5 + w3.length) === w3;
      }
      function O6(w3) {
        return l6.substring(b5, b5 + w3.length).toUpperCase() === w3.toUpperCase();
      }
      function T2(w3) {
        var I4 = t4.reg("^", w3), P = I4.exec(S4());
        return P ? (y7(P[0].length), P[0]) : null;
      }
      return {
        char: v3,
        getIndex: function() {
          return b5;
        },
        getMatch: T2,
        getSource: function() {
          return l6;
        },
        skip: y7,
        skipBlanks: E3,
        substringFromIndex: S4,
        substringStartsWith: V2,
        substringStartsWithCaseInsensitive: O6
      };
    }
    function J2(l6, A6) {
      function b5(V2, O6) {
        var T2 = t4.PI.exec(V2.substringFromIndex());
        return T2 ? T2[1].toLowerCase() === "xml" ? O6.fatalError(
          "xml declaration is only allowed at the start of the document, but found at position " + V2.getIndex()
        ) : (V2.skip(T2[0].length), T2[0]) : O6.fatalError("processing instruction is not well-formed at position " + V2.getIndex());
      }
      var v3 = l6.getSource();
      if (l6.char() === "[") {
        l6.skip(1);
        for (var y7 = l6.getIndex(); l6.getIndex() < v3.length; ) {
          if (l6.skipBlanks(), l6.char() === "]") {
            var E3 = v3.substring(y7, l6.getIndex());
            return l6.skip(1), E3;
          }
          var S4 = null;
          if (l6.char() === "<" && l6.char(1) === "!")
            switch (l6.char(2)) {
              case "E":
                l6.char(3) === "L" ? S4 = l6.getMatch(t4.elementdecl) : l6.char(3) === "N" && (S4 = l6.getMatch(t4.EntityDecl));
                break;
              case "A":
                S4 = l6.getMatch(t4.AttlistDecl);
                break;
              case "N":
                S4 = l6.getMatch(t4.NotationDecl);
                break;
              case "-":
                S4 = l6.getMatch(t4.Comment);
                break;
            }
          else if (l6.char() === "<" && l6.char(1) === "?")
            S4 = b5(l6, A6);
          else if (l6.char() === "%")
            S4 = l6.getMatch(t4.PEReference);
          else
            return A6.fatalError("Error detected in Markup declaration");
          if (!S4)
            return A6.fatalError("Error in internal subset at position " + l6.getIndex());
        }
        return A6.fatalError("doctype internal subset is not well-formed, missing ]");
      }
    }
    function Ne(l6, A6, b5, v3, y7) {
      var E3 = $3(l6, A6);
      switch (y7 ? E3.char(2).toUpperCase() : E3.char(2)) {
        case "-":
          var S4 = E3.getMatch(t4.Comment);
          return S4 ? (b5.comment(S4, t4.COMMENT_START.length, S4.length - t4.COMMENT_START.length - t4.COMMENT_END.length), E3.getIndex()) : v3.fatalError("comment is not well-formed at position " + E3.getIndex());
        case "[":
          var V2 = E3.getMatch(t4.CDSect);
          return V2 ? !y7 && !b5.currentElement ? v3.fatalError("CDATA outside of element") : (b5.startCDATA(), b5.characters(V2, t4.CDATA_START.length, V2.length - t4.CDATA_START.length - t4.CDATA_END.length), b5.endCDATA(), E3.getIndex()) : v3.fatalError("Invalid CDATA starting at position " + A6);
        case "D": {
          if (b5.doc && b5.doc.documentElement)
            return v3.fatalError("Doctype not allowed inside or after documentElement at position " + E3.getIndex());
          if (y7 ? !E3.substringStartsWithCaseInsensitive(t4.DOCTYPE_DECL_START) : !E3.substringStartsWith(t4.DOCTYPE_DECL_START))
            return v3.fatalError("Expected " + t4.DOCTYPE_DECL_START + " at position " + E3.getIndex());
          if (E3.skip(t4.DOCTYPE_DECL_START.length), E3.skipBlanks() < 1)
            return v3.fatalError("Expected whitespace after " + t4.DOCTYPE_DECL_START + " at position " + E3.getIndex());
          var O6 = {
            name: void 0,
            publicId: void 0,
            systemId: void 0,
            internalSubset: void 0
          };
          if (O6.name = E3.getMatch(t4.Name), !O6.name)
            return v3.fatalError("doctype name missing or contains unexpected characters at position " + E3.getIndex());
          if (y7 && O6.name.toLowerCase() !== "html" && v3.warning("Unexpected DOCTYPE in HTML document at position " + E3.getIndex()), E3.skipBlanks(), E3.substringStartsWith(t4.PUBLIC) || E3.substringStartsWith(t4.SYSTEM)) {
            var T2 = t4.ExternalID_match.exec(E3.substringFromIndex());
            if (!T2)
              return v3.fatalError("doctype external id is not well-formed at position " + E3.getIndex());
            T2.groups.SystemLiteralOnly !== void 0 ? O6.systemId = T2.groups.SystemLiteralOnly : (O6.systemId = T2.groups.SystemLiteral, O6.publicId = T2.groups.PubidLiteral), E3.skip(T2[0].length);
          } else if (y7 && E3.substringStartsWithCaseInsensitive(t4.SYSTEM)) {
            if (E3.skip(t4.SYSTEM.length), E3.skipBlanks() < 1)
              return v3.fatalError("Expected whitespace after " + t4.SYSTEM + " at position " + E3.getIndex());
            if (O6.systemId = E3.getMatch(t4.ABOUT_LEGACY_COMPAT_SystemLiteral), !O6.systemId)
              return v3.fatalError(
                "Expected " + t4.ABOUT_LEGACY_COMPAT + " in single or double quotes after " + t4.SYSTEM + " at position " + E3.getIndex()
              );
          }
          return y7 && O6.systemId && !t4.ABOUT_LEGACY_COMPAT_SystemLiteral.test(O6.systemId) && v3.warning("Unexpected doctype.systemId in HTML document at position " + E3.getIndex()), y7 || (E3.skipBlanks(), O6.internalSubset = J2(E3, v3)), E3.skipBlanks(), E3.char() !== ">" ? v3.fatalError("doctype not terminated with > at position " + E3.getIndex()) : (E3.skip(1), b5.startDTD(O6.name, O6.publicId, O6.systemId, O6.internalSubset), b5.endDTD(), E3.getIndex());
        }
        default:
          return v3.fatalError('Not well-formed XML starting with "<!" at position ' + A6);
      }
    }
    function Z3(l6, A6, b5, v3) {
      var y7 = l6.substring(A6).match(t4.PI);
      if (!y7)
        return v3.fatalError("Invalid processing instruction starting at position " + A6);
      if (y7[1].toLowerCase() === "xml") {
        if (A6 > 0)
          return v3.fatalError(
            "processing instruction at position " + A6 + " is an xml declaration which is only at the start of the document"
          );
        if (!t4.XMLDecl.test(l6.substring(A6)))
          return v3.fatalError("xml declaration is not well-formed");
      }
      return b5.processingInstruction(y7[1], y7[2]), A6 + y7[0].length;
    }
    function we() {
      this.attributeNames = /* @__PURE__ */ Object.create(null);
    }
    return we.prototype = {
      setTagName: function(l6) {
        if (!t4.QName_exact.test(l6))
          throw new Error("invalid tagName:" + l6);
        this.tagName = l6;
      },
      addValue: function(l6, A6, b5) {
        if (!t4.QName_exact.test(l6))
          throw new Error("invalid attribute:" + l6);
        this.attributeNames[l6] = this.length, this[this.length++] = { qName: l6, value: A6, offset: b5 };
      },
      length: 0,
      getLocalName: function(l6) {
        return this[l6].localName;
      },
      getLocator: function(l6) {
        return this[l6].locator;
      },
      getQName: function(l6) {
        return this[l6].qName;
      },
      getURI: function(l6) {
        return this[l6].uri;
      },
      getValue: function(l6) {
        return this[l6].value;
      }
      //	,getIndex:function(uri, localName)){
      //		if(localName){
      //
      //		}else{
      //			var qName = uri
      //		}
      //	},
      //	getValue:function(){return this.getValue(this.getIndex.apply(this,arguments))},
      //	getType:function(uri,localName){}
      //	getType:function(i){},
    }, ht.XMLReader = q3, ht.parseUtils = $3, ht.parseDoctypeCommentOrCData = Ne, ht;
  }
  function Ar() {
    if (Mt) return We;
    Mt = 1;
    var n2 = st(), t4 = Xt(), u3 = dt(), s3 = Dr(), c5 = gr(), o5 = t4.DOMImplementation, h4 = n2.hasDefaultHTMLNamespace, C2 = n2.isHTMLMimeType, f7 = n2.isValidMimeType, g5 = n2.MIME_TYPE, D = n2.NAMESPACE, p5 = u3.ParseError, B3 = c5.XMLReader;
    function k4(d6) {
      return d6.replace(/\r[\n\u0085]/g, `
`).replace(/[\r\u0085\u2028\u2029]/g, `
`);
    }
    function Y2(d6) {
      if (d6 = d6 || {}, d6.locator === void 0 && (d6.locator = true), this.assign = d6.assign || n2.assign, this.domHandler = d6.domHandler || X2, this.onError = d6.onError || d6.errorHandler, d6.errorHandler && typeof d6.errorHandler != "function")
        throw new TypeError("errorHandler object is no longer supported, switch to onError!");
      d6.errorHandler && d6.errorHandler("warning", "The `errorHandler` option has been deprecated, use `onError` instead!", this), this.normalizeLineEndings = d6.normalizeLineEndings || k4, this.locator = !!d6.locator, this.xmlns = this.assign(/* @__PURE__ */ Object.create(null), d6.xmlns);
    }
    Y2.prototype.parseFromString = function(d6, _3) {
      if (!f7(_3))
        throw new TypeError('DOMParser.parseFromString: the provided mimeType "' + _3 + '" is not valid.');
      var x5 = this.assign(/* @__PURE__ */ Object.create(null), this.xmlns), U5 = s3.XML_ENTITIES, $3 = x5[""] || null;
      h4(_3) ? (U5 = s3.HTML_ENTITIES, $3 = D.HTML) : _3 === g5.XML_SVG_IMAGE && ($3 = D.SVG), x5[""] = $3, x5.xml = x5.xml || D.XML;
      var J2 = new this.domHandler({
        mimeType: _3,
        defaultNamespace: $3,
        onError: this.onError
      }), Ne = this.locator ? {} : void 0;
      this.locator && J2.setDocumentLocator(Ne);
      var Z3 = new B3();
      Z3.errorHandler = J2, Z3.domBuilder = J2;
      var we = !n2.isHTMLMimeType(_3);
      return we && typeof d6 != "string" && Z3.errorHandler.fatalError("source is not a string"), Z3.parse(this.normalizeLineEndings(String(d6)), x5, U5), J2.doc.documentElement || Z3.errorHandler.fatalError("missing root element"), J2.doc;
    };
    function X2(d6) {
      var _3 = d6 || {};
      this.mimeType = _3.mimeType || g5.XML_APPLICATION, this.defaultNamespace = _3.defaultNamespace || null, this.cdata = false, this.currentElement = void 0, this.doc = void 0, this.locator = void 0, this.onError = _3.onError;
    }
    function te(d6, _3) {
      _3.lineNumber = d6.lineNumber, _3.columnNumber = d6.columnNumber;
    }
    X2.prototype = {
      /**
       * Either creates an XML or an HTML document and stores it under `this.doc`.
       * If it is an XML document, `this.defaultNamespace` is used to create it,
       * and it will not contain any `childNodes`.
       * If it is an HTML document, it will be created without any `childNodes`.
       *
       * @see http://www.saxproject.org/apidoc/org/xml/sax/ContentHandler.html
       */
      startDocument: function() {
        var d6 = new o5();
        this.doc = C2(this.mimeType) ? d6.createHTMLDocument(false) : d6.createDocument(this.defaultNamespace, "");
      },
      startElement: function(d6, _3, x5, U5) {
        var $3 = this.doc, J2 = $3.createElementNS(d6, x5 || _3), Ne = U5.length;
        W(this, J2), this.currentElement = J2, this.locator && te(this.locator, J2);
        for (var Z3 = 0; Z3 < Ne; Z3++) {
          var d6 = U5.getURI(Z3), we = U5.getValue(Z3), x5 = U5.getQName(Z3), l6 = $3.createAttributeNS(d6, x5);
          this.locator && te(U5.getLocator(Z3), l6), l6.value = l6.nodeValue = we, J2.setAttributeNode(l6);
        }
      },
      endElement: function(d6, _3, x5) {
        this.currentElement = this.currentElement.parentNode;
      },
      startPrefixMapping: function(d6, _3) {
      },
      endPrefixMapping: function(d6) {
      },
      processingInstruction: function(d6, _3) {
        var x5 = this.doc.createProcessingInstruction(d6, _3);
        this.locator && te(this.locator, x5), W(this, x5);
      },
      ignorableWhitespace: function(d6, _3, x5) {
      },
      characters: function(d6, _3, x5) {
        if (d6 = q3.apply(this, arguments), d6) {
          if (this.cdata)
            var U5 = this.doc.createCDATASection(d6);
          else
            var U5 = this.doc.createTextNode(d6);
          this.currentElement ? this.currentElement.appendChild(U5) : /^\s*$/.test(d6) && this.doc.appendChild(U5), this.locator && te(this.locator, U5);
        }
      },
      skippedEntity: function(d6) {
      },
      endDocument: function() {
        this.doc.normalize();
      },
      /**
       * Stores the locator to be able to set the `columnNumber` and `lineNumber`
       * on the created DOM nodes.
       *
       * @param {Locator} locator
       */
      setDocumentLocator: function(d6) {
        d6 && (d6.lineNumber = 0), this.locator = d6;
      },
      //LexicalHandler
      comment: function(d6, _3, x5) {
        d6 = q3.apply(this, arguments);
        var U5 = this.doc.createComment(d6);
        this.locator && te(this.locator, U5), W(this, U5);
      },
      startCDATA: function() {
        this.cdata = true;
      },
      endCDATA: function() {
        this.cdata = false;
      },
      startDTD: function(d6, _3, x5, U5) {
        var $3 = this.doc.implementation;
        if ($3 && $3.createDocumentType) {
          var J2 = $3.createDocumentType(d6, _3, x5, U5);
          this.locator && te(this.locator, J2), W(this, J2), this.doc.doctype = J2;
        }
      },
      reportError: function(d6, _3) {
        if (typeof this.onError == "function")
          try {
            this.onError(d6, _3, this);
          } catch (x5) {
            throw new p5("Reporting " + d6 + ' "' + _3 + '" caused ' + x5, this.locator);
          }
        else
          console.error("[xmldom " + d6 + "]	" + _3, M2(this.locator));
      },
      /**
       * @see http://www.saxproject.org/apidoc/org/xml/sax/ErrorHandler.html
       */
      warning: function(d6) {
        this.reportError("warning", d6);
      },
      error: function(d6) {
        this.reportError("error", d6);
      },
      /**
       * This function reports a fatal error and throws a ParseError.
       *
       * @param {string} message
       * - The message to be used for reporting and throwing the error.
       * @returns {never}
       * This function always throws an error and never returns a value.
       * @throws {ParseError}
       * Always throws a ParseError with the provided message.
       */
      fatalError: function(d6) {
        throw this.reportError("fatalError", d6), new p5(d6, this.locator);
      }
    };
    function M2(d6) {
      if (d6)
        return `
@#[line:` + d6.lineNumber + ",col:" + d6.columnNumber + "]";
    }
    function q3(d6, _3, x5) {
      return typeof d6 == "string" ? d6.substr(_3, x5) : d6.length >= _3 + x5 || _3 ? new java.lang.String(d6, _3, x5) + "" : d6;
    }
    "endDTD,startEntity,endEntity,attributeDecl,elementDecl,externalEntityDecl,internalEntityDecl,resolveEntity,getExternalSubset,notationDecl,unparsedEntityDecl".replace(
      /\w+/g,
      function(d6) {
        X2.prototype[d6] = function() {
          return null;
        };
      }
    );
    function W(d6, _3) {
      d6.currentElement ? d6.currentElement.appendChild(_3) : d6.doc.appendChild(_3);
    }
    function ue2(d6) {
      if (d6 === "error") throw "onErrorStopParsing";
    }
    function me() {
      throw "onWarningStopParsing";
    }
    return We.__DOMHandler = X2, We.DOMParser = Y2, We.normalizeLineEndings = k4, We.onErrorStopParsing = ue2, We.onWarningStopParsing = me, We;
  }
  function vr() {
    if (It) return G;
    It = 1;
    var n2 = st();
    G.assign = n2.assign, G.hasDefaultHTMLNamespace = n2.hasDefaultHTMLNamespace, G.isHTMLMimeType = n2.isHTMLMimeType, G.isValidMimeType = n2.isValidMimeType, G.MIME_TYPE = n2.MIME_TYPE, G.NAMESPACE = n2.NAMESPACE;
    var t4 = dt();
    G.DOMException = t4.DOMException, G.DOMExceptionName = t4.DOMExceptionName, G.ExceptionCode = t4.ExceptionCode, G.ParseError = t4.ParseError;
    var u3 = Xt();
    G.Attr = u3.Attr, G.CDATASection = u3.CDATASection, G.CharacterData = u3.CharacterData, G.Comment = u3.Comment, G.Document = u3.Document, G.DocumentFragment = u3.DocumentFragment, G.DocumentType = u3.DocumentType, G.DOMImplementation = u3.DOMImplementation, G.Element = u3.Element, G.Entity = u3.Entity, G.EntityReference = u3.EntityReference, G.LiveNodeList = u3.LiveNodeList, G.NamedNodeMap = u3.NamedNodeMap, G.Node = u3.Node, G.NodeList = u3.NodeList, G.Notation = u3.Notation, G.ProcessingInstruction = u3.ProcessingInstruction, G.Text = u3.Text, G.XMLSerializer = u3.XMLSerializer;
    var s3 = Ar();
    return G.DOMParser = s3.DOMParser, G.normalizeLineEndings = s3.normalizeLineEndings, G.onErrorStopParsing = s3.onErrorStopParsing, G.onWarningStopParsing = s3.onWarningStopParsing, G;
  }
  var re, fe, Kt, qt, er, Oe, Qe, vt, Gt, tr, rr, ur, Lr, Be, Nt, nr, sr, Pr, kr, pt, Ur, ft, H3, cr, lr, hr, Vt, pr, fr, dr, Er, G, ae2, wt, it, yt, K2, L4, _t, St, We, gt, Ot, ht, Rt, Mt, It, xt, Tr, qr, Gr;
  var init_RosLib = __esm({
    "node_modules/roslib/dist/RosLib.js"() {
      init_eventemitter3();
      init_dist();
      init_bson();
      init_lib2();
      init_lib3();
      re = class extends import_index.default {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.name - The service name, like '/add_two_ints'.
         * @param options.serviceType - The service type, like 'rospy_tutorials/AddTwoInts'.
         */
        constructor({
          ros: t4,
          name: u3,
          serviceType: s3
        }) {
          super(), this.#e = null, this.isAdvertised = false, this.#t = Promise.resolve(), this.#r = false, this.ros = t4, this.name = u3, this.serviceType = s3;
        }
        #e;
        #t;
        #r;
        /**
         * Call the service. Returns the service response in the
         * callback. Does nothing if this service is currently advertised.
         *
         * @param request - The service request to send.
         * @param [callback] - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         * @param [timeout] - Optional timeout, in seconds, for the service call. A non-positive value means no timeout.
         *                             If not provided, the rosbridge server will use its default value.
         */
        callService(t4, u3, s3 = console.error, c5) {
          if (this.isAdvertised)
            return;
          const o5 = `call_service:${this.name}:${v4_default()}`;
          this.ros.once(o5, function(h4) {
            Ft(h4) && (h4.result ? u3?.(h4.values) : s3(h4.values ?? ""));
          }), this.ros.callOnConnection({
            op: "call_service",
            id: o5,
            service: this.name,
            args: t4,
            timeout: c5
          });
        }
        /**
         * Advertise the service. This turns the Service object from a client
         * into a server. The callback will be called with every request
         * that's made on this service.
         *
         * @param callback This works similarly to the callback for a C++ service in that you should take care not to overwrite the response object.
         *  Instead, only modify the values within.
         */
        async advertise(t4) {
          return this.#t = this.#t.then(() => {
            this.isAdvertised && this.#u(), this.#e = (u3) => {
              if (!At(u3))
                throw new Error(
                  `Invalid message received on service channel: ${JSON.stringify(u3)}`
                );
              const s3 = {};
              let c5;
              try {
                c5 = t4(u3.args, s3);
              } catch {
                c5 = false;
              }
              c5 ? this.ros.callOnConnection({
                op: "service_response",
                service: this.name,
                values: s3,
                result: c5,
                id: u3.id
              }) : this.ros.callOnConnection({
                op: "service_response",
                service: this.name,
                result: c5,
                id: u3.id
              });
            }, this.ros.on(this.name, this.#e), this.ros.callOnConnection({
              op: "advertise_service",
              type: this.serviceType,
              service: this.name
            }), this.isAdvertised = true;
          }).catch((u3) => {
            throw this.emit("error", u3), u3;
          }), this.#t;
        }
        /**
         * Internal method to perform unadvertisement without queueing
         */
        #u() {
          if (!(!this.isAdvertised || this.#r)) {
            this.#r = true;
            try {
              this.isAdvertised = false, this.#e && (this.ros.off(this.name, this.#e), this.#e = null), this.ros.callOnConnection({
                op: "unadvertise_service",
                service: this.name
              });
            } finally {
              this.#r = false;
            }
          }
        }
        async unadvertise() {
          return this.#t = this.#t.then(() => {
            this.#u();
          }).catch((t4) => {
            throw this.emit("error", t4), t4;
          }), this.#t;
        }
        /**
         * An alternate form of Service advertisement that supports a modern Promise-based interface for use with async/await.
         * @param callback An asynchronous callback processing the request and returning a response.
         */
        async advertiseAsync(t4) {
          return this.#t = this.#t.then(() => {
            this.isAdvertised && this.#u(), this.#e = (u3) => {
              if (!At(u3))
                throw new Error(
                  `Invalid message received on service channel: ${JSON.stringify(u3)}`
                );
              (async () => {
                try {
                  this.ros.callOnConnection({
                    op: "service_response",
                    service: this.name,
                    result: true,
                    values: await t4(u3.args),
                    id: u3.id
                  });
                } catch (s3) {
                  this.ros.callOnConnection({
                    op: "service_response",
                    service: this.name,
                    result: false,
                    values: String(s3),
                    id: u3.id
                  });
                }
              })().catch(console.error);
            }, this.ros.on(this.name, this.#e), this.ros.callOnConnection({
              op: "advertise_service",
              type: this.serviceType,
              service: this.name
            }), this.isAdvertised = true;
          }).catch((u3) => {
            throw this.emit("error", u3), u3;
          }), this.#t;
        }
      };
      fe = class extends import_index.default {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.name - The topic name, like '/cmd_vel'.
         * @param options.messageType - The message type, like 'std_msgs/String'.
         * @param [options.compression=none] - The type of compression to use, like 'png', 'cbor', or 'cbor-raw'.
         * @param [options.throttle_rate=0] - The rate (in ms in between messages) at which to throttle the topics.
         * @param [options.queue_size=100] - The queue created at bridge side for re-publishing webtopics.
         * @param [options.latch=false] - Latch the topic when publishing.
         * @param [options.queue_length=0] - The queue length at bridge side used when subscribing.
         * @param [options.reconnect_on_close=true] - The flag to enable resubscription and readvertisement on close event.
         */
        constructor({
          ros: t4,
          name: u3,
          messageType: s3,
          compression: c5 = "none",
          throttle_rate: o5 = 0,
          latch: h4 = false,
          queue_size: C2 = 100,
          queue_length: f7 = 0,
          reconnect_on_close: g5 = true
        }) {
          super(), this.waitForReconnect = false, this.reconnectFunc = void 0, this.isAdvertised = false, this.subscribeId = null, this.#e = (D) => {
            if (Bt(D))
              this.emit("message", D.msg);
            else
              throw new Error(
                `Unexpected message on topic channel: ${JSON.stringify(D)}`
              );
          }, this.ros = t4, this.name = u3, this.messageType = s3, this.compression = c5, this.throttle_rate = o5, this.latch = h4, this.queue_size = C2, this.queue_length = f7, this.reconnect_on_close = g5, this.compression && this.compression !== "png" && this.compression !== "cbor" && this.compression !== "cbor-raw" && this.compression !== "none" && (this.emit(
            "warning",
            `${this.compression} compression is not supported. No compression will be used.`
          ), this.compression = "none"), this.throttle_rate < 0 && (this.emit(
            "warning",
            `${this.throttle_rate.toString()} is not allowed. Set to 0`
          ), this.throttle_rate = 0), this.reconnect_on_close ? this.callForSubscribeAndAdvertise = (D) => {
            this.ros.callOnConnection(D), this.waitForReconnect = false, this.reconnectFunc = () => {
              this.waitForReconnect || (this.waitForReconnect = true, this.ros.callOnConnection(D), this.ros.once("connection", () => {
                this.waitForReconnect = false;
              }));
            }, this.ros.on("close", this.reconnectFunc);
          } : this.callForSubscribeAndAdvertise = (D) => {
            this.ros.callOnConnection(D);
          };
        }
        #e;
        /**
         * Every time a message is published for the given topic, the callback
         * will be called with the message object.
         *
         * @param callback - Function with the following params:
         */
        subscribe(t4) {
          this.on("message", t4), !this.subscribeId && (this.ros.on(this.name, this.#e), this.subscribeId = `subscribe:${this.name}:${v4_default()}`, this.callForSubscribeAndAdvertise({
            op: "subscribe",
            id: this.subscribeId,
            type: this.messageType,
            topic: this.name,
            compression: this.compression,
            throttle_rate: this.throttle_rate,
            queue_length: this.queue_length
          }));
        }
        /**
         * Unregister as a subscriber for the topic. Unsubscribing will stop
         * and remove all subscribe callbacks. To remove a callback, you must
         * explicitly pass the callback function in.
         *
         * @param [callback] - The callback to unregister, if
         *     provided and other listeners are registered the topic won't
         *     unsubscribe, just stop emitting to the passed listener.
         */
        unsubscribe(t4) {
          t4 && (this.off("message", t4), this.listeners("message").length) || this.subscribeId && (this.ros.off(this.name, this.#e), this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unsubscribe"), this.ros.callOnConnection({
            op: "unsubscribe",
            id: this.subscribeId,
            topic: this.name
          }), this.subscribeId = null);
        }
        /**
         * Register as a publisher for the topic.
         */
        advertise() {
          this.isAdvertised || (this.advertiseId = `advertise:${this.name}:${v4_default()}`, this.callForSubscribeAndAdvertise({
            op: "advertise",
            id: this.advertiseId,
            type: this.messageType,
            topic: this.name,
            latch: this.latch,
            queue_size: this.queue_size
          }), this.isAdvertised = true, this.reconnect_on_close || this.ros.on("close", () => {
            this.isAdvertised = false;
          }));
        }
        /**
         * Unregister as a publisher for the topic.
         */
        unadvertise() {
          this.isAdvertised && (this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unadvertise"), this.ros.callOnConnection({
            op: "unadvertise",
            id: this.advertiseId,
            topic: this.name
          }), this.isAdvertised = false);
        }
        /**
         * Publish the message.
         *
         * @param message - The message to publish.
         */
        publish(t4) {
          this.isAdvertised || this.advertise(), this.ros.callOnConnection({
            op: "publish",
            id: `publish:${this.name}:${v4_default()}`,
            topic: this.name,
            msg: t4
          });
        }
        /**
         * Retrieves list of publishers for this topic.
         *
         * @param callback - Function with the following params:
         *   * publishers - The list of publishers.
         * @param [failedCallback] - The callback function when the service call failed.
         */
        getPublishers(t4, u3 = console.error) {
          const s3 = new re({
            ros: this.ros,
            name: "/rosapi/publishers",
            serviceType: "rosapi/Publishers"
          }), c5 = {
            topic: this.name
          };
          s3.callService(
            c5,
            function(o5) {
              t4(o5.publishers);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
      };
      Kt = class {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.name - The param name, like max_vel_x.
         */
        constructor({ ros: t4, name: u3 }) {
          this.ros = t4, this.name = u3;
        }
        /**
         * Fetch the value of the param.
         *
         * @param callback - The callback function.
         * @param [failedCallback] - The callback function when the service call failed or the parameter retrieval was unsuccessful.
         */
        get(t4, u3 = console.error) {
          const s3 = new re({
            ros: this.ros,
            name: "rosapi/get_param",
            serviceType: "rosapi/GetParam"
          }), c5 = { name: this.name };
          s3.callService(
            c5,
            function(o5) {
              "successful" in o5 && !o5.successful ? u3(o5.reason) : t4(JSON.parse(o5.value));
            },
            u3
          );
        }
        /**
         * Set the value of the param in ROS.
         *
         * @param value - The value to set param to.
         * @param [callback] - The callback function.
         * @param [failedCallback] - The callback function when the service call failed or the parameter setting was unsuccessful.
         */
        set(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this.ros,
            name: "rosapi/set_param",
            serviceType: "rosapi/SetParam"
          }), o5 = {
            name: this.name,
            value: JSON.stringify(t4)
          };
          c5.callService(
            o5,
            function(h4) {
              "successful" in h4 && !h4.successful ? s3(h4.reason) : u3 && u3(h4);
            },
            s3
          );
        }
        /**
         * Delete this parameter on the ROS server.
         *
         * @param callback - The callback function.
         * @param [failedCallback] - The callback function when the service call failed or the parameter deletion was unsuccessful.
         */
        delete(t4, u3 = console.error) {
          const s3 = new re({
            ros: this.ros,
            name: "rosapi/delete_param",
            serviceType: "rosapi/DeleteParam"
          }), c5 = {
            name: this.name
          };
          s3.callService(
            c5,
            function(o5) {
              "successful" in o5 && !o5.successful ? u3(o5.reason) : t4(o5);
            },
            u3
          );
        }
      };
      qt = class extends import_index.default {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.serverName - The action server name, like '/fibonacci'.
         * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
         * @param [options.timeout] - The timeout length when connecting to the action server.
         * @param [options.omitFeedback] - The flag to indicate whether to omit the feedback channel or not.
         * @param [options.omitStatus] - The flag to indicate whether to omit the status channel or not.
         * @param [options.omitResult] - The flag to indicate whether to omit the result channel or not.
         */
        constructor({
          ros: t4,
          serverName: u3,
          actionName: s3,
          timeout: c5,
          omitFeedback: o5,
          omitStatus: h4,
          omitResult: C2
        }) {
          super(), this.goals = {}, this.receivedStatus = false, this.ros = t4, this.serverName = u3, this.actionName = s3, this.timeout = c5, this.omitFeedback = o5, this.omitStatus = h4, this.omitResult = C2, this.feedbackListener = new fe({
            ros: this.ros,
            name: `${this.serverName}/feedback`,
            messageType: `${this.actionName}Feedback`
          }), this.statusListener = new fe({
            ros: this.ros,
            name: `${this.serverName}/status`,
            messageType: "actionlib_msgs/GoalStatusArray"
          }), this.resultListener = new fe({
            ros: this.ros,
            name: `${this.serverName}/result`,
            messageType: `${this.actionName}Result`
          }), this.goalTopic = new fe({
            ros: this.ros,
            name: `${this.serverName}/goal`,
            messageType: `${this.actionName}Goal`
          }), this.cancelTopic = new fe({
            ros: this.ros,
            name: `${this.serverName}/cancel`,
            messageType: "actionlib_msgs/GoalID"
          }), this.goalTopic.advertise(), this.cancelTopic.advertise(), this.omitStatus || this.statusListener.subscribe((f7) => {
            this.receivedStatus = true, f7.status_list.forEach((g5) => {
              const D = this.goals[g5.goal_id.id];
              D && D.emit("status", g5);
            });
          }), this.omitFeedback || this.feedbackListener.subscribe((f7) => {
            const g5 = this.goals[f7.status.goal_id.id];
            g5 && (g5.emit("status", f7.status), g5.emit("feedback", f7.feedback));
          }), this.omitResult || this.resultListener.subscribe((f7) => {
            const g5 = this.goals[f7.status.goal_id.id];
            g5 && (g5.emit("status", f7.status), g5.emit("result", f7.result));
          }), this.timeout && setTimeout(() => {
            this.receivedStatus || this.emit("timeout");
          }, this.timeout);
        }
        /**
         * Cancel all goals associated with this ActionClient.
         */
        cancel() {
          const t4 = {};
          this.cancelTopic.publish(t4);
        }
        /**
         * Unsubscribe and unadvertise all topics associated with this ActionClient.
         */
        dispose() {
          this.goalTopic.unadvertise(), this.cancelTopic.unadvertise(), this.omitStatus || this.statusListener.unsubscribe(), this.omitFeedback || this.feedbackListener.unsubscribe(), this.omitResult || this.resultListener.unsubscribe();
        }
      };
      er = class extends import_index.default {
        /**
         * @param options
         * @param options.actionClient - The ROSLIB.ActionClient to use with this goal.
         * @param options.goalMessage - The JSON object containing the goal for the action server.
         */
        constructor({
          actionClient: t4,
          goalMessage: u3
        }) {
          super(), this.isFinished = false, this.status = void 0, this.result = void 0, this.feedback = void 0, this.goalID = `goal_${v4_default()}`, this.actionClient = t4, this.goalMessage = {
            goal_id: {
              stamp: {
                secs: 0,
                nsecs: 0
              },
              id: this.goalID
            },
            goal: u3
          }, this.on("status", (s3) => {
            this.status = s3;
          }), this.on("result", (s3) => {
            this.isFinished = true, this.result = s3;
          }), this.on("feedback", (s3) => {
            this.feedback = s3;
          }), this.actionClient.goals[this.goalID] = this;
        }
        /**
         * Send the goal to the action server.
         *
         * @param [timeout] - A timeout length for the goal's result.
         */
        send(t4) {
          this.actionClient.goalTopic.publish(this.goalMessage), t4 && setTimeout(() => {
            this.isFinished || this.emit("timeout");
          }, t4);
        }
        /**
         * Cancel the current goal.
         */
        cancel() {
          const t4 = {
            id: this.goalID
          };
          this.actionClient.cancelTopic.publish(t4);
        }
      };
      Oe = class _Oe {
        constructor(t4) {
          this.x = t4?.x ?? 0, this.y = t4?.y ?? 0, this.z = t4?.z ?? 0;
        }
        /**
         * Set the values of this vector to the sum of itself and the given vector.
         *
         * @param v - The vector to add with.
         */
        add(t4) {
          this.x += t4.x, this.y += t4.y, this.z += t4.z;
        }
        /**
         * Set the values of this vector to the difference of itself and the given vector.
         *
         * @param v - The vector to subtract with.
         */
        subtract(t4) {
          this.x -= t4.x, this.y -= t4.y, this.z -= t4.z;
        }
        /**
         * Multiply the given Quaternion with this vector.
         *
         * @param q - The quaternion to multiply with.
         */
        multiplyQuaternion(t4) {
          const u3 = t4.w * this.x + t4.y * this.z - t4.z * this.y, s3 = t4.w * this.y + t4.z * this.x - t4.x * this.z, c5 = t4.w * this.z + t4.x * this.y - t4.y * this.x, o5 = -t4.x * this.x - t4.y * this.y - t4.z * this.z;
          this.x = u3 * t4.w + o5 * -t4.x + s3 * -t4.z - c5 * -t4.y, this.y = s3 * t4.w + o5 * -t4.y + c5 * -t4.x - u3 * -t4.z, this.z = c5 * t4.w + o5 * -t4.z + u3 * -t4.y - s3 * -t4.x;
        }
        /**
         * Clone a copy of this vector.
         *
         * @returns The cloned vector.
         */
        clone() {
          return new _Oe(this);
        }
      };
      Qe = class _Qe {
        constructor(t4) {
          this.x = t4?.x ?? 0, this.y = t4?.y ?? 0, this.z = t4?.z ?? 0, this.w = typeof t4?.w == "number" ? t4.w : 1;
        }
        /**
         * Perform a conjugation on this quaternion.
         */
        conjugate() {
          this.x *= -1, this.y *= -1, this.z *= -1;
        }
        /**
         * Return the norm of this quaternion.
         */
        norm() {
          return Math.sqrt(
            this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
          );
        }
        /**
         * Perform a normalization on this quaternion.
         */
        normalize() {
          let t4 = Math.sqrt(
            this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
          );
          t4 === 0 ? (this.x = 0, this.y = 0, this.z = 0, this.w = 1) : (t4 = 1 / t4, this.x = this.x * t4, this.y = this.y * t4, this.z = this.z * t4, this.w = this.w * t4);
        }
        /**
         * Convert this quaternion into its inverse.
         */
        invert() {
          this.conjugate(), this.normalize();
        }
        /**
         * Set the values of this quaternion to the product of itself and the given quaternion.
         *
         * @param q - The quaternion to multiply with.
         */
        multiply(t4) {
          const u3 = this.x * t4.w + this.y * t4.z - this.z * t4.y + this.w * t4.x, s3 = -this.x * t4.z + this.y * t4.w + this.z * t4.x + this.w * t4.y, c5 = this.x * t4.y - this.y * t4.x + this.z * t4.w + this.w * t4.z, o5 = -this.x * t4.x - this.y * t4.y - this.z * t4.z + this.w * t4.w;
          this.x = u3, this.y = s3, this.z = c5, this.w = o5;
        }
        /**
         * Clone a copy of this quaternion.
         *
         * @returns The cloned quaternion.
         */
        clone() {
          return new _Qe(this);
        }
      };
      vt = class _vt {
        constructor(t4) {
          this.translation = new Oe(t4.translation), this.rotation = new Qe(t4.rotation);
        }
        /**
         * Clone a copy of this transform.
         *
         * @returns The cloned transform.
         */
        clone() {
          return new _vt(this);
        }
      };
      Gt = class {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param [options.fixedFrame=base_link] - The fixed frame.
         * @param [options.angularThres=2.0] - The angular threshold for the TF republisher.
         * @param [options.transThres=0.01] - The translation threshold for the TF republisher.
         * @param [options.rate=10.0] - The rate for the TF republisher.
         * @param [options.updateDelay=50] - The time (in ms) to wait after a new subscription
         *     to update the TF republisher's list of TFs.
         * @param [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
         * @param [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
         */
        constructor({
          ros: t4,
          fixedFrame: u3 = "base_link",
          angularThres: s3 = 2,
          transThres: c5 = 0.01,
          rate: o5 = 10,
          updateDelay: h4 = 50,
          topicTimeout: C2 = 2,
          serverName: f7 = "/tf2_web_republisher"
        }) {
          this.frameInfos = {}, this.republisherUpdateRequested = false, this.ros = t4, this.fixedFrame = u3, this.angularThres = s3, this.transThres = c5, this.rate = o5, this.updateDelay = h4;
          const g5 = C2, D = Math.floor(g5), p5 = Math.floor((g5 - D) * 1e9);
          this.topicTimeout = {
            secs: D,
            nsecs: p5
          }, this.serverName = f7;
        }
        /**
         * Process the incoming TF message and send them out using the callback
         * functions.
         *
         * @param tf - The TF message from the server.
         */
        processTFArray(t4) {
          t4.transforms.forEach((u3) => {
            let s3 = u3.child_frame_id;
            s3.startsWith("/") && (s3 = s3.substring(1));
            const c5 = this.frameInfos[s3];
            if (c5) {
              const o5 = new vt({
                translation: u3.transform.translation,
                rotation: u3.transform.rotation
              });
              c5.transform = o5, c5.cbs.forEach((h4) => {
                h4(o5);
              });
            }
          }, this);
        }
        /**
         * Create and send a new goal (or service request) to the tf2_web_republisher
         * based on the current list of TFs.
         * This method should be overridden by subclasses.
         */
        updateGoal() {
          throw new Error("updateGoal() must be implemented by subclass");
        }
        /**
         * Subscribe to the given TF frame.
         *
         * @param frameID - The TF frame to subscribe to.
         * @param callback - Function with the following params:
         */
        subscribe(t4, u3) {
          t4.startsWith("/") && (t4 = t4.substring(1)), this.frameInfos[t4] || (this.frameInfos[t4] = {
            cbs: []
          }, this.republisherUpdateRequested || (setTimeout(() => {
            this.updateGoal();
          }, this.updateDelay), this.republisherUpdateRequested = true));
          const s3 = this.frameInfos[t4]?.transform;
          s3 && u3(s3), this.frameInfos[t4]?.cbs.push(u3);
        }
        /**
         * Unsubscribe from the given TF frame.
         *
         * @param frameID - The TF frame to unsubscribe from.
         * @param [callback] - The callback function to remove.
         */
        unsubscribe(t4, u3) {
          t4.startsWith("/") && (t4 = t4.substring(1));
          const s3 = this.frameInfos[t4];
          for (var c5 = s3?.cbs ?? [], o5 = c5.length; o5--; )
            c5[o5] === u3 && c5.splice(o5, 1);
          (!u3 || c5.length === 0) && delete this.frameInfos[t4];
        }
      };
      tr = class extends Gt {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param [options.fixedFrame=base_link] - The fixed frame.
         * @param [options.angularThres=2.0] - The angular threshold for the TF republisher.
         * @param [options.transThres=0.01] - The translation threshold for the TF republisher.
         * @param [options.rate=10.0] - The rate for the TF republisher.
         * @param [options.updateDelay=50] - The time (in ms) to wait after a new subscription
         *     to update the TF republisher's list of TFs.
         * @param [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
         * @param [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
         */
        constructor(t4) {
          super(t4), this.currentGoal = false, this.currentTopic = false, this.#e = void 0, this.#t = false, this.actionClient = new qt({
            ros: this.ros,
            serverName: this.serverName,
            actionName: "tf2_web_republisher/TFSubscriptionAction",
            omitStatus: true,
            omitResult: true
          });
        }
        #e;
        #t;
        /**
         * Create and send a new goal (or service request) to the tf2_web_republisher
         * based on the current list of TFs.
         */
        updateGoal() {
          const t4 = {
            source_frames: Object.keys(this.frameInfos),
            target_frame: this.fixedFrame,
            angular_thres: this.angularThres,
            trans_thres: this.transThres,
            rate: this.rate
          };
          this.currentGoal && this.currentGoal.cancel(), this.currentGoal = new er({
            actionClient: this.actionClient,
            goalMessage: t4
          }), this.currentGoal.on("feedback", (u3) => {
            this.processTFArray(u3);
          }), this.currentGoal.send(), this.republisherUpdateRequested = false;
        }
        /**
         * Process the service response and subscribe to the tf republisher
         * topic.
         *
         * @param response - The service response containing the topic name.
         */
        processResponse(t4) {
          this.#t || (this.currentTopic && this.currentTopic.unsubscribe(this.#e), this.currentTopic = new fe({
            ros: this.ros,
            name: t4.topic_name,
            messageType: "tf2_web_republisher/TFArray"
          }), this.#e = (u3) => {
            this.processTFArray(u3);
          }, this.currentTopic.subscribe(this.#e));
        }
        /**
         * Unsubscribe and unadvertise all topics associated with this TFClient.
         */
        dispose() {
          this.#t = true, this.actionClient.dispose(), this.currentTopic && this.currentTopic.unsubscribe(this.#e);
        }
      };
      rr = class extends import_index.default {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.serverName - The action server name, like '/fibonacci'.
         * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
         */
        constructor({
          ros: t4,
          serverName: u3,
          actionName: s3
        }) {
          super(), this.currentGoal = null, this.nextGoal = null, this.ros = t4, this.serverName = u3, this.actionName = s3, this.feedbackPublisher = new fe({
            ros: this.ros,
            name: `${this.serverName}/feedback`,
            messageType: `${this.actionName}Feedback`
          }), this.feedbackPublisher.advertise();
          const c5 = new fe({
            ros: this.ros,
            name: `${this.serverName}/status`,
            messageType: "actionlib_msgs/GoalStatusArray"
          });
          c5.advertise(), this.resultPublisher = new fe({
            ros: this.ros,
            name: `${this.serverName}/result`,
            messageType: `${this.actionName}Result`
          }), this.resultPublisher.advertise();
          const o5 = new fe({
            ros: this.ros,
            name: `${this.serverName}/goal`,
            messageType: `${this.actionName}Goal`
          }), h4 = new fe({
            ros: this.ros,
            name: `${this.serverName}/cancel`,
            messageType: "actionlib_msgs/GoalID"
          });
          this.statusMessage = {
            header: {
              stamp: { secs: 0, nsecs: 100 },
              frame_id: ""
            },
            /** @type {{goal_id: any, status: number}[]} */
            status_list: []
          }, o5.subscribe((f7) => {
            this.currentGoal ? (this.nextGoal = f7, this.emit("cancel")) : (this.statusMessage.status_list = [
              { goal_id: f7.goal_id, status: 1 }
            ], this.currentGoal = f7, this.emit("goal", f7.goal));
          });
          const C2 = function(f7, g5) {
            return f7.secs > g5.secs ? false : f7.secs < g5.secs ? true : f7.nsecs < g5.nsecs;
          };
          h4.subscribe((f7) => {
            f7.stamp.secs === 0 && f7.stamp.nsecs === 0 && f7.id === "" ? (this.nextGoal = null, this.currentGoal && this.emit("cancel")) : (f7.id === this.currentGoal?.goal_id.id ? this.emit("cancel") : f7.id === this.nextGoal?.goal_id.id && (this.nextGoal = null), this.nextGoal && C2(this.nextGoal.goal_id.stamp, f7.stamp) && (this.nextGoal = null), this.currentGoal && C2(this.currentGoal.goal_id.stamp, f7.stamp) && this.emit("cancel"));
          }), setInterval(() => {
            const f7 = /* @__PURE__ */ new Date(), g5 = Math.floor(f7.getTime() / 1e3), D = Math.round(
              1e9 * (f7.getTime() / 1e3 - g5)
            );
            this.statusMessage.header = {
              ...this.statusMessage.header,
              stamp: { secs: g5, nsecs: D }
            }, c5.publish(this.statusMessage);
          }, 500);
        }
        /**
         * Set action state to succeeded and return to client.
         *
         * @param result - The result to return to the client.
         */
        setSucceeded(t4) {
          if (this.currentGoal !== null) {
            const u3 = {
              status: { goal_id: this.currentGoal.goal_id, status: 3 },
              result: t4
            };
            this.resultPublisher.publish(u3), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
          }
        }
        /**
         * Set action state to aborted and return to client.
         *
         * @param result - The result to return to the client.
         */
        setAborted(t4) {
          if (this.currentGoal !== null) {
            const u3 = {
              status: { goal_id: this.currentGoal.goal_id, status: 4 },
              result: t4
            };
            this.resultPublisher.publish(u3), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
          }
        }
        /**
         * Send a feedback message.
         *
         * @param feedback - The feedback to send to the client.
         */
        sendFeedback(t4) {
          if (this.currentGoal !== null) {
            const u3 = {
              status: { goal_id: this.currentGoal.goal_id, status: 1 },
              feedback: t4
            };
            this.feedbackPublisher.publish(u3);
          }
        }
        /**
         * Handle case where client requests preemption.
         */
        setPreempted() {
          if (this.currentGoal !== null) {
            this.statusMessage.status_list = [];
            const t4 = {
              status: { goal_id: this.currentGoal.goal_id, status: 2 }
            };
            this.resultPublisher.publish(t4), this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
          }
        }
      };
      ur = async (n2) => {
        if (typeof WebSocket == "function") {
          const o5 = await Promise.resolve().then(() => (init_NativeWebSocketTransport_CF_ebnyS(), NativeWebSocketTransport_CF_ebnyS_exports)), { NativeWebSocketTransport: h4 } = o5, C2 = new WebSocket(n2);
          return C2.binaryType = "arraybuffer", new h4(C2);
        }
        const t4 = await Promise.resolve().then(() => __toESM(require_browser(), 1)), u3 = await Promise.resolve().then(() => (init_WsWebSocketTransport_6_v9C0gj(), WsWebSocketTransport_6_v9C0gj_exports)), { WsWebSocketTransport: s3 } = u3, c5 = new t4.WebSocket(n2);
        return c5.binaryType = "arraybuffer", new s3(c5);
      };
      Lr = class extends import_index.default {
        // private write, public read via getter method
        #e;
        constructor({
          url: t4,
          transportFactory: u3 = ur
        } = {}) {
          super(), this.#e = false, this.transportFactory = u3, t4 && this.connect(t4).catch(console.error);
        }
        get isConnected() {
          return this.#e;
        }
        async connect(t4) {
          if (this.transport && !this.transport.isClosed())
            return;
          const u3 = await this.transportFactory(t4);
          this.transport = u3, u3.on("open", (s3) => {
            this.#e = true, this.emit("connection", s3);
          }), u3.on("close", (s3) => {
            this.#e = false, this.emit("close", s3);
          }), u3.on("error", (s3) => {
            this.emit("error", s3);
          }), u3.on("message", (s3) => {
            this.handleMessage(s3);
          });
        }
        close() {
          this.transport?.close();
        }
        handleMessage(t4) {
          Bt(t4) ? this.emit(t4.topic, t4) : Ft(t4) ? t4.id ? this.emit(t4.id, t4) : console.error("Received service response without ID") : At(t4) ? this.emit(t4.service, t4) : Lt(t4) ? this.emit(t4.action, t4) : Pt(t4) ? this.emit(t4.id, t4) : kt(t4) ? this.emit(t4.id, t4) : Ut(t4) ? this.emit(t4.id, t4) : Qt(t4) && (t4.id ? this.emit(`status:${t4.id}`, t4) : this.emit("status", t4));
        }
        /**
         * Send an authorization request to the server.
         *
         * @param mac - MAC (hash) string given by the trusted source.
         * @param client - IP of the client.
         * @param dest - IP of the destination.
         * @param rand - Random string given by the trusted source.
         * @param t - Time of the authorization request.
         * @param level - User level as a string given by the client.
         * @param end - End time of the client's session.
         */
        authenticate(t4, u3, s3, c5, o5, h4, C2) {
          this.callOnConnection({
            op: "auth",
            mac: t4,
            client: u3,
            dest: s3,
            rand: c5,
            t: o5,
            level: h4,
            end: C2
          });
        }
        /**
         * Sends the message to the transport.
         * If not connected, queues the message to send once reconnected.
         */
        callOnConnection(t4) {
          this.isConnected ? this.transport?.send(t4) : this.once("connection", () => {
            this.transport?.send(t4);
          });
        }
        /**
         * Send a set_level request to the server.
         *
         * @param level - Status level (none, error, warning, info).
         * @param [id] - Operation ID to change status level on.
         */
        setStatusLevel(t4, u3) {
          const s3 = {
            op: "set_level",
            level: t4,
            id: u3
          };
          this.callOnConnection(s3);
        }
        /**
         * Retrieve a list of action servers in ROS as an array of string.
         *
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getActionServers(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/action_servers",
            serviceType: "rosapi/GetActionServers"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5.action_servers);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        /**
         * Retrieve a list of topics in ROS as an array.
         *
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getTopics(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/topics",
            serviceType: "rosapi/Topics"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        /**
         * Retrieve a list of topics in ROS as an array of a specific type.
         *
         * @param topicType - The topic type to find.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getTopicsForType(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/topics_for_type",
            serviceType: "rosapi/TopicsForType"
          }), o5 = {
            type: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4.topics);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve a list of active service names in ROS.
         *
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getServices(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/services",
            serviceType: "rosapi/Services"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5.services);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        /**
         * Retrieve a list of services in ROS as an array as specific type.
         *
         * @param serviceType - The service type to find.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getServicesForType(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/services_for_type",
            serviceType: "rosapi/ServicesForType"
          }), o5 = {
            type: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4.services);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve the details of a ROS service request.
         *
         * @param type - The type of the service.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getServiceRequestDetails(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/service_request_details",
            serviceType: "rosapi/ServiceRequestDetails"
          }), o5 = {
            type: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve the details of a ROS service response.
         *
         * @param type - The type of the service.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getServiceResponseDetails(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/service_response_details",
            serviceType: "rosapi/ServiceResponseDetails"
          }), o5 = {
            type: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve a list of active node names in ROS.
         *
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getNodes(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/nodes",
            serviceType: "rosapi/Nodes"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5.nodes);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        /**
         * Retrieve a list of subscribed topics, publishing topics and services of a specific node.
         *
         * @param node - Name of the node.
         */
        getNodeDetails(t4, u3, s3 = console.error) {
          new re({
            ros: this,
            name: "rosapi/node_details",
            serviceType: "rosapi/NodeDetails"
          }).callService({ node: t4 }, u3, s3);
        }
        /**
         * Retrieve a list of parameter names from the ROS Parameter Server.
         *
         * @param callback - Function with the following params:
         * @param failedCallback - The callback function when the service call failed with params:
         */
        getParams(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/get_param_names",
            serviceType: "rosapi/GetParamNames"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5.names);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        /**
         * Retrieve the type of a ROS topic.
         *
         * @param topic - Name of the topic.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getTopicType(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/topic_type",
            serviceType: "rosapi/TopicType"
          }), o5 = {
            topic: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4.type);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve the type of a ROS service.
         *
         * @param service - Name of the service.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getServiceType(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/service_type",
            serviceType: "rosapi/ServiceType"
          }), o5 = {
            service: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4.type);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Retrieve the details of a ROS message.
         *
         * @param message - The name of the message type.
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getMessageDetails(t4, u3, s3 = console.error) {
          const c5 = new re({
            ros: this,
            name: "rosapi/message_details",
            serviceType: "rosapi/MessageDetails"
          }), o5 = {
            type: t4
          };
          c5.callService(
            o5,
            function(h4) {
              u3(h4.typedefs);
            },
            function(h4) {
              s3(h4);
            }
          );
        }
        /**
         * Decode a typedef array into a dictionary like `rosmsg show foo/bar`.
         *
         * @param defs - Array of type_def dictionary.
         */
        decodeTypeDefs(t4) {
          const u3 = (s3, c5) => {
            const o5 = {};
            for (let h4 = 0; h4 < s3.fieldnames.length; h4++) {
              const C2 = s3.fieldarraylen[h4], f7 = s3.fieldnames[h4], g5 = s3.fieldtypes[h4];
              if (f7 === void 0 || g5 === void 0)
                throw new Error(
                  "Received mismatched type definition vector lengths!"
                );
              if (!g5.includes("/"))
                C2 === -1 ? o5[f7] = g5 : o5[f7] = [g5];
              else {
                let D;
                for (const p5 of c5)
                  if (p5.type === g5) {
                    D = p5;
                    break;
                  }
                if (D) {
                  const p5 = u3(D, c5);
                  C2 === -1 ? o5[f7] = p5 : o5[f7] = [p5];
                } else
                  this.emit("error", `Cannot find ${g5} in decodeTypeDefs`);
              }
            }
            return o5;
          };
          return t4[0] ? u3(t4[0], t4) : {};
        }
        /**
         * @callback getTopicsAndRawTypesCallback
         * @param {Object} result - The result object with the following params:
         * @param {string[]} result.topics - Array of topic names.
         * @param {string[]} result.types - Array of message type names.
         * @param {string[]} result.typedefs_full_text - Array of full definitions of message types, similar to `gendeps --cat`.
         */
        /**
         * @callback getTopicsAndRawTypesFailedCallback
         * @param {string} error - The error message reported by ROS.
         */
        /**
         * Retrieve a list of topics and their associated type definitions.
         *
         * @param callback - Function with the following params:
         * @param [failedCallback] - The callback function when the service call failed with params:
         */
        getTopicsAndRawTypes(t4, u3 = console.error) {
          const s3 = new re({
            ros: this,
            name: "rosapi/topics_and_raw_types",
            serviceType: "rosapi/TopicsAndRawTypes"
          }), c5 = {};
          s3.callService(
            c5,
            function(o5) {
              t4(o5);
            },
            function(o5) {
              u3(o5);
            }
          );
        }
        Topic(t4) {
          return new fe({ ros: this, ...t4 });
        }
        Param(t4) {
          return new Kt({ ros: this, ...t4 });
        }
        Service(t4) {
          return new re({ ros: this, ...t4 });
        }
        TFClient(t4) {
          return new tr({ ros: this, ...t4 });
        }
        ActionClient(t4) {
          return new qt({
            ros: this,
            ...t4
          });
        }
        SimpleActionServer(t4) {
          return new rr({
            ros: this,
            ...t4
          });
        }
      };
      Be = /* @__PURE__ */ ((n2) => (n2[n2.STATUS_UNKNOWN = 0] = "STATUS_UNKNOWN", n2[n2.STATUS_ACCEPTED = 1] = "STATUS_ACCEPTED", n2[n2.STATUS_EXECUTING = 2] = "STATUS_EXECUTING", n2[n2.STATUS_CANCELING = 3] = "STATUS_CANCELING", n2[n2.STATUS_SUCCEEDED = 4] = "STATUS_SUCCEEDED", n2[n2.STATUS_CANCELED = 5] = "STATUS_CANCELED", n2[n2.STATUS_ABORTED = 6] = "STATUS_ABORTED", n2))(Be || {});
      Nt = class extends Error {
        constructor(t4, u3) {
          super(`${ir(t4)}${u3 ? `: ${u3}` : ""}`), this.name = "GoalError";
        }
      };
      nr = class {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.name - The action name, like '/fibonacci'.
         * @param options.actionType - The action type, like 'example_interfaces/Fibonacci'.
         */
        constructor({
          ros: t4,
          name: u3,
          actionType: s3
        }) {
          this.isAdvertised = false, this.#e = null, this.#t = null, this.ros = t4, this.name = u3, this.actionType = s3;
        }
        #e;
        #t;
        /**
         * Sends an action goal. Returns the feedback in the feedback callback while the action is running
         * and the result in the result callback when the action is completed.
         * Does nothing if this action is currently advertised.
         *
         * @param goal - The action goal to send.
         * @param resultCallback - The callback function when the action is completed.
         * @param [feedbackCallback] - The callback function when the action publishes feedback.
         * @param [failedCallback] - The callback function when the action failed.
         */
        sendGoal(t4, u3, s3, c5 = console.error) {
          if (this.isAdvertised)
            return;
          const o5 = `send_action_goal:${this.name}:${v4_default()}`;
          return this.ros.on(o5, (h4) => {
            if (Ut(h4)) {
              const C2 = h4.status;
              h4.result ? C2 !== Be.STATUS_SUCCEEDED ? c5(
                String(new Nt(C2, JSON.stringify(h4.values)))
              ) : u3(h4.values) : c5(String(new Nt(C2, h4.values)));
            } else kt(h4) && s3?.(h4.values);
          }), this.ros.callOnConnection({
            op: "send_action_goal",
            id: o5,
            action: this.name,
            action_type: this.actionType,
            args: t4,
            feedback: true
          }), o5;
        }
        /**
         * Cancels an action goal.
         *
         * @param id - The ID of the action goal to cancel.
         */
        cancelGoal(t4) {
          this.ros.callOnConnection({
            op: "cancel_action_goal",
            id: t4,
            action: this.name
          });
        }
        /**
         * Cancels all action goals.
         */
        cancelAllGoals() {
          this.ros.callOnConnection({
            op: "call_service",
            service: `${this.name}/_action/cancel_goal`,
            args: {}
          });
        }
        /**
         * Advertise the action. This turns the Action object from a client
         * into a server. The callback will be called with every goal sent to this action.
         *
         * @param actionCallback - This works similarly to the callback for a C++ action.
         * @param cancelCallback - A callback function to execute when the action is canceled.
         */
        advertise(t4, u3) {
          this.isAdvertised || typeof t4 != "function" || (this.#e = t4, this.#t = u3, this.ros.on(this.name, (s3) => {
            if (Lt(s3))
              this.#r(s3);
            else
              throw new Error(
                "Received unrelated message on Action server event stream!"
              );
          }), this.ros.callOnConnection({
            op: "advertise_action",
            type: this.actionType,
            action: this.name
          }), this.isAdvertised = true);
        }
        /**
         * Unadvertise a previously advertised action.
         */
        unadvertise() {
          this.isAdvertised && (this.ros.callOnConnection({
            op: "unadvertise_action",
            action: this.name
          }), this.isAdvertised = false);
        }
        /**
         * Helper function that executes an action by calling the provided
         * action callback with the auto-generated ID as a user-accessible input.
         * Should not be called manually.
         *
         * @param rosbridgeRequest - The rosbridge request containing the action goal to send and its ID.
         * @param rosbridgeRequest.id - The ID of the action goal.
         * @param rosbridgeRequest.args - The arguments of the action goal.
         */
        #r(t4) {
          const u3 = t4.id;
          if (typeof u3 == "string" && this.ros.on(u3, (s3) => {
            Pt(s3) && this.#t && this.#t(u3);
          }), this.#e)
            if (t4.args)
              this.#e(t4.args, u3);
            else
              throw new Error(
                "Received Action goal with no arguments! This should never happen, because rosbridge should fill in blanks!"
              );
        }
        /**
         * Helper function to send action feedback inside an action handler.
         *
         * @param id - The action goal ID.
         * @param feedback - The feedback to send.
         */
        sendFeedback(t4, u3) {
          this.ros.callOnConnection({
            op: "action_feedback",
            id: t4,
            action: this.name,
            values: u3
          });
        }
        /**
         * Helper function to set an action as succeeded.
         *
         * @param id - The action goal ID.
         * @param result - The result to set.
         */
        setSucceeded(t4, u3) {
          this.ros.callOnConnection({
            op: "action_result",
            id: t4,
            action: this.name,
            values: u3,
            status: Be.STATUS_SUCCEEDED,
            result: true
          });
        }
        /**
         * Helper function to set an action as canceled.
         *
         * @param id - The action goal ID.
         * @param result - The result to set.
         */
        setCanceled(t4, u3) {
          this.ros.callOnConnection({
            op: "action_result",
            id: t4,
            action: this.name,
            values: u3,
            status: Be.STATUS_CANCELED,
            result: true
          });
        }
        /**
         * Helper function to set an action as failed.
         *
         * @param id - The action goal ID.
         */
        setFailed(t4) {
          this.ros.callOnConnection({
            op: "action_result",
            id: t4,
            action: this.name,
            status: Be.STATUS_ABORTED,
            result: false
          });
        }
      };
      sr = new TextDecoder();
      Pr = class extends import_index.default {
        /**
         * Buffer Map for incoming message fragments.
         */
        #e = /* @__PURE__ */ new Map();
        /**
         * Decodes a raw message received from the transport
         * and emits it as a RosbridgeMessage over the "message" event.
         * If an error occurs, it is emitted as an "error" event.
         *
         * The default implementation handles multiple compression formats
         * and fragment messages. Subclasses may override this method to provide
         * custom handling of raw messages and when to emit messages.
         */
        handleRawMessage(t4) {
          try {
            ut(t4) ? this.handleRosbridgeMessage(t4) : typeof Blob < "u" && t4 instanceof Blob ? this.handleBsonMessage(t4) : t4 instanceof ArrayBuffer ? this.handleCborMessage(t4) : this.handleJsonMessage(String(t4));
          } catch (u3) {
            this.emit("error", u3);
          }
        }
        /**
         * Handles a RosbridgeMessage.
         * If the message is a fragment, it is appended to the fragment buffer.
         * If the message is a PNG, it is decompressed and reprocessed.
         * Otherwise, the message is emitted.
         */
        handleRosbridgeMessage(t4) {
          Jt(t4) ? this.handleRosbridgeFragmentMessage(t4) : Zt(t4) ? this.handleRosbridgePngMessage(t4) : this.emit("message", t4);
        }
        /**
         * Appends a fragment to the current fragment buffer for the message id.
         * If all fragments are received, the message is reconstructed and processed.
         */
        handleRosbridgeFragmentMessage(t4) {
          const { id: u3, data: s3, num: c5, total: o5 } = t4;
          if (!u3 || typeof c5 != "number" || typeof o5 != "number" || typeof s3 != "string")
            return;
          const h4 = Math.floor(o5);
          this.#e.has(u3) || this.#e.set(u3, {
            fragments: [],
            received: 0,
            total: h4
          });
          const C2 = this.#e.get(u3);
          if (!C2)
            throw new Error(`Fragment buffer entry missing for id: ${u3}`);
          if (c5 < h4 && typeof C2.fragments[c5] > "u" && (C2.fragments[c5] = s3, C2.received++), C2.received === h4) {
            const f7 = C2.fragments.join("");
            let g5;
            try {
              g5 = JSON.parse(f7);
            } catch (D) {
              throw new Error("Fragments did not form a valid JSON message!", {
                cause: D
              });
            } finally {
              this.#e.delete(u3);
            }
            if (ut(g5))
              this.handleRosbridgeMessage(g5);
            else
              throw new Error("Received invalid rosbridge message!");
          }
        }
        /**
         * Decompresses a PNG image expecting the result to be a RosbridgeMessage.
         * It is one technique for compressing JSON data.
         */
        handleRosbridgePngMessage(t4) {
          const u3 = ar(t4.data);
          if (ut(u3))
            this.handleRosbridgeMessage(u3);
          else
            throw new Error("Decompressed PNG data was invalid!");
        }
        /**
         * Deserializes a Blob of BSON expecting the result to be a RosbridgeMessage.
         * It is one technique for compressing JSON data.
         */
        handleBsonMessage(t4) {
          const u3 = new FileReader();
          u3.onload = () => {
            if (u3.result instanceof ArrayBuffer) {
              const s3 = new Uint8Array(u3.result), c5 = deserialize(s3);
              ut(c5) ? this.handleRosbridgeMessage(c5) : this.emit("error", new Error("Decoded BSON data was invalid!"));
            }
          }, u3.readAsArrayBuffer(t4);
        }
        /**
         * Deserializes an ArrayBuffer of CBOR expecting the result to be a RosbridgeMessage.
         * It is one technique for compressing JSON data.
         */
        handleCborMessage(t4) {
          const u3 = l4(new Uint8Array(t4));
          if (ut(u3))
            this.handleRosbridgeMessage(u3);
          else
            throw new Error("Decoded CBOR data was invalid!");
        }
        /**
         * Deserializes a JSON string expecting the result to be a RosbridgeMessage.
         */
        handleJsonMessage(t4) {
          const u3 = JSON.parse(t4);
          if (ut(u3))
            this.handleRosbridgeMessage(u3);
          else
            throw new Error("Received invalid rosbridge message!");
        }
      };
      kr = class extends import_index.default {
        /**
         * @param options
         * @param options.ros - The ROSLIB.Ros connection handle.
         * @param options.serverName - The action server name, like '/fibonacci'.
         * @param options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
         */
        constructor({
          ros: t4,
          serverName: u3,
          actionName: s3
        }) {
          super(), this.ros = t4, this.serverName = u3, this.actionName = s3;
          const c5 = new fe({
            ros: this.ros,
            name: `${this.serverName}/goal`,
            messageType: `${this.actionName}Goal`
          }), o5 = new fe({
            ros: this.ros,
            name: `${this.serverName}/feedback`,
            messageType: `${this.actionName}Feedback`
          }), h4 = new fe({
            ros: this.ros,
            name: `${this.serverName}/status`,
            messageType: "actionlib_msgs/GoalStatusArray"
          }), C2 = new fe({
            ros: this.ros,
            name: `${this.serverName}/result`,
            messageType: `${this.actionName}Result`
          });
          c5.subscribe((f7) => {
            this.emit("goal", f7);
          }), h4.subscribe((f7) => {
            f7.status_list.forEach((g5) => {
              this.emit("status", g5);
            });
          }), o5.subscribe((f7) => {
            this.emit("status", f7.status), this.emit("feedback", f7.feedback);
          }), C2.subscribe((f7) => {
            this.emit("status", f7.status), this.emit("result", f7.result);
          });
        }
      };
      pt = class _pt {
        constructor(t4) {
          this.position = new Oe(t4?.position), this.orientation = new Qe(t4?.orientation);
        }
        /**
         * Apply a transform against this pose.
         *
         * @param tf - The transform to be applied.
         */
        applyTransform(t4) {
          this.position.multiplyQuaternion(t4.rotation), this.position.add(t4.translation);
          const u3 = new Qe(t4.rotation);
          u3.multiply(this.orientation), this.orientation = u3;
        }
        /**
         * Clone a copy of this pose.
         *
         * @returns The cloned pose.
         */
        clone() {
          return new _pt(this);
        }
        /**
         * Multiply this pose with another pose without altering this pose.
         *
         * @returns The result of the multiplication.
         */
        multiply(t4) {
          const u3 = t4.clone();
          return u3.applyTransform({
            rotation: this.orientation,
            translation: this.position
          }), u3;
        }
        /**
         * Compute the inverse of this pose.
         *
         * @returns The inverse of the pose.
         */
        getInverse() {
          const t4 = this.clone();
          return t4.orientation.invert(), t4.position.multiplyQuaternion(t4.orientation), t4.position.x *= -1, t4.position.y *= -1, t4.position.z *= -1, t4;
        }
      };
      Ur = class extends Gt {
        constructor(t4) {
          super(t4), this.goal_id = "", this.actionClient = new nr({
            ros: this.ros,
            name: this.serverName,
            actionType: "tf2_web_republisher_interfaces/TFSubscription"
          });
        }
        /**
         * Create and send a new goal (or service request) to the tf2_web_republisher
         * based on the current list of TFs.
         */
        updateGoal() {
          const t4 = {
            source_frames: Object.keys(this.frameInfos),
            target_frame: this.fixedFrame,
            angular_thres: this.angularThres,
            trans_thres: this.transThres,
            rate: this.rate
          };
          this.goal_id !== "" && this.actionClient.cancelGoal(this.goal_id), this.currentGoal = t4;
          const u3 = this.actionClient.sendGoal(
            t4,
            () => {
            },
            (s3) => {
              this.processTFArray(s3);
            }
          );
          typeof u3 == "string" && (this.goal_id = u3), this.republisherUpdateRequested = false;
        }
        /**
         * Unsubscribe and unadvertise all topics associated with this TFClient.
         */
        dispose() {
          this.goal_id !== "" && this.actionClient.cancelGoal(this.goal_id);
        }
      };
      ft = /* @__PURE__ */ ((n2) => (n2[n2.SPHERE = 0] = "SPHERE", n2[n2.BOX = 1] = "BOX", n2[n2.CYLINDER = 2] = "CYLINDER", n2[n2.MESH = 3] = "MESH", n2))(ft || {});
      H3 = /* @__PURE__ */ ((n2) => (n2.Name = "name", n2.Type = "type", n2.Parent = "parent", n2.Link = "link", n2.Child = "child", n2.Limit = "limit", n2.Upper = "upper", n2.Lower = "lower", n2.Origin = "origin", n2.Xyz = "xyz", n2.Rpy = "rpy", n2.Size = "size", n2.Rgba = "rgba", n2.Length = "length", n2.Radius = "radius", n2.Visuals = "visual", n2.Texture = "texture", n2.Filename = "filename", n2.Color = "color", n2.Geometry = "geometry", n2.Material = "material", n2.Scale = "scale", n2.Axis = "axis", n2))(H3 || {});
      cr = class {
        constructor({ xml: t4 }) {
          this.dimension = null, this.type = ft.BOX;
          const u3 = t4.getAttribute(H3.Size)?.split(" ");
          u3?.[0] && u3[1] && u3[2] && (this.dimension = new Oe({
            x: parseFloat(u3[0]),
            y: parseFloat(u3[1]),
            z: parseFloat(u3[2])
          }));
        }
      };
      lr = class {
        constructor({ xml: t4 }) {
          this.r = 0, this.g = 0, this.b = 0, this.a = 1;
          const u3 = t4.getAttribute(H3.Rgba)?.split(" ");
          u3?.[0] && u3[1] && u3[2] && u3[3] && (this.r = parseFloat(u3[0]), this.g = parseFloat(u3[1]), this.b = parseFloat(u3[2]), this.a = parseFloat(u3[3]));
        }
      };
      hr = class {
        constructor({ xml: t4 }) {
          this.type = ft.CYLINDER, this.length = parseFloat(t4.getAttribute(H3.Length) ?? "NaN"), this.radius = parseFloat(t4.getAttribute(H3.Radius) ?? "NaN");
        }
      };
      Vt = class {
        constructor({ xml: t4 }) {
          this.textureFilename = null, this.color = null, this.name = t4.getAttribute(H3.Name) ?? "unknown_name";
          const u3 = t4.getElementsByTagName(H3.Texture);
          u3[0] && (this.textureFilename = u3[0].getAttribute(H3.Filename));
          const s3 = t4.getElementsByTagName(H3.Color);
          s3[0] && (this.color = new lr({
            xml: s3[0]
          }));
        }
        isLink() {
          return this.color === null && this.textureFilename === null;
        }
        assign(t4) {
          return Object.assign(this, t4);
        }
      };
      pr = class {
        constructor({ xml: t4 }) {
          this.scale = null, this.type = ft.MESH, this.filename = t4.getAttribute(H3.Filename);
          const u3 = t4.getAttribute(H3.Scale)?.split(" ");
          u3?.[0] && u3[1] && u3[2] && (this.scale = new Oe({
            x: parseFloat(u3[0]),
            y: parseFloat(u3[1]),
            z: parseFloat(u3[2])
          }));
        }
      };
      fr = class {
        constructor({ xml: t4 }) {
          this.radius = NaN, this.type = ft.SPHERE, this.radius = parseFloat(t4.getAttribute(H3.Radius) ?? "NaN");
        }
      };
      dr = class {
        constructor({ xml: t4 }) {
          this.origin = new pt(), this.geometry = null, this.material = null, this.name = t4.getAttribute(H3.Name);
          const u3 = t4.getElementsByTagName(H3.Origin);
          u3[0] && (this.origin = zt(u3[0]));
          const s3 = t4.getElementsByTagName(H3.Geometry);
          s3[0] && (this.geometry = mr(s3[0]));
          const c5 = t4.getElementsByTagName(H3.Material);
          c5[0] && (this.material = new Vt({
            xml: c5[0]
          }));
        }
      };
      Er = class {
        constructor({ xml: t4 }) {
          this.visuals = [], this.name = t4.getAttribute(H3.Name) ?? "unknown_name";
          const u3 = t4.getElementsByTagName(H3.Visuals);
          for (const s3 of u3)
            this.visuals.push(
              new dr({
                xml: s3
              })
            );
        }
      };
      G = {};
      ae2 = {};
      it = {};
      K2 = {};
      L4 = {};
      We = {};
      gt = {};
      ht = {};
      xt = vr();
      Tr = class {
        constructor({ xml: t4 }) {
          this.parent = null, this.child = null, this.minval = NaN, this.maxval = NaN, this.origin = new pt(), this.axis = new Oe({
            x: 1,
            y: 0,
            z: 0
          }), this.name = t4.getAttribute(H3.Name) ?? "unknown_name", this.type = t4.getAttribute(H3.Type);
          const u3 = t4.getElementsByTagName(H3.Parent);
          u3[0] && (this.parent = u3[0].getAttribute(H3.Link));
          const s3 = t4.getElementsByTagName(H3.Child);
          s3[0] && (this.child = s3[0].getAttribute(H3.Link));
          const c5 = t4.getElementsByTagName(H3.Limit);
          c5[0] && (this.minval = parseFloat(
            c5[0].getAttribute(H3.Lower) ?? "NaN"
          ), this.maxval = parseFloat(
            c5[0].getAttribute(H3.Upper) ?? "NaN"
          ));
          const o5 = t4.getElementsByTagName(H3.Origin);
          o5[0] && (this.origin = zt(o5[0]));
          const h4 = t4.getElementsByTagName(H3.Axis);
          if (h4[0]) {
            const C2 = h4[0].getAttribute(H3.Xyz)?.split(" ");
            if (C2?.length !== 3)
              throw new Error(
                "If specified, axis must have an xyz value composed of three numbers"
              );
            const [f7, g5, D] = C2.map(parseFloat);
            this.axis = new Oe({
              x: f7,
              y: g5,
              z: D
            });
          }
        }
      };
      qr = class {
        constructor({ xml: t4, string: u3 }) {
          this.materials = {}, this.links = {}, this.joints = {};
          let s3 = t4;
          if (u3 && (s3 = new xt.DOMParser().parseFromString(u3, xt.MIME_TYPE.XML_TEXT).documentElement ?? void 0), !s3)
            throw new Error("No URDF document parsed!");
          this.name = s3.getAttribute(H3.Name);
          const c5 = s3.childNodes;
          for (const o5 of c5)
            if (Ht(o5))
              switch (o5.tagName) {
                case "material": {
                  const h4 = new Vt({ xml: o5 });
                  if (!Object.hasOwn(this.materials, h4.name)) {
                    this.materials[h4.name] = h4;
                    break;
                  }
                  const C2 = this.materials[h4.name];
                  C2?.isLink() ? C2.assign(h4) : console.warn(`Material ${h4.name} is not unique.`);
                  break;
                }
                case "link": {
                  const h4 = new Er({ xml: o5 });
                  if (Object.hasOwn(this.links, h4.name)) {
                    console.warn(`Link ${h4.name} is not unique.`);
                    break;
                  }
                  for (const C2 of h4.visuals) {
                    const f7 = C2.material;
                    if (!f7?.name)
                      continue;
                    const g5 = this.materials[f7.name];
                    g5 ? C2.material = g5 : this.materials[f7.name] = f7;
                  }
                  this.links[h4.name] = h4;
                  break;
                }
                case "joint": {
                  const h4 = new Tr({ xml: o5 });
                  this.joints[h4.name] = h4;
                  break;
                }
              }
        }
      };
      Gr = "2.1.0";
    }
  });

  // entry.js
  init_RosLib();
  globalThis.ROSLIB = RosLib_exports;
})();
