// Foxglove CDR vendor bundle
// 内容: @foxglove/rosmsg + @foxglove/rosmsg2-serialization + @foxglove/cdr
// 自动生成 — 重新生成: node scripts/build_vendor.mjs

var __create = Object.create;
var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __getProtoOf = Object.getPrototypeOf;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __commonJS = (cb, mod) => function __require() {
  return mod || (0, cb[__getOwnPropNames(cb)[0]])((mod = { exports: {} }).exports, mod), mod.exports;
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

// node_modules/@foxglove/rosmsg/dist/index.js
var require_dist = __commonJS({
  "node_modules/@foxglove/rosmsg/dist/index.js"(exports, module) {
    (() => {
      var __webpack_modules__ = {
        /***/
        91: (
          /***/
          ((__unused_webpack_module, exports2) => {
            "use strict";
            Object.defineProperty(exports2, "__esModule", { value: true });
            exports2.buildRos2Type = buildRos2Type;
            const TYPE = String.raw`(?<type>[a-zA-Z0-9_/]+)`;
            const STRING_BOUND = String.raw`(?:<=(?<stringBound>\d+))`;
            const ARRAY_BOUND = String.raw`(?:(?<unboundedArray>\[\])|\[(?<arrayLength>\d+)\]|\[<=(?<arrayBound>\d+)\])`;
            const NAME = String.raw`(?<name>[a-zA-Z0-9_]+)`;
            const QUOTED_STRING = String.raw`'(?:\\.|[^'\\])*'|"(?:\\.|[^"\\])*"`;
            const COMMENT_TERMINATED_LITERAL = String.raw`(?:${QUOTED_STRING}|(?:\\.|[^\s'"#\\])(?:\\.|[^#\\])*)`;
            const ARRAY_TERMINATED_LITERAL = String.raw`(?:${QUOTED_STRING}|(?:\\.|[^\s'"\],#\\])(?:\\.|[^\],#\\])*)`;
            const CONSTANT_ASSIGNMENT = String.raw`\s*=\s*(?<constantValue>${COMMENT_TERMINATED_LITERAL}?)`;
            const DEFAULT_VALUE_ARRAY = String.raw`\[(?:${ARRAY_TERMINATED_LITERAL},)*${ARRAY_TERMINATED_LITERAL}?\]`;
            const DEFAULT_VALUE = String.raw`(?<defaultValue>${DEFAULT_VALUE_ARRAY}|${COMMENT_TERMINATED_LITERAL})`;
            const COMMENT = String.raw`(?:#.*)`;
            const DEFINITION_LINE_REGEX = new RegExp(String.raw`^${TYPE}${STRING_BOUND}?${ARRAY_BOUND}?\s+${NAME}(?:${CONSTANT_ASSIGNMENT}|\s+${DEFAULT_VALUE})?\s*${COMMENT}?$`);
            const STRING_ESCAPES = String.raw`\\(?<char>['"abfnrtv\\])|\\(?<oct>[0-7]{1,3})|\\x(?<hex2>[a-fA-F0-9]{2})|\\u(?<hex4>[a-fA-F0-9]{4})|\\U(?<hex8>[a-fA-F0-9]{8})`;
            const BUILTIN_TYPES = [
              "bool",
              "byte",
              "char",
              "float32",
              "float64",
              "int8",
              "uint8",
              "int16",
              "uint16",
              "int32",
              "uint32",
              "int64",
              "uint64",
              "string",
              "wstring",
              "time",
              "duration",
              "builtin_interfaces/Time",
              "builtin_interfaces/Duration",
              "builtin_interfaces/msg/Time",
              "builtin_interfaces/msg/Duration"
            ];
            function parseBigIntLiteral(str, min, max) {
              const value = BigInt(str);
              if (value < min || value > max) {
                throw new Error(`Number ${str} out of range [${min}, ${max}]`);
              }
              return value;
            }
            function parseNumberLiteral(str, min, max) {
              const value = parseInt(str);
              if (Number.isNaN(value)) {
                throw new Error(`Invalid numeric literal: ${str}`);
              }
              if (value < min || value > max) {
                throw new Error(`Number ${str} out of range [${min}, ${max}]`);
              }
              return value;
            }
            const LITERAL_REGEX = new RegExp(ARRAY_TERMINATED_LITERAL, "y");
            const COMMA_OR_END_REGEX = /\s*(,)\s*|\s*$/y;
            function parseArrayLiteral(type, rawStr) {
              if (!rawStr.startsWith("[") || !rawStr.endsWith("]")) {
                throw new Error("Array must start with [ and end with ]");
              }
              const str = rawStr.substring(1, rawStr.length - 1);
              if (type === "string" || type === "wstring") {
                const results = [];
                let offset = 0;
                while (offset < str.length) {
                  if (str[offset] === ",") {
                    throw new Error("Expected array element before comma");
                  }
                  LITERAL_REGEX.lastIndex = offset;
                  let match = LITERAL_REGEX.exec(str);
                  if (match) {
                    results.push(parseStringLiteral(match[0]));
                    offset = LITERAL_REGEX.lastIndex;
                  }
                  COMMA_OR_END_REGEX.lastIndex = offset;
                  match = COMMA_OR_END_REGEX.exec(str);
                  if (!match) {
                    throw new Error("Expected comma or end of array");
                  }
                  if (!match[1]) {
                    break;
                  }
                  offset = COMMA_OR_END_REGEX.lastIndex;
                }
                return results;
              }
              return str.split(",").map((part) => parsePrimitiveLiteral(type, part.trim()));
            }
            function parseStringLiteral(maybeQuotedStr) {
              let quoteThatMustBeEscaped = "";
              let str = maybeQuotedStr;
              for (const quote of ["'", '"']) {
                if (maybeQuotedStr.startsWith(quote)) {
                  if (!maybeQuotedStr.endsWith(quote)) {
                    throw new Error(`Expected terminating ${quote} in string literal: ${maybeQuotedStr}`);
                  }
                  quoteThatMustBeEscaped = quote;
                  str = maybeQuotedStr.substring(quote.length, maybeQuotedStr.length - quote.length);
                  break;
                }
              }
              if (
                // eslint-disable-next-line no-constant-binary-expression, @typescript-eslint/no-unnecessary-condition
                !new RegExp(String.raw`^(?:[^\\${quoteThatMustBeEscaped}]|${STRING_ESCAPES})*$`).test(str) == void 0
              ) {
                throw new Error(`Invalid string literal: ${str}`);
              }
              return str.replace(new RegExp(STRING_ESCAPES, "g"), (...args) => {
                const { char, oct, hex2, hex4, hex8 } = args[args.length - 1];
                const hex = hex2 ?? hex4 ?? hex8;
                if (char != void 0) {
                  return {
                    "'": "'",
                    '"': '"',
                    a: "\x07",
                    b: "\b",
                    f: "\f",
                    n: "\n",
                    r: "\r",
                    t: "	",
                    v: "\v",
                    "\\": "\\"
                  }[char];
                } else if (oct != void 0) {
                  return String.fromCodePoint(parseInt(oct, 8));
                } else if (hex != void 0) {
                  return String.fromCodePoint(parseInt(hex, 16));
                } else {
                  throw new Error("Expected exactly one matched group");
                }
              });
            }
            function parsePrimitiveLiteral(type, str) {
              switch (type) {
                case "bool":
                  if (["true", "True", "1"].includes(str)) {
                    return true;
                  } else if (["false", "False", "0"].includes(str)) {
                    return false;
                  }
                  break;
                case "float32":
                case "float64": {
                  const value = parseFloat(str);
                  if (!Number.isNaN(value)) {
                    return value;
                  }
                  break;
                }
                case "int8":
                  return parseNumberLiteral(str, ~127, 127);
                case "uint8":
                  return parseNumberLiteral(str, 0, 255);
                case "int16":
                  return parseNumberLiteral(str, ~32767, 32767);
                case "uint16":
                  return parseNumberLiteral(str, 0, 65535);
                case "int32":
                  return parseNumberLiteral(str, ~2147483647, 2147483647);
                case "uint32":
                  return parseNumberLiteral(str, 0, 4294967295);
                case "int64":
                  return parseBigIntLiteral(str, ~0x7fffffffffffffffn, 0x7fffffffffffffffn);
                case "uint64":
                  return parseBigIntLiteral(str, 0n, 0xffffffffffffffffn);
                case "string":
                case "wstring":
                  return parseStringLiteral(str);
              }
              throw new Error(`Invalid literal of type ${type}: ${str}`);
            }
            function normalizeType(type) {
              switch (type) {
                case "char":
                  return "uint8";
                case "byte":
                  return "uint8";
                case "builtin_interfaces/Time":
                case "builtin_interfaces/msg/Time":
                  return "time";
                case "builtin_interfaces/Duration":
                case "builtin_interfaces/msg/Duration":
                  return "duration";
              }
              return type;
            }
            function buildRos2Type(lines) {
              const definitions = [];
              let complexTypeName;
              for (const { line } of lines) {
                let match;
                if (line.startsWith("#")) {
                  continue;
                } else if (match = /^MSG: ([^ ]+)\s*(?:#.+)?$/.exec(line)) {
                  complexTypeName = match[1];
                  continue;
                } else if (match = DEFINITION_LINE_REGEX.exec(line)) {
                  const { type: rawType, stringBound, unboundedArray, arrayLength, arrayBound, name, constantValue, defaultValue } = match.groups;
                  const type = normalizeType(rawType);
                  if (stringBound != void 0 && type !== "string" && type !== "wstring") {
                    throw new Error(`Invalid string bound for type ${type}`);
                  }
                  if (constantValue != void 0) {
                    if (!/^[A-Z](?:_?[A-Z0-9]+)*$/.test(name)) {
                      throw new Error(`Invalid constant name: ${name}`);
                    }
                  } else {
                    if (!/^[a-z](?:_?[a-z0-9]+)*$/.test(name)) {
                      throw new Error(`Invalid field name: ${name}`);
                    }
                  }
                  const isComplex = !BUILTIN_TYPES.includes(type);
                  const isArray = unboundedArray != void 0 || arrayLength != void 0 || arrayBound != void 0;
                  definitions.push({
                    name,
                    type,
                    isComplex: constantValue != void 0 ? isComplex || void 0 : isComplex,
                    isConstant: constantValue != void 0 || void 0,
                    isArray: constantValue != void 0 ? isArray || void 0 : isArray,
                    arrayLength: arrayLength != void 0 ? parseInt(arrayLength) : void 0,
                    arrayUpperBound: arrayBound != void 0 ? parseInt(arrayBound) : void 0,
                    upperBound: stringBound != void 0 ? parseInt(stringBound) : void 0,
                    defaultValue: defaultValue != void 0 ? isArray ? parseArrayLiteral(type, defaultValue.trim()) : parsePrimitiveLiteral(type, defaultValue.trim()) : void 0,
                    value: constantValue != void 0 ? parsePrimitiveLiteral(type, constantValue.trim()) : void 0,
                    valueText: constantValue?.trim()
                  });
                } else {
                  throw new Error(`Could not parse line: '${line}'`);
                }
              }
              return { name: complexTypeName, definitions };
            }
          })
        ),
        /***/
        155: (
          /***/
          (function(__unused_webpack_module, exports2, __webpack_require__2) {
            "use strict";
            var __createBinding = this && this.__createBinding || (Object.create ? (function(o, m, k, k2) {
              if (k2 === void 0) k2 = k;
              var desc = Object.getOwnPropertyDescriptor(m, k);
              if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
                desc = { enumerable: true, get: function() {
                  return m[k];
                } };
              }
              Object.defineProperty(o, k2, desc);
            }) : (function(o, m, k, k2) {
              if (k2 === void 0) k2 = k;
              o[k2] = m[k];
            }));
            var __exportStar = this && this.__exportStar || function(m, exports3) {
              for (var p in m) if (p !== "default" && !Object.prototype.hasOwnProperty.call(exports3, p)) __createBinding(exports3, m, p);
            };
            Object.defineProperty(exports2, "__esModule", { value: true });
            __exportStar(__webpack_require__2(498), exports2);
            __exportStar(__webpack_require__2(900), exports2);
          })
        ),
        /***/
        425: (
          /***/
          ((__unused_webpack_module, exports2, __webpack_require__2) => {
            "use strict";
            Object.defineProperty(exports2, "__esModule", { value: true });
            exports2.md5 = md5;
            const md5_typescript_1 = __webpack_require__2(479);
            const BUILTIN_TYPES = /* @__PURE__ */ new Set([
              "int8",
              "uint8",
              "int16",
              "uint16",
              "int32",
              "uint32",
              "int64",
              "uint64",
              "float32",
              "float64",
              "string",
              "bool",
              "char",
              "byte",
              "time",
              "duration"
            ]);
            function md5(msgDefs) {
              if (msgDefs.length === 0) {
                throw new Error(`Cannot produce md5sum for empty msgDefs`);
              }
              const subMsgDefs = /* @__PURE__ */ new Map();
              for (const msgDef of msgDefs) {
                if (msgDef.name != void 0) {
                  subMsgDefs.set(msgDef.name, msgDef);
                }
              }
              const first = msgDefs[0];
              return computeMessageMd5(first, subMsgDefs);
            }
            function computeMessageMd5(msgDef, subMsgDefs) {
              let output = "";
              const constants = msgDef.definitions.filter(({ isConstant }) => isConstant);
              const variables = msgDef.definitions.filter(({ isConstant }) => isConstant == void 0 || !isConstant);
              for (const def of constants) {
                output += `${def.type} ${def.name}=${def.valueText ?? String(def.value)}
`;
              }
              for (const def of variables) {
                if (isBuiltin(def.type)) {
                  const arrayLength = def.arrayLength != void 0 ? String(def.arrayLength) : "";
                  const array = def.isArray === true ? `[${arrayLength}]` : "";
                  output += `${def.type}${array} ${def.name}
`;
                } else {
                  const subMsgDef = subMsgDefs.get(def.type);
                  if (subMsgDef == void 0) {
                    throw new Error(`Missing definition for submessage type "${def.type}"`);
                  }
                  const subMd5 = computeMessageMd5(subMsgDef, subMsgDefs);
                  output += `${subMd5} ${def.name}
`;
                }
              }
              output = output.trimEnd();
              return md5_typescript_1.Md5.init(output);
            }
            function isBuiltin(typeName) {
              return BUILTIN_TYPES.has(typeName);
            }
          })
        ),
        /***/
        479: (
          /***/
          ((__unused_webpack_module, __webpack_exports__2, __webpack_require__2) => {
            "use strict";
            __webpack_require__2.r(__webpack_exports__2);
            __webpack_require__2.d(__webpack_exports__2, {
              /* harmony export */
              Md5: () => (
                /* binding */
                Md5
              )
              /* harmony export */
            });
            var Md5 = (
              /** @class */
              (function() {
                function Md52() {
                }
                Md52.AddUnsigned = function(lX, lY) {
                  var lX4, lY4, lX8, lY8, lResult;
                  lX8 = lX & 2147483648;
                  lY8 = lY & 2147483648;
                  lX4 = lX & 1073741824;
                  lY4 = lY & 1073741824;
                  lResult = (lX & 1073741823) + (lY & 1073741823);
                  if (!!(lX4 & lY4)) {
                    return lResult ^ 2147483648 ^ lX8 ^ lY8;
                  }
                  if (!!(lX4 | lY4)) {
                    if (!!(lResult & 1073741824)) {
                      return lResult ^ 3221225472 ^ lX8 ^ lY8;
                    } else {
                      return lResult ^ 1073741824 ^ lX8 ^ lY8;
                    }
                  } else {
                    return lResult ^ lX8 ^ lY8;
                  }
                };
                Md52.FF = function(a, b, c, d, x, s, ac) {
                  a = this.AddUnsigned(a, this.AddUnsigned(this.AddUnsigned(this.F(b, c, d), x), ac));
                  return this.AddUnsigned(this.RotateLeft(a, s), b);
                };
                Md52.GG = function(a, b, c, d, x, s, ac) {
                  a = this.AddUnsigned(a, this.AddUnsigned(this.AddUnsigned(this.G(b, c, d), x), ac));
                  return this.AddUnsigned(this.RotateLeft(a, s), b);
                };
                Md52.HH = function(a, b, c, d, x, s, ac) {
                  a = this.AddUnsigned(a, this.AddUnsigned(this.AddUnsigned(this.H(b, c, d), x), ac));
                  return this.AddUnsigned(this.RotateLeft(a, s), b);
                };
                Md52.II = function(a, b, c, d, x, s, ac) {
                  a = this.AddUnsigned(a, this.AddUnsigned(this.AddUnsigned(this.I(b, c, d), x), ac));
                  return this.AddUnsigned(this.RotateLeft(a, s), b);
                };
                Md52.ConvertToWordArray = function(string) {
                  var lWordCount, lMessageLength = string.length, lNumberOfWords_temp1 = lMessageLength + 8, lNumberOfWords_temp2 = (lNumberOfWords_temp1 - lNumberOfWords_temp1 % 64) / 64, lNumberOfWords = (lNumberOfWords_temp2 + 1) * 16, lWordArray = Array(lNumberOfWords - 1), lBytePosition = 0, lByteCount = 0;
                  while (lByteCount < lMessageLength) {
                    lWordCount = (lByteCount - lByteCount % 4) / 4;
                    lBytePosition = lByteCount % 4 * 8;
                    lWordArray[lWordCount] = lWordArray[lWordCount] | string.charCodeAt(lByteCount) << lBytePosition;
                    lByteCount++;
                  }
                  lWordCount = (lByteCount - lByteCount % 4) / 4;
                  lBytePosition = lByteCount % 4 * 8;
                  lWordArray[lWordCount] = lWordArray[lWordCount] | 128 << lBytePosition;
                  lWordArray[lNumberOfWords - 2] = lMessageLength << 3;
                  lWordArray[lNumberOfWords - 1] = lMessageLength >>> 29;
                  return lWordArray;
                };
                Md52.WordToHex = function(lValue) {
                  var WordToHexValue = "", WordToHexValue_temp = "", lByte, lCount;
                  for (lCount = 0; lCount <= 3; lCount++) {
                    lByte = lValue >>> lCount * 8 & 255;
                    WordToHexValue_temp = "0" + lByte.toString(16);
                    WordToHexValue = WordToHexValue + WordToHexValue_temp.substr(WordToHexValue_temp.length - 2, 2);
                  }
                  return WordToHexValue;
                };
                Md52.Utf8Encode = function(string) {
                  var utftext = "", c;
                  string = string.replace(/\r\n/g, "\n");
                  for (var n = 0; n < string.length; n++) {
                    c = string.charCodeAt(n);
                    if (c < 128) {
                      utftext += String.fromCharCode(c);
                    } else if (c > 127 && c < 2048) {
                      utftext += String.fromCharCode(c >> 6 | 192);
                      utftext += String.fromCharCode(c & 63 | 128);
                    } else {
                      utftext += String.fromCharCode(c >> 12 | 224);
                      utftext += String.fromCharCode(c >> 6 & 63 | 128);
                      utftext += String.fromCharCode(c & 63 | 128);
                    }
                  }
                  return utftext;
                };
                Md52.init = function(string) {
                  var temp;
                  if (typeof string !== "string")
                    string = JSON.stringify(string);
                  this._string = this.Utf8Encode(string);
                  this.x = this.ConvertToWordArray(this._string);
                  this.a = 1732584193;
                  this.b = 4023233417;
                  this.c = 2562383102;
                  this.d = 271733878;
                  for (this.k = 0; this.k < this.x.length; this.k += 16) {
                    this.AA = this.a;
                    this.BB = this.b;
                    this.CC = this.c;
                    this.DD = this.d;
                    this.a = this.FF(this.a, this.b, this.c, this.d, this.x[this.k], this.S11, 3614090360);
                    this.d = this.FF(this.d, this.a, this.b, this.c, this.x[this.k + 1], this.S12, 3905402710);
                    this.c = this.FF(this.c, this.d, this.a, this.b, this.x[this.k + 2], this.S13, 606105819);
                    this.b = this.FF(this.b, this.c, this.d, this.a, this.x[this.k + 3], this.S14, 3250441966);
                    this.a = this.FF(this.a, this.b, this.c, this.d, this.x[this.k + 4], this.S11, 4118548399);
                    this.d = this.FF(this.d, this.a, this.b, this.c, this.x[this.k + 5], this.S12, 1200080426);
                    this.c = this.FF(this.c, this.d, this.a, this.b, this.x[this.k + 6], this.S13, 2821735955);
                    this.b = this.FF(this.b, this.c, this.d, this.a, this.x[this.k + 7], this.S14, 4249261313);
                    this.a = this.FF(this.a, this.b, this.c, this.d, this.x[this.k + 8], this.S11, 1770035416);
                    this.d = this.FF(this.d, this.a, this.b, this.c, this.x[this.k + 9], this.S12, 2336552879);
                    this.c = this.FF(this.c, this.d, this.a, this.b, this.x[this.k + 10], this.S13, 4294925233);
                    this.b = this.FF(this.b, this.c, this.d, this.a, this.x[this.k + 11], this.S14, 2304563134);
                    this.a = this.FF(this.a, this.b, this.c, this.d, this.x[this.k + 12], this.S11, 1804603682);
                    this.d = this.FF(this.d, this.a, this.b, this.c, this.x[this.k + 13], this.S12, 4254626195);
                    this.c = this.FF(this.c, this.d, this.a, this.b, this.x[this.k + 14], this.S13, 2792965006);
                    this.b = this.FF(this.b, this.c, this.d, this.a, this.x[this.k + 15], this.S14, 1236535329);
                    this.a = this.GG(this.a, this.b, this.c, this.d, this.x[this.k + 1], this.S21, 4129170786);
                    this.d = this.GG(this.d, this.a, this.b, this.c, this.x[this.k + 6], this.S22, 3225465664);
                    this.c = this.GG(this.c, this.d, this.a, this.b, this.x[this.k + 11], this.S23, 643717713);
                    this.b = this.GG(this.b, this.c, this.d, this.a, this.x[this.k], this.S24, 3921069994);
                    this.a = this.GG(this.a, this.b, this.c, this.d, this.x[this.k + 5], this.S21, 3593408605);
                    this.d = this.GG(this.d, this.a, this.b, this.c, this.x[this.k + 10], this.S22, 38016083);
                    this.c = this.GG(this.c, this.d, this.a, this.b, this.x[this.k + 15], this.S23, 3634488961);
                    this.b = this.GG(this.b, this.c, this.d, this.a, this.x[this.k + 4], this.S24, 3889429448);
                    this.a = this.GG(this.a, this.b, this.c, this.d, this.x[this.k + 9], this.S21, 568446438);
                    this.d = this.GG(this.d, this.a, this.b, this.c, this.x[this.k + 14], this.S22, 3275163606);
                    this.c = this.GG(this.c, this.d, this.a, this.b, this.x[this.k + 3], this.S23, 4107603335);
                    this.b = this.GG(this.b, this.c, this.d, this.a, this.x[this.k + 8], this.S24, 1163531501);
                    this.a = this.GG(this.a, this.b, this.c, this.d, this.x[this.k + 13], this.S21, 2850285829);
                    this.d = this.GG(this.d, this.a, this.b, this.c, this.x[this.k + 2], this.S22, 4243563512);
                    this.c = this.GG(this.c, this.d, this.a, this.b, this.x[this.k + 7], this.S23, 1735328473);
                    this.b = this.GG(this.b, this.c, this.d, this.a, this.x[this.k + 12], this.S24, 2368359562);
                    this.a = this.HH(this.a, this.b, this.c, this.d, this.x[this.k + 5], this.S31, 4294588738);
                    this.d = this.HH(this.d, this.a, this.b, this.c, this.x[this.k + 8], this.S32, 2272392833);
                    this.c = this.HH(this.c, this.d, this.a, this.b, this.x[this.k + 11], this.S33, 1839030562);
                    this.b = this.HH(this.b, this.c, this.d, this.a, this.x[this.k + 14], this.S34, 4259657740);
                    this.a = this.HH(this.a, this.b, this.c, this.d, this.x[this.k + 1], this.S31, 2763975236);
                    this.d = this.HH(this.d, this.a, this.b, this.c, this.x[this.k + 4], this.S32, 1272893353);
                    this.c = this.HH(this.c, this.d, this.a, this.b, this.x[this.k + 7], this.S33, 4139469664);
                    this.b = this.HH(this.b, this.c, this.d, this.a, this.x[this.k + 10], this.S34, 3200236656);
                    this.a = this.HH(this.a, this.b, this.c, this.d, this.x[this.k + 13], this.S31, 681279174);
                    this.d = this.HH(this.d, this.a, this.b, this.c, this.x[this.k], this.S32, 3936430074);
                    this.c = this.HH(this.c, this.d, this.a, this.b, this.x[this.k + 3], this.S33, 3572445317);
                    this.b = this.HH(this.b, this.c, this.d, this.a, this.x[this.k + 6], this.S34, 76029189);
                    this.a = this.HH(this.a, this.b, this.c, this.d, this.x[this.k + 9], this.S31, 3654602809);
                    this.d = this.HH(this.d, this.a, this.b, this.c, this.x[this.k + 12], this.S32, 3873151461);
                    this.c = this.HH(this.c, this.d, this.a, this.b, this.x[this.k + 15], this.S33, 530742520);
                    this.b = this.HH(this.b, this.c, this.d, this.a, this.x[this.k + 2], this.S34, 3299628645);
                    this.a = this.II(this.a, this.b, this.c, this.d, this.x[this.k], this.S41, 4096336452);
                    this.d = this.II(this.d, this.a, this.b, this.c, this.x[this.k + 7], this.S42, 1126891415);
                    this.c = this.II(this.c, this.d, this.a, this.b, this.x[this.k + 14], this.S43, 2878612391);
                    this.b = this.II(this.b, this.c, this.d, this.a, this.x[this.k + 5], this.S44, 4237533241);
                    this.a = this.II(this.a, this.b, this.c, this.d, this.x[this.k + 12], this.S41, 1700485571);
                    this.d = this.II(this.d, this.a, this.b, this.c, this.x[this.k + 3], this.S42, 2399980690);
                    this.c = this.II(this.c, this.d, this.a, this.b, this.x[this.k + 10], this.S43, 4293915773);
                    this.b = this.II(this.b, this.c, this.d, this.a, this.x[this.k + 1], this.S44, 2240044497);
                    this.a = this.II(this.a, this.b, this.c, this.d, this.x[this.k + 8], this.S41, 1873313359);
                    this.d = this.II(this.d, this.a, this.b, this.c, this.x[this.k + 15], this.S42, 4264355552);
                    this.c = this.II(this.c, this.d, this.a, this.b, this.x[this.k + 6], this.S43, 2734768916);
                    this.b = this.II(this.b, this.c, this.d, this.a, this.x[this.k + 13], this.S44, 1309151649);
                    this.a = this.II(this.a, this.b, this.c, this.d, this.x[this.k + 4], this.S41, 4149444226);
                    this.d = this.II(this.d, this.a, this.b, this.c, this.x[this.k + 11], this.S42, 3174756917);
                    this.c = this.II(this.c, this.d, this.a, this.b, this.x[this.k + 2], this.S43, 718787259);
                    this.b = this.II(this.b, this.c, this.d, this.a, this.x[this.k + 9], this.S44, 3951481745);
                    this.a = this.AddUnsigned(this.a, this.AA);
                    this.b = this.AddUnsigned(this.b, this.BB);
                    this.c = this.AddUnsigned(this.c, this.CC);
                    this.d = this.AddUnsigned(this.d, this.DD);
                  }
                  temp = this.WordToHex(this.a) + this.WordToHex(this.b) + this.WordToHex(this.c) + this.WordToHex(this.d);
                  return temp.toLowerCase();
                };
                Md52.x = Array();
                Md52.S11 = 7;
                Md52.S12 = 12;
                Md52.S13 = 17;
                Md52.S14 = 22;
                Md52.S21 = 5;
                Md52.S22 = 9;
                Md52.S23 = 14;
                Md52.S24 = 20;
                Md52.S31 = 4;
                Md52.S32 = 11;
                Md52.S33 = 16;
                Md52.S34 = 23;
                Md52.S41 = 6;
                Md52.S42 = 10;
                Md52.S43 = 15;
                Md52.S44 = 21;
                Md52.RotateLeft = function(lValue, iShiftBits) {
                  return lValue << iShiftBits | lValue >>> 32 - iShiftBits;
                };
                Md52.F = function(x, y, z) {
                  return x & y | ~x & z;
                };
                Md52.G = function(x, y, z) {
                  return x & z | y & ~z;
                };
                Md52.H = function(x, y, z) {
                  return x ^ y ^ z;
                };
                Md52.I = function(x, y, z) {
                  return y ^ (x | ~z);
                };
                return Md52;
              })()
            );
          })
        ),
        /***/
        498: (
          /***/
          ((__unused_webpack_module, exports2) => {
            "use strict";
            Object.defineProperty(exports2, "__esModule", { value: true });
          })
        ),
        /***/
        662: (
          /***/
          (function(module2) {
            (function(root, factory) {
              if (module2.exports) {
                module2.exports = factory();
              } else {
                root.nearley = factory();
              }
            })(this, function() {
              function Rule(name, symbols, postprocess) {
                this.id = ++Rule.highestId;
                this.name = name;
                this.symbols = symbols;
                this.postprocess = postprocess;
                return this;
              }
              Rule.highestId = 0;
              Rule.prototype.toString = function(withCursorAt) {
                var symbolSequence = typeof withCursorAt === "undefined" ? this.symbols.map(getSymbolShortDisplay).join(" ") : this.symbols.slice(0, withCursorAt).map(getSymbolShortDisplay).join(" ") + " \u25CF " + this.symbols.slice(withCursorAt).map(getSymbolShortDisplay).join(" ");
                return this.name + " \u2192 " + symbolSequence;
              };
              function State(rule, dot, reference, wantedBy) {
                this.rule = rule;
                this.dot = dot;
                this.reference = reference;
                this.data = [];
                this.wantedBy = wantedBy;
                this.isComplete = this.dot === rule.symbols.length;
              }
              State.prototype.toString = function() {
                return "{" + this.rule.toString(this.dot) + "}, from: " + (this.reference || 0);
              };
              State.prototype.nextState = function(child) {
                var state = new State(this.rule, this.dot + 1, this.reference, this.wantedBy);
                state.left = this;
                state.right = child;
                if (state.isComplete) {
                  state.data = state.build();
                  state.right = void 0;
                }
                return state;
              };
              State.prototype.build = function() {
                var children = [];
                var node = this;
                do {
                  children.push(node.right.data);
                  node = node.left;
                } while (node.left);
                children.reverse();
                return children;
              };
              State.prototype.finish = function() {
                if (this.rule.postprocess) {
                  this.data = this.rule.postprocess(this.data, this.reference, Parser.fail);
                }
              };
              function Column(grammar, index) {
                this.grammar = grammar;
                this.index = index;
                this.states = [];
                this.wants = {};
                this.scannable = [];
                this.completed = {};
              }
              Column.prototype.process = function(nextColumn) {
                var states = this.states;
                var wants = this.wants;
                var completed = this.completed;
                for (var w = 0; w < states.length; w++) {
                  var state = states[w];
                  if (state.isComplete) {
                    state.finish();
                    if (state.data !== Parser.fail) {
                      var wantedBy = state.wantedBy;
                      for (var i = wantedBy.length; i--; ) {
                        var left = wantedBy[i];
                        this.complete(left, state);
                      }
                      if (state.reference === this.index) {
                        var exp = state.rule.name;
                        (this.completed[exp] = this.completed[exp] || []).push(state);
                      }
                    }
                  } else {
                    var exp = state.rule.symbols[state.dot];
                    if (typeof exp !== "string") {
                      this.scannable.push(state);
                      continue;
                    }
                    if (wants[exp]) {
                      wants[exp].push(state);
                      if (completed.hasOwnProperty(exp)) {
                        var nulls = completed[exp];
                        for (var i = 0; i < nulls.length; i++) {
                          var right = nulls[i];
                          this.complete(state, right);
                        }
                      }
                    } else {
                      wants[exp] = [state];
                      this.predict(exp);
                    }
                  }
                }
              };
              Column.prototype.predict = function(exp) {
                var rules = this.grammar.byName[exp] || [];
                for (var i = 0; i < rules.length; i++) {
                  var r = rules[i];
                  var wantedBy = this.wants[exp];
                  var s = new State(r, 0, this.index, wantedBy);
                  this.states.push(s);
                }
              };
              Column.prototype.complete = function(left, right) {
                var copy = left.nextState(right);
                this.states.push(copy);
              };
              function Grammar(rules, start) {
                this.rules = rules;
                this.start = start || this.rules[0].name;
                var byName = this.byName = {};
                this.rules.forEach(function(rule) {
                  if (!byName.hasOwnProperty(rule.name)) {
                    byName[rule.name] = [];
                  }
                  byName[rule.name].push(rule);
                });
              }
              Grammar.fromCompiled = function(rules, start) {
                var lexer = rules.Lexer;
                if (rules.ParserStart) {
                  start = rules.ParserStart;
                  rules = rules.ParserRules;
                }
                var rules = rules.map(function(r) {
                  return new Rule(r.name, r.symbols, r.postprocess);
                });
                var g = new Grammar(rules, start);
                g.lexer = lexer;
                return g;
              };
              function StreamLexer() {
                this.reset("");
              }
              StreamLexer.prototype.reset = function(data, state) {
                this.buffer = data;
                this.index = 0;
                this.line = state ? state.line : 1;
                this.lastLineBreak = state ? -state.col : 0;
              };
              StreamLexer.prototype.next = function() {
                if (this.index < this.buffer.length) {
                  var ch = this.buffer[this.index++];
                  if (ch === "\n") {
                    this.line += 1;
                    this.lastLineBreak = this.index;
                  }
                  return { value: ch };
                }
              };
              StreamLexer.prototype.save = function() {
                return {
                  line: this.line,
                  col: this.index - this.lastLineBreak
                };
              };
              StreamLexer.prototype.formatError = function(token, message) {
                var buffer = this.buffer;
                if (typeof buffer === "string") {
                  var lines = buffer.split("\n").slice(
                    Math.max(0, this.line - 5),
                    this.line
                  );
                  var nextLineBreak = buffer.indexOf("\n", this.index);
                  if (nextLineBreak === -1) nextLineBreak = buffer.length;
                  var col = this.index - this.lastLineBreak;
                  var lastLineDigits = String(this.line).length;
                  message += " at line " + this.line + " col " + col + ":\n\n";
                  message += lines.map(function(line, i) {
                    return pad(this.line - lines.length + i + 1, lastLineDigits) + " " + line;
                  }, this).join("\n");
                  message += "\n" + pad("", lastLineDigits + col) + "^\n";
                  return message;
                } else {
                  return message + " at index " + (this.index - 1);
                }
                function pad(n, length) {
                  var s = String(n);
                  return Array(length - s.length + 1).join(" ") + s;
                }
              };
              function Parser(rules, start, options) {
                if (rules instanceof Grammar) {
                  var grammar = rules;
                  var options = start;
                } else {
                  var grammar = Grammar.fromCompiled(rules, start);
                }
                this.grammar = grammar;
                this.options = {
                  keepHistory: false,
                  lexer: grammar.lexer || new StreamLexer()
                };
                for (var key in options || {}) {
                  this.options[key] = options[key];
                }
                this.lexer = this.options.lexer;
                this.lexerState = void 0;
                var column = new Column(grammar, 0);
                var table = this.table = [column];
                column.wants[grammar.start] = [];
                column.predict(grammar.start);
                column.process();
                this.current = 0;
              }
              Parser.fail = {};
              Parser.prototype.feed = function(chunk) {
                var lexer = this.lexer;
                lexer.reset(chunk, this.lexerState);
                var token;
                while (true) {
                  try {
                    token = lexer.next();
                    if (!token) {
                      break;
                    }
                  } catch (e) {
                    var nextColumn = new Column(this.grammar, this.current + 1);
                    this.table.push(nextColumn);
                    var err = new Error(this.reportLexerError(e));
                    err.offset = this.current;
                    err.token = e.token;
                    throw err;
                  }
                  var column = this.table[this.current];
                  if (!this.options.keepHistory) {
                    delete this.table[this.current - 1];
                  }
                  var n = this.current + 1;
                  var nextColumn = new Column(this.grammar, n);
                  this.table.push(nextColumn);
                  var literal = token.text !== void 0 ? token.text : token.value;
                  var value = lexer.constructor === StreamLexer ? token.value : token;
                  var scannable = column.scannable;
                  for (var w = scannable.length; w--; ) {
                    var state = scannable[w];
                    var expect = state.rule.symbols[state.dot];
                    if (expect.test ? expect.test(value) : expect.type ? expect.type === token.type : expect.literal === literal) {
                      var next = state.nextState({ data: value, token, isToken: true, reference: n - 1 });
                      nextColumn.states.push(next);
                    }
                  }
                  nextColumn.process();
                  if (nextColumn.states.length === 0) {
                    var err = new Error(this.reportError(token));
                    err.offset = this.current;
                    err.token = token;
                    throw err;
                  }
                  if (this.options.keepHistory) {
                    column.lexerState = lexer.save();
                  }
                  this.current++;
                }
                if (column) {
                  this.lexerState = lexer.save();
                }
                this.results = this.finish();
                return this;
              };
              Parser.prototype.reportLexerError = function(lexerError) {
                var tokenDisplay, lexerMessage;
                var token = lexerError.token;
                if (token) {
                  tokenDisplay = "input " + JSON.stringify(token.text[0]) + " (lexer error)";
                  lexerMessage = this.lexer.formatError(token, "Syntax error");
                } else {
                  tokenDisplay = "input (lexer error)";
                  lexerMessage = lexerError.message;
                }
                return this.reportErrorCommon(lexerMessage, tokenDisplay);
              };
              Parser.prototype.reportError = function(token) {
                var tokenDisplay = (token.type ? token.type + " token: " : "") + JSON.stringify(token.value !== void 0 ? token.value : token);
                var lexerMessage = this.lexer.formatError(token, "Syntax error");
                return this.reportErrorCommon(lexerMessage, tokenDisplay);
              };
              Parser.prototype.reportErrorCommon = function(lexerMessage, tokenDisplay) {
                var lines = [];
                lines.push(lexerMessage);
                var lastColumnIndex = this.table.length - 2;
                var lastColumn = this.table[lastColumnIndex];
                var expectantStates = lastColumn.states.filter(function(state) {
                  var nextSymbol = state.rule.symbols[state.dot];
                  return nextSymbol && typeof nextSymbol !== "string";
                });
                if (expectantStates.length === 0) {
                  lines.push("Unexpected " + tokenDisplay + ". I did not expect any more input. Here is the state of my parse table:\n");
                  this.displayStateStack(lastColumn.states, lines);
                } else {
                  lines.push("Unexpected " + tokenDisplay + ". Instead, I was expecting to see one of the following:\n");
                  var stateStacks = expectantStates.map(function(state) {
                    return this.buildFirstStateStack(state, []) || [state];
                  }, this);
                  stateStacks.forEach(function(stateStack) {
                    var state = stateStack[0];
                    var nextSymbol = state.rule.symbols[state.dot];
                    var symbolDisplay = this.getSymbolDisplay(nextSymbol);
                    lines.push("A " + symbolDisplay + " based on:");
                    this.displayStateStack(stateStack, lines);
                  }, this);
                }
                lines.push("");
                return lines.join("\n");
              };
              Parser.prototype.displayStateStack = function(stateStack, lines) {
                var lastDisplay;
                var sameDisplayCount = 0;
                for (var j = 0; j < stateStack.length; j++) {
                  var state = stateStack[j];
                  var display = state.rule.toString(state.dot);
                  if (display === lastDisplay) {
                    sameDisplayCount++;
                  } else {
                    if (sameDisplayCount > 0) {
                      lines.push("    ^ " + sameDisplayCount + " more lines identical to this");
                    }
                    sameDisplayCount = 0;
                    lines.push("    " + display);
                  }
                  lastDisplay = display;
                }
              };
              Parser.prototype.getSymbolDisplay = function(symbol) {
                return getSymbolLongDisplay(symbol);
              };
              Parser.prototype.buildFirstStateStack = function(state, visited) {
                if (visited.indexOf(state) !== -1) {
                  return null;
                }
                if (state.wantedBy.length === 0) {
                  return [state];
                }
                var prevState = state.wantedBy[0];
                var childVisited = [state].concat(visited);
                var childResult = this.buildFirstStateStack(prevState, childVisited);
                if (childResult === null) {
                  return null;
                }
                return [state].concat(childResult);
              };
              Parser.prototype.save = function() {
                var column = this.table[this.current];
                column.lexerState = this.lexerState;
                return column;
              };
              Parser.prototype.restore = function(column) {
                var index = column.index;
                this.current = index;
                this.table[index] = column;
                this.table.splice(index + 1);
                this.lexerState = column.lexerState;
                this.results = this.finish();
              };
              Parser.prototype.rewind = function(index) {
                if (!this.options.keepHistory) {
                  throw new Error("set option `keepHistory` to enable rewinding");
                }
                this.restore(this.table[index]);
              };
              Parser.prototype.finish = function() {
                var considerations = [];
                var start = this.grammar.start;
                var column = this.table[this.table.length - 1];
                column.states.forEach(function(t) {
                  if (t.rule.name === start && t.dot === t.rule.symbols.length && t.reference === 0 && t.data !== Parser.fail) {
                    considerations.push(t);
                  }
                });
                return considerations.map(function(c) {
                  return c.data;
                });
              };
              function getSymbolLongDisplay(symbol) {
                var type = typeof symbol;
                if (type === "string") {
                  return symbol;
                } else if (type === "object") {
                  if (symbol.literal) {
                    return JSON.stringify(symbol.literal);
                  } else if (symbol instanceof RegExp) {
                    return "character matching " + symbol;
                  } else if (symbol.type) {
                    return symbol.type + " token";
                  } else if (symbol.test) {
                    return "token matching " + String(symbol.test);
                  } else {
                    throw new Error("Unknown symbol type: " + symbol);
                  }
                }
              }
              function getSymbolShortDisplay(symbol) {
                var type = typeof symbol;
                if (type === "string") {
                  return symbol;
                } else if (type === "object") {
                  if (symbol.literal) {
                    return JSON.stringify(symbol.literal);
                  } else if (symbol instanceof RegExp) {
                    return symbol.toString();
                  } else if (symbol.type) {
                    return "%" + symbol.type;
                  } else if (symbol.test) {
                    return "<" + String(symbol.test) + ">";
                  } else {
                    throw new Error("Unknown symbol type: " + symbol);
                  }
                }
              }
              return {
                Parser,
                Grammar,
                Rule
              };
            });
          })
        ),
        /***/
        666: (
          /***/
          ((module2, __unused_webpack_exports, __webpack_require__2) => {
            (function() {
              function id(x) {
                return x[0];
              }
              const moo = __webpack_require__2(694);
              const lexer = moo.compile({
                space: { match: /\s+/, lineBreaks: true },
                number: /-?(?:[0-9]|[1-9][0-9]+)(?:\.[0-9]+)?(?:[eE][-+]?[0-9]+)?\b/,
                comment: /#[^\n]*/,
                "[": "[",
                "]": "]",
                assignment: /=[^\n]*/,
                // Leading underscores are disallowed in field names, while constant names have no explicit restrictions.
                // So we are more lenient in lexing here, and the validation steps below are more strict.
                // See: https://github.com/ros/genmsg/blob/7d8b6ce6f43b6e39ea8261125d270f2d3062356f/src/genmsg/msg_loader.py#L188-L238
                fieldOrType: /[a-zA-Z_][a-zA-Z0-9_]*(?:\/[a-zA-Z][a-zA-Z0-9_]*)?/
              });
              function extend(objs) {
                return objs.reduce((r, p) => ({ ...r, ...p }), {});
              }
              var grammar = {
                Lexer: lexer,
                ParserRules: [
                  { "name": "main$ebnf$1", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$1", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "boolType", "arrayType", "__", "field", "_", "main$ebnf$1", "simple"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$2", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$2", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "bigintType", "arrayType", "__", "field", "_", "main$ebnf$2", "simple"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$3", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$3", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "numericType", "arrayType", "__", "field", "_", "main$ebnf$3", "simple"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$4", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$4", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "stringType", "arrayType", "__", "field", "_", "main$ebnf$4", "simple"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$5", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$5", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "timeType", "arrayType", "__", "field", "_", "main$ebnf$5", "simple"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$6", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$6", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "customType", "arrayType", "__", "field", "_", "main$ebnf$6", "complex"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$7", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$7", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "boolType", "__", "constantField", "_", "boolConstantValue", "_", "main$ebnf$7"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$8", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$8", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "bigintType", "__", "constantField", "_", "bigintConstantValue", "_", "main$ebnf$8"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$9", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$9", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "numericType", "__", "constantField", "_", "numericConstantValue", "_", "main$ebnf$9"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main$ebnf$10", "symbols": ["comment"], "postprocess": id },
                  { "name": "main$ebnf$10", "symbols": [], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["_", "stringType", "__", "constantField", "_", "stringConstantValue", "_", "main$ebnf$10"], "postprocess": function(d) {
                    return extend(d);
                  } },
                  { "name": "main", "symbols": ["comment"], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "main", "symbols": ["blankLine"], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "boolType", "symbols": [{ "literal": "bool" }], "postprocess": function(d) {
                    return { type: d[0].value };
                  } },
                  { "name": "bigintType$subexpression$1", "symbols": [{ "literal": "int64" }] },
                  { "name": "bigintType$subexpression$1", "symbols": [{ "literal": "uint64" }] },
                  { "name": "bigintType", "symbols": ["bigintType$subexpression$1"], "postprocess": function(d) {
                    return { type: d[0][0].value };
                  } },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "byte" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "char" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "float32" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "float64" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "int8" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "uint8" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "int16" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "uint16" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "int32" }] },
                  { "name": "numericType$subexpression$1", "symbols": [{ "literal": "uint32" }] },
                  { "name": "numericType", "symbols": ["numericType$subexpression$1"], "postprocess": function(d) {
                    return { type: d[0][0].value };
                  } },
                  { "name": "stringType", "symbols": [{ "literal": "string" }], "postprocess": function(d) {
                    return { type: d[0].value };
                  } },
                  { "name": "timeType$subexpression$1", "symbols": [{ "literal": "time" }] },
                  { "name": "timeType$subexpression$1", "symbols": [{ "literal": "duration" }] },
                  { "name": "timeType", "symbols": ["timeType$subexpression$1"], "postprocess": function(d) {
                    return { type: d[0][0].value };
                  } },
                  { "name": "customType", "symbols": [lexer.has("fieldOrType") ? { type: "fieldOrType" } : fieldOrType], "postprocess": function(d, _, reject) {
                    const PRIMITIVE_TYPES = ["bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "string", "time", "duration"];
                    const type = d[0].value;
                    if (PRIMITIVE_TYPES.includes(type)) return reject;
                    return { type };
                  } },
                  { "name": "arrayType", "symbols": [{ "literal": "[" }, "_", { "literal": "]" }], "postprocess": function(d) {
                    return { isArray: true };
                  } },
                  { "name": "arrayType", "symbols": [{ "literal": "[" }, "_", "number", "_", { "literal": "]" }], "postprocess": function(d) {
                    return { isArray: true, arrayLength: d[2] };
                  } },
                  { "name": "arrayType", "symbols": ["_"], "postprocess": function(d) {
                    return { isArray: false };
                  } },
                  { "name": "field", "symbols": [lexer.has("fieldOrType") ? { type: "fieldOrType" } : fieldOrType], "postprocess": function(d, _, reject) {
                    const name = d[0].value;
                    if (name.match(/^[a-zA-Z][a-zA-Z0-9_]*$/) == void 0) return reject;
                    return { name };
                  } },
                  { "name": "constantField", "symbols": [lexer.has("fieldOrType") ? { type: "fieldOrType" } : fieldOrType], "postprocess": function(d, _, reject) {
                    const name = d[0].value;
                    if (name.match(/^[a-zA-Z_][a-zA-Z0-9_]*$/) == void 0) return reject;
                    return { name, isConstant: true };
                  } },
                  { "name": "boolConstantValue", "symbols": ["assignment"], "postprocess": function(d, _, reject) {
                    const valueText = d[0].split("#")[0].trim();
                    if (valueText === "True" || valueText === "1") return { value: true, valueText };
                    if (valueText === "False" || valueText === "0") return { value: false, valueText };
                    return reject;
                  } },
                  { "name": "numericConstantValue", "symbols": ["assignment"], "postprocess": function(d, _, reject) {
                    const valueText = d[0].split("#")[0].trim();
                    const value = parseFloat(valueText);
                    return !isNaN(value) ? { value, valueText } : reject;
                  } },
                  { "name": "bigintConstantValue", "symbols": ["assignment"], "postprocess": function(d, _, reject) {
                    const valueText = d[0].split("#")[0].trim();
                    try {
                      const value = BigInt(valueText);
                      return { value, valueText };
                    } catch {
                      return reject;
                    }
                  } },
                  { "name": "stringConstantValue", "symbols": ["assignment"], "postprocess": function(d) {
                    return { value: d[0], valueText: d[0] };
                  } },
                  { "name": "bool$subexpression$1", "symbols": [{ "literal": "True" }] },
                  { "name": "bool$subexpression$1", "symbols": [{ "literal": "1" }] },
                  { "name": "bool", "symbols": ["bool$subexpression$1"], "postprocess": function(d) {
                    return true;
                  } },
                  { "name": "bool$subexpression$2", "symbols": [{ "literal": "False" }] },
                  { "name": "bool$subexpression$2", "symbols": [{ "literal": "0" }] },
                  { "name": "bool", "symbols": ["bool$subexpression$2"], "postprocess": function(d) {
                    return false;
                  } },
                  { "name": "number", "symbols": [lexer.has("number") ? { type: "number" } : number], "postprocess": function(d) {
                    return parseFloat(d[0].value);
                  } },
                  { "name": "assignment", "symbols": [lexer.has("assignment") ? { type: "assignment" } : assignment], "postprocess": function(d) {
                    return d[0].value.substr(1).trim();
                  } },
                  { "name": "comment", "symbols": [lexer.has("comment") ? { type: "comment" } : comment], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "blankLine", "symbols": ["_"], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "_$subexpression$1", "symbols": [] },
                  { "name": "_$subexpression$1", "symbols": [lexer.has("space") ? { type: "space" } : space] },
                  { "name": "_", "symbols": ["_$subexpression$1"], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "__", "symbols": [lexer.has("space") ? { type: "space" } : space], "postprocess": function(d) {
                    return null;
                  } },
                  { "name": "simple", "symbols": [], "postprocess": function() {
                    return { isComplex: false };
                  } },
                  { "name": "complex", "symbols": [], "postprocess": function() {
                    return { isComplex: true };
                  } }
                ],
                ParserStart: "main"
              };
              if (typeof module2.exports !== "undefined") {
                module2.exports = grammar;
              } else {
                window.grammar = grammar;
              }
            })();
          })
        ),
        /***/
        694: (
          /***/
          (function(module2, exports2) {
            var __WEBPACK_AMD_DEFINE_FACTORY__, __WEBPACK_AMD_DEFINE_ARRAY__, __WEBPACK_AMD_DEFINE_RESULT__;
            (function(root, factory) {
              if (true) {
                !(__WEBPACK_AMD_DEFINE_ARRAY__ = [], __WEBPACK_AMD_DEFINE_FACTORY__ = factory, __WEBPACK_AMD_DEFINE_RESULT__ = typeof __WEBPACK_AMD_DEFINE_FACTORY__ === "function" ? __WEBPACK_AMD_DEFINE_FACTORY__.apply(exports2, __WEBPACK_AMD_DEFINE_ARRAY__) : __WEBPACK_AMD_DEFINE_FACTORY__, __WEBPACK_AMD_DEFINE_RESULT__ !== void 0 && (module2.exports = __WEBPACK_AMD_DEFINE_RESULT__));
              } else {
              }
            })(this, function() {
              "use strict";
              var hasOwnProperty = Object.prototype.hasOwnProperty;
              var toString = Object.prototype.toString;
              var hasSticky = typeof new RegExp().sticky === "boolean";
              function isRegExp(o) {
                return o && toString.call(o) === "[object RegExp]";
              }
              function isObject(o) {
                return o && typeof o === "object" && !isRegExp(o) && !Array.isArray(o);
              }
              function reEscape(s) {
                return s.replace(/[-\/\\^$*+?.()|[\]{}]/g, "\\$&");
              }
              function reGroups(s) {
                var re = new RegExp("|" + s);
                return re.exec("").length - 1;
              }
              function reCapture(s) {
                return "(" + s + ")";
              }
              function reUnion(regexps) {
                if (!regexps.length) return "(?!)";
                var source = regexps.map(function(s) {
                  return "(?:" + s + ")";
                }).join("|");
                return "(?:" + source + ")";
              }
              function regexpOrLiteral(obj) {
                if (typeof obj === "string") {
                  return "(?:" + reEscape(obj) + ")";
                } else if (isRegExp(obj)) {
                  if (obj.ignoreCase) throw new Error("RegExp /i flag not allowed");
                  if (obj.global) throw new Error("RegExp /g flag is implied");
                  if (obj.sticky) throw new Error("RegExp /y flag is implied");
                  if (obj.multiline) throw new Error("RegExp /m flag is implied");
                  return obj.source;
                } else {
                  throw new Error("Not a pattern: " + obj);
                }
              }
              function pad(s, length) {
                if (s.length > length) {
                  return s;
                }
                return Array(length - s.length + 1).join(" ") + s;
              }
              function lastNLines(string, numLines) {
                var position = string.length;
                var lineBreaks = 0;
                while (true) {
                  var idx = string.lastIndexOf("\n", position - 1);
                  if (idx === -1) {
                    break;
                  } else {
                    lineBreaks++;
                  }
                  position = idx;
                  if (lineBreaks === numLines) {
                    break;
                  }
                  if (position === 0) {
                    break;
                  }
                }
                var startPosition = lineBreaks < numLines ? 0 : position + 1;
                return string.substring(startPosition).split("\n");
              }
              function objectToRules(object) {
                var keys = Object.getOwnPropertyNames(object);
                var result = [];
                for (var i = 0; i < keys.length; i++) {
                  var key = keys[i];
                  var thing = object[key];
                  var rules = [].concat(thing);
                  if (key === "include") {
                    for (var j = 0; j < rules.length; j++) {
                      result.push({ include: rules[j] });
                    }
                    continue;
                  }
                  var match = [];
                  rules.forEach(function(rule) {
                    if (isObject(rule)) {
                      if (match.length) result.push(ruleOptions(key, match));
                      result.push(ruleOptions(key, rule));
                      match = [];
                    } else {
                      match.push(rule);
                    }
                  });
                  if (match.length) result.push(ruleOptions(key, match));
                }
                return result;
              }
              function arrayToRules(array) {
                var result = [];
                for (var i = 0; i < array.length; i++) {
                  var obj = array[i];
                  if (obj.include) {
                    var include = [].concat(obj.include);
                    for (var j = 0; j < include.length; j++) {
                      result.push({ include: include[j] });
                    }
                    continue;
                  }
                  if (!obj.type) {
                    throw new Error("Rule has no type: " + JSON.stringify(obj));
                  }
                  result.push(ruleOptions(obj.type, obj));
                }
                return result;
              }
              function ruleOptions(type, obj) {
                if (!isObject(obj)) {
                  obj = { match: obj };
                }
                if (obj.include) {
                  throw new Error("Matching rules cannot also include states");
                }
                var options = {
                  defaultType: type,
                  lineBreaks: !!obj.error || !!obj.fallback,
                  pop: false,
                  next: null,
                  push: null,
                  error: false,
                  fallback: false,
                  value: null,
                  type: null,
                  shouldThrow: false
                };
                for (var key in obj) {
                  if (hasOwnProperty.call(obj, key)) {
                    options[key] = obj[key];
                  }
                }
                if (typeof options.type === "string" && type !== options.type) {
                  throw new Error("Type transform cannot be a string (type '" + options.type + "' for token '" + type + "')");
                }
                var match = options.match;
                options.match = Array.isArray(match) ? match : match ? [match] : [];
                options.match.sort(function(a, b) {
                  return isRegExp(a) && isRegExp(b) ? 0 : isRegExp(b) ? -1 : isRegExp(a) ? 1 : b.length - a.length;
                });
                return options;
              }
              function toRules(spec) {
                return Array.isArray(spec) ? arrayToRules(spec) : objectToRules(spec);
              }
              var defaultErrorRule = ruleOptions("error", { lineBreaks: true, shouldThrow: true });
              function compileRules(rules, hasStates) {
                var errorRule = null;
                var fast = /* @__PURE__ */ Object.create(null);
                var fastAllowed = true;
                var unicodeFlag = null;
                var groups = [];
                var parts = [];
                for (var i = 0; i < rules.length; i++) {
                  if (rules[i].fallback) {
                    fastAllowed = false;
                  }
                }
                for (var i = 0; i < rules.length; i++) {
                  var options = rules[i];
                  if (options.include) {
                    throw new Error("Inheritance is not allowed in stateless lexers");
                  }
                  if (options.error || options.fallback) {
                    if (errorRule) {
                      if (!options.fallback === !errorRule.fallback) {
                        throw new Error("Multiple " + (options.fallback ? "fallback" : "error") + " rules not allowed (for token '" + options.defaultType + "')");
                      } else {
                        throw new Error("fallback and error are mutually exclusive (for token '" + options.defaultType + "')");
                      }
                    }
                    errorRule = options;
                  }
                  var match = options.match.slice();
                  if (fastAllowed) {
                    while (match.length && typeof match[0] === "string" && match[0].length === 1) {
                      var word = match.shift();
                      fast[word.charCodeAt(0)] = options;
                    }
                  }
                  if (options.pop || options.push || options.next) {
                    if (!hasStates) {
                      throw new Error("State-switching options are not allowed in stateless lexers (for token '" + options.defaultType + "')");
                    }
                    if (options.fallback) {
                      throw new Error("State-switching options are not allowed on fallback tokens (for token '" + options.defaultType + "')");
                    }
                  }
                  if (match.length === 0) {
                    continue;
                  }
                  fastAllowed = false;
                  groups.push(options);
                  for (var j = 0; j < match.length; j++) {
                    var obj = match[j];
                    if (!isRegExp(obj)) {
                      continue;
                    }
                    if (unicodeFlag === null) {
                      unicodeFlag = obj.unicode;
                    } else if (unicodeFlag !== obj.unicode && options.fallback === false) {
                      throw new Error("If one rule is /u then all must be");
                    }
                  }
                  var pat = reUnion(match.map(regexpOrLiteral));
                  var regexp = new RegExp(pat);
                  if (regexp.test("")) {
                    throw new Error("RegExp matches empty string: " + regexp);
                  }
                  var groupCount = reGroups(pat);
                  if (groupCount > 0) {
                    throw new Error("RegExp has capture groups: " + regexp + "\nUse (?: \u2026 ) instead");
                  }
                  if (!options.lineBreaks && regexp.test("\n")) {
                    throw new Error("Rule should declare lineBreaks: " + regexp);
                  }
                  parts.push(reCapture(pat));
                }
                var fallbackRule = errorRule && errorRule.fallback;
                var flags = hasSticky && !fallbackRule ? "ym" : "gm";
                var suffix = hasSticky || fallbackRule ? "" : "|";
                if (unicodeFlag === true) flags += "u";
                var combined = new RegExp(reUnion(parts) + suffix, flags);
                return { regexp: combined, groups, fast, error: errorRule || defaultErrorRule };
              }
              function compile(rules) {
                var result = compileRules(toRules(rules));
                return new Lexer({ start: result }, "start");
              }
              function checkStateGroup(g, name, map) {
                var state = g && (g.push || g.next);
                if (state && !map[state]) {
                  throw new Error("Missing state '" + state + "' (in token '" + g.defaultType + "' of state '" + name + "')");
                }
                if (g && g.pop && +g.pop !== 1) {
                  throw new Error("pop must be 1 (in token '" + g.defaultType + "' of state '" + name + "')");
                }
              }
              function compileStates(states, start) {
                var all = states.$all ? toRules(states.$all) : [];
                delete states.$all;
                var keys = Object.getOwnPropertyNames(states);
                if (!start) start = keys[0];
                var ruleMap = /* @__PURE__ */ Object.create(null);
                for (var i = 0; i < keys.length; i++) {
                  var key = keys[i];
                  ruleMap[key] = toRules(states[key]).concat(all);
                }
                for (var i = 0; i < keys.length; i++) {
                  var key = keys[i];
                  var rules = ruleMap[key];
                  var included = /* @__PURE__ */ Object.create(null);
                  for (var j = 0; j < rules.length; j++) {
                    var rule = rules[j];
                    if (!rule.include) continue;
                    var splice = [j, 1];
                    if (rule.include !== key && !included[rule.include]) {
                      included[rule.include] = true;
                      var newRules = ruleMap[rule.include];
                      if (!newRules) {
                        throw new Error("Cannot include nonexistent state '" + rule.include + "' (in state '" + key + "')");
                      }
                      for (var k = 0; k < newRules.length; k++) {
                        var newRule = newRules[k];
                        if (rules.indexOf(newRule) !== -1) continue;
                        splice.push(newRule);
                      }
                    }
                    rules.splice.apply(rules, splice);
                    j--;
                  }
                }
                var map = /* @__PURE__ */ Object.create(null);
                for (var i = 0; i < keys.length; i++) {
                  var key = keys[i];
                  map[key] = compileRules(ruleMap[key], true);
                }
                for (var i = 0; i < keys.length; i++) {
                  var name = keys[i];
                  var state = map[name];
                  var groups = state.groups;
                  for (var j = 0; j < groups.length; j++) {
                    checkStateGroup(groups[j], name, map);
                  }
                  var fastKeys = Object.getOwnPropertyNames(state.fast);
                  for (var j = 0; j < fastKeys.length; j++) {
                    checkStateGroup(state.fast[fastKeys[j]], name, map);
                  }
                }
                return new Lexer(map, start);
              }
              function keywordTransform(map) {
                var isMap = typeof Map !== "undefined";
                var reverseMap = isMap ? /* @__PURE__ */ new Map() : /* @__PURE__ */ Object.create(null);
                var types = Object.getOwnPropertyNames(map);
                for (var i = 0; i < types.length; i++) {
                  var tokenType = types[i];
                  var item = map[tokenType];
                  var keywordList = Array.isArray(item) ? item : [item];
                  keywordList.forEach(function(keyword) {
                    if (typeof keyword !== "string") {
                      throw new Error("keyword must be string (in keyword '" + tokenType + "')");
                    }
                    if (isMap) {
                      reverseMap.set(keyword, tokenType);
                    } else {
                      reverseMap[keyword] = tokenType;
                    }
                  });
                }
                return function(k) {
                  return isMap ? reverseMap.get(k) : reverseMap[k];
                };
              }
              var Lexer = function(states, state) {
                this.startState = state;
                this.states = states;
                this.buffer = "";
                this.stack = [];
                this.reset();
              };
              Lexer.prototype.reset = function(data, info) {
                this.buffer = data || "";
                this.index = 0;
                this.line = info ? info.line : 1;
                this.col = info ? info.col : 1;
                this.queuedToken = info ? info.queuedToken : null;
                this.queuedText = info ? info.queuedText : "";
                this.queuedThrow = info ? info.queuedThrow : null;
                this.setState(info ? info.state : this.startState);
                this.stack = info && info.stack ? info.stack.slice() : [];
                return this;
              };
              Lexer.prototype.save = function() {
                return {
                  line: this.line,
                  col: this.col,
                  state: this.state,
                  stack: this.stack.slice(),
                  queuedToken: this.queuedToken,
                  queuedText: this.queuedText,
                  queuedThrow: this.queuedThrow
                };
              };
              Lexer.prototype.setState = function(state) {
                if (!state || this.state === state) return;
                this.state = state;
                var info = this.states[state];
                this.groups = info.groups;
                this.error = info.error;
                this.re = info.regexp;
                this.fast = info.fast;
              };
              Lexer.prototype.popState = function() {
                this.setState(this.stack.pop());
              };
              Lexer.prototype.pushState = function(state) {
                this.stack.push(this.state);
                this.setState(state);
              };
              var eat = hasSticky ? function(re, buffer) {
                return re.exec(buffer);
              } : function(re, buffer) {
                var match = re.exec(buffer);
                if (match[0].length === 0) {
                  return null;
                }
                return match;
              };
              Lexer.prototype._getGroup = function(match) {
                var groupCount = this.groups.length;
                for (var i = 0; i < groupCount; i++) {
                  if (match[i + 1] !== void 0) {
                    return this.groups[i];
                  }
                }
                throw new Error("Cannot find token type for matched text");
              };
              function tokenToString() {
                return this.value;
              }
              Lexer.prototype.next = function() {
                var index = this.index;
                if (this.queuedGroup) {
                  var token = this._token(this.queuedGroup, this.queuedText, index);
                  this.queuedGroup = null;
                  this.queuedText = "";
                  return token;
                }
                var buffer = this.buffer;
                if (index === buffer.length) {
                  return;
                }
                var group = this.fast[buffer.charCodeAt(index)];
                if (group) {
                  return this._token(group, buffer.charAt(index), index);
                }
                var re = this.re;
                re.lastIndex = index;
                var match = eat(re, buffer);
                var error = this.error;
                if (match == null) {
                  return this._token(error, buffer.slice(index, buffer.length), index);
                }
                var group = this._getGroup(match);
                var text = match[0];
                if (error.fallback && match.index !== index) {
                  this.queuedGroup = group;
                  this.queuedText = text;
                  return this._token(error, buffer.slice(index, match.index), index);
                }
                return this._token(group, text, index);
              };
              Lexer.prototype._token = function(group, text, offset) {
                var lineBreaks = 0;
                if (group.lineBreaks) {
                  var matchNL = /\n/g;
                  var nl = 1;
                  if (text === "\n") {
                    lineBreaks = 1;
                  } else {
                    while (matchNL.exec(text)) {
                      lineBreaks++;
                      nl = matchNL.lastIndex;
                    }
                  }
                }
                var token = {
                  type: typeof group.type === "function" && group.type(text) || group.defaultType,
                  value: typeof group.value === "function" ? group.value(text) : text,
                  text,
                  toString: tokenToString,
                  offset,
                  lineBreaks,
                  line: this.line,
                  col: this.col
                };
                var size = text.length;
                this.index += size;
                this.line += lineBreaks;
                if (lineBreaks !== 0) {
                  this.col = size - nl + 1;
                } else {
                  this.col += size;
                }
                if (group.shouldThrow) {
                  var err = new Error(this.formatError(token, "invalid syntax"));
                  throw err;
                }
                if (group.pop) this.popState();
                else if (group.push) this.pushState(group.push);
                else if (group.next) this.setState(group.next);
                return token;
              };
              if (typeof Symbol !== "undefined" && Symbol.iterator) {
                var LexerIterator = function(lexer) {
                  this.lexer = lexer;
                };
                LexerIterator.prototype.next = function() {
                  var token = this.lexer.next();
                  return { value: token, done: !token };
                };
                LexerIterator.prototype[Symbol.iterator] = function() {
                  return this;
                };
                Lexer.prototype[Symbol.iterator] = function() {
                  return new LexerIterator(this);
                };
              }
              Lexer.prototype.formatError = function(token, message) {
                if (token == null) {
                  var text = this.buffer.slice(this.index);
                  var token = {
                    text,
                    offset: this.index,
                    lineBreaks: text.indexOf("\n") === -1 ? 0 : 1,
                    line: this.line,
                    col: this.col
                  };
                }
                var numLinesAround = 2;
                var firstDisplayedLine = Math.max(token.line - numLinesAround, 1);
                var lastDisplayedLine = token.line + numLinesAround;
                var lastLineDigits = String(lastDisplayedLine).length;
                var displayedLines = lastNLines(
                  this.buffer,
                  this.line - token.line + numLinesAround + 1
                ).slice(0, 5);
                var errorLines = [];
                errorLines.push(message + " at line " + token.line + " col " + token.col + ":");
                errorLines.push("");
                for (var i = 0; i < displayedLines.length; i++) {
                  var line = displayedLines[i];
                  var lineNo = firstDisplayedLine + i;
                  errorLines.push(pad(String(lineNo), lastLineDigits) + "  " + line);
                  if (lineNo === token.line) {
                    errorLines.push(pad("", lastLineDigits + token.col + 1) + "^");
                  }
                }
                return errorLines.join("\n");
              };
              Lexer.prototype.clone = function() {
                return new Lexer(this.states, this.state);
              };
              Lexer.prototype.has = function(tokenType) {
                return true;
              };
              return {
                compile,
                states: compileStates,
                error: Object.freeze({ error: true }),
                fallback: Object.freeze({ fallback: true }),
                keywords: keywordTransform
              };
            });
          })
        ),
        /***/
        812: (
          /***/
          ((__unused_webpack_module, exports2) => {
            "use strict";
            Object.defineProperty(exports2, "__esModule", { value: true });
            exports2.stringify = stringify;
            function stringify(msgDefs) {
              let output = "";
              for (let i = 0; i < msgDefs.length; i++) {
                const msgDef = msgDefs[i];
                const constants = msgDef.definitions.filter(({ isConstant }) => isConstant);
                const variables = msgDef.definitions.filter(({ isConstant }) => isConstant == void 0 || !isConstant);
                if (i > 0) {
                  output += "\n================================================================================\n";
                  output += `MSG: ${msgDef.name ?? ""}
`;
                }
                for (const def of constants) {
                  output += `${def.type} ${def.name} = ${def.valueText ?? String(def.value)}
`;
                }
                if (variables.length > 0) {
                  if (output.length > 0) {
                    output += "\n";
                  }
                  for (const def of variables) {
                    const upperBound = def.upperBound != void 0 ? `<=${def.upperBound}` : "";
                    const arrayLength = def.arrayLength != void 0 ? String(def.arrayLength) : def.arrayUpperBound != void 0 ? `<=${def.arrayUpperBound}` : "";
                    const array = def.isArray === true ? `[${arrayLength}]` : "";
                    const defaultValue = def.defaultValue != void 0 ? ` ${stringifyDefaultValue(def.defaultValue)}` : "";
                    output += `${def.type}${upperBound}${array} ${def.name}${defaultValue}
`;
                  }
                }
              }
              return output.trimEnd();
            }
            function stringifyDefaultValue(value) {
              if (Array.isArray(value)) {
                return `[${value.map((x) => typeof x === "bigint" ? x.toString() : JSON.stringify(x)).join(", ")}]`;
              }
              return typeof value === "bigint" ? value.toString() : JSON.stringify(value);
            }
          })
        ),
        /***/
        881: (
          /***/
          (function(__unused_webpack_module, exports2, __webpack_require__2) {
            "use strict";
            var __createBinding = this && this.__createBinding || (Object.create ? (function(o, m, k, k2) {
              if (k2 === void 0) k2 = k;
              var desc = Object.getOwnPropertyDescriptor(m, k);
              if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
                desc = { enumerable: true, get: function() {
                  return m[k];
                } };
              }
              Object.defineProperty(o, k2, desc);
            }) : (function(o, m, k, k2) {
              if (k2 === void 0) k2 = k;
              o[k2] = m[k];
            }));
            var __exportStar = this && this.__exportStar || function(m, exports3) {
              for (var p in m) if (p !== "default" && !Object.prototype.hasOwnProperty.call(exports3, p)) __createBinding(exports3, m, p);
            };
            Object.defineProperty(exports2, "__esModule", { value: true });
            __exportStar(__webpack_require__2(425), exports2);
            __exportStar(__webpack_require__2(954), exports2);
            __exportStar(__webpack_require__2(812), exports2);
          })
        ),
        /***/
        900: (
          /***/
          ((__unused_webpack_module, exports2) => {
            "use strict";
            Object.defineProperty(exports2, "__esModule", { value: true });
            exports2.isMsgDefFieldEqual = isMsgDefFieldEqual;
            exports2.isMsgDefEqual = isMsgDefEqual;
            function defaultValuesEqual(lhs, rhs) {
              if (Array.isArray(lhs) && Array.isArray(rhs)) {
                if (lhs.length !== rhs.length) {
                  return false;
                }
                for (let i = 0; i < lhs.length; i++) {
                  if (lhs[i] !== rhs[i]) {
                    return false;
                  }
                }
                return true;
              }
              return lhs === rhs;
            }
            function isMsgDefFieldEqual(lhs, rhs) {
              return lhs.type === rhs.type && lhs.name === rhs.name && (lhs.isComplex ?? false) === (rhs.isComplex ?? false) && lhs.enumType === rhs.enumType && (lhs.isArray ?? false) === (rhs.isArray ?? false) && lhs.arrayLength === rhs.arrayLength && (lhs.isConstant ?? false) === (rhs.isConstant ?? false) && lhs.value === rhs.value && lhs.valueText === rhs.valueText && lhs.upperBound === rhs.upperBound && lhs.arrayUpperBound === rhs.arrayUpperBound && (lhs.deprecated ?? false) === (rhs.deprecated ?? false) && defaultValuesEqual(lhs.defaultValue, rhs.defaultValue);
            }
            function isMsgDefEqual(lhs, rhs) {
              return (lhs.name == void 0 || rhs.name == void 0 || lhs.name === rhs.name) && lhs.definitions.length === rhs.definitions.length && lhs.definitions.every((def, i) => isMsgDefFieldEqual(def, rhs.definitions[i]));
            }
          })
        ),
        /***/
        954: (
          /***/
          (function(__unused_webpack_module, exports2, __webpack_require__2) {
            "use strict";
            var __importDefault = this && this.__importDefault || function(mod) {
              return mod && mod.__esModule ? mod : { "default": mod };
            };
            Object.defineProperty(exports2, "__esModule", { value: true });
            exports2.parse = parse2;
            exports2.fixupTypes = fixupTypes;
            exports2.normalizeType = normalizeType;
            const message_definition_1 = __webpack_require__2(155);
            const nearley_1 = __webpack_require__2(662);
            const buildRos2Type_1 = __webpack_require__2(91);
            const ros1_ne_1 = __importDefault(__webpack_require__2(666));
            const ROS1_GRAMMAR = nearley_1.Grammar.fromCompiled(ros1_ne_1.default);
            function parse2(messageDefinition, options = {}) {
              const allLines = messageDefinition.split("\n").map((line) => line.trim()).filter((line) => line);
              let definitionLines = [];
              const types = [];
              allLines.forEach((line) => {
                if (line.startsWith("#")) {
                  return;
                }
                if (line.startsWith("==")) {
                  types.push(options.ros2 === true ? (0, buildRos2Type_1.buildRos2Type)(definitionLines) : buildType(definitionLines, ROS1_GRAMMAR));
                  definitionLines = [];
                } else {
                  definitionLines.push({ line });
                }
              });
              types.push(options.ros2 === true ? (0, buildRos2Type_1.buildRos2Type)(definitionLines) : buildType(definitionLines, ROS1_GRAMMAR));
              const seenTypes = [];
              const uniqueTypes = types.filter((definition) => {
                return seenTypes.find((otherDefinition) => (0, message_definition_1.isMsgDefEqual)(definition, otherDefinition)) ? false : seenTypes.push(definition);
              });
              if (options.skipTypeFixup !== true) {
                fixupTypes(uniqueTypes);
              }
              return uniqueTypes;
            }
            function fixupTypes(types) {
              types.forEach(({ definitions, name }) => {
                definitions.forEach((definition) => {
                  if (definition.isComplex === true) {
                    const typeNamespace = name?.split("/").slice(0, -1).join("/");
                    const foundName = findTypeByName(types, definition.type, typeNamespace).name;
                    if (foundName == void 0) {
                      throw new Error(`Missing type definition for ${definition.type}`);
                    }
                    definition.type = foundName;
                  }
                });
              });
            }
            function buildType(lines, grammar) {
              const definitions = [];
              let complexTypeName;
              lines.forEach(({ line }) => {
                if (line.startsWith("MSG:")) {
                  const [_, name] = simpleTokenization(line);
                  complexTypeName = name?.trim();
                  return;
                }
                const parser = new nearley_1.Parser(grammar);
                parser.feed(line);
                const results = parser.finish();
                if (results.length === 0) {
                  throw new Error(`Could not parse line: '${line}'`);
                } else if (results.length > 1) {
                  throw new Error(`Ambiguous line: '${line}'`);
                }
                const result = results[0];
                if (result != void 0) {
                  result.type = normalizeType(result.type);
                  definitions.push(result);
                }
              });
              return { name: complexTypeName, definitions };
            }
            function simpleTokenization(line) {
              return line.replace(/#.*/gi, "").split(" ").filter((word) => word);
            }
            function findTypeByName(types, name, typeNamespace) {
              const matches = types.filter((type) => {
                const typeName = type.name ?? "";
                if (name.length === 0) {
                  return typeName.length === 0;
                }
                if (name.includes("/")) {
                  return typeName === name;
                } else if (name === "Header") {
                  return typeName === `std_msgs/Header`;
                } else if (typeNamespace) {
                  return typeName === `${typeNamespace}/${name}`;
                } else {
                  return typeName.endsWith(`/${name}`);
                }
              });
              if (matches[0] == void 0) {
                throw new Error(`Expected 1 top level type definition for '${name}' but found ${matches.length}`);
              }
              if (matches.length > 1) {
                throw new Error(`Cannot unambiguously determine fully-qualified type name for '${name}'`);
              }
              return matches[0];
            }
            function normalizeType(type) {
              if (type === "char") {
                return "uint8";
              } else if (type === "byte") {
                return "int8";
              }
              return type;
            }
          })
        )
        /******/
      };
      var __webpack_module_cache__ = {};
      function __webpack_require__(moduleId) {
        var cachedModule = __webpack_module_cache__[moduleId];
        if (cachedModule !== void 0) {
          return cachedModule.exports;
        }
        var module2 = __webpack_module_cache__[moduleId] = {
          /******/
          // no module.id needed
          /******/
          // no module.loaded needed
          /******/
          exports: {}
          /******/
        };
        __webpack_modules__[moduleId].call(module2.exports, module2, module2.exports, __webpack_require__);
        return module2.exports;
      }
      (() => {
        __webpack_require__.d = (exports2, definition) => {
          for (var key in definition) {
            if (__webpack_require__.o(definition, key) && !__webpack_require__.o(exports2, key)) {
              Object.defineProperty(exports2, key, { enumerable: true, get: definition[key] });
            }
          }
        };
      })();
      (() => {
        __webpack_require__.o = (obj, prop) => Object.prototype.hasOwnProperty.call(obj, prop);
      })();
      (() => {
        __webpack_require__.r = (exports2) => {
          if (typeof Symbol !== "undefined" && Symbol.toStringTag) {
            Object.defineProperty(exports2, Symbol.toStringTag, { value: "Module" });
          }
          Object.defineProperty(exports2, "__esModule", { value: true });
        };
      })();
      var __webpack_exports__ = __webpack_require__(881);
      module.exports = __webpack_exports__;
    })();
  }
});

// node_modules/@foxglove/cdr/dist/EncapsulationKind.js
var require_EncapsulationKind = __commonJS({
  "node_modules/@foxglove/cdr/dist/EncapsulationKind.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.EncapsulationKind = void 0;
    var EncapsulationKind;
    (function(EncapsulationKind2) {
      EncapsulationKind2[EncapsulationKind2["CDR_BE"] = 0] = "CDR_BE";
      EncapsulationKind2[EncapsulationKind2["CDR_LE"] = 1] = "CDR_LE";
      EncapsulationKind2[EncapsulationKind2["PL_CDR_BE"] = 2] = "PL_CDR_BE";
      EncapsulationKind2[EncapsulationKind2["PL_CDR_LE"] = 3] = "PL_CDR_LE";
      EncapsulationKind2[EncapsulationKind2["CDR2_BE"] = 16] = "CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["CDR2_LE"] = 17] = "CDR2_LE";
      EncapsulationKind2[EncapsulationKind2["PL_CDR2_BE"] = 18] = "PL_CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["PL_CDR2_LE"] = 19] = "PL_CDR2_LE";
      EncapsulationKind2[EncapsulationKind2["DELIMITED_CDR2_BE"] = 20] = "DELIMITED_CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["DELIMITED_CDR2_LE"] = 21] = "DELIMITED_CDR2_LE";
      EncapsulationKind2[EncapsulationKind2["RTPS_CDR2_BE"] = 6] = "RTPS_CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["RTPS_CDR2_LE"] = 7] = "RTPS_CDR2_LE";
      EncapsulationKind2[EncapsulationKind2["RTPS_DELIMITED_CDR2_BE"] = 8] = "RTPS_DELIMITED_CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["RTPS_DELIMITED_CDR2_LE"] = 9] = "RTPS_DELIMITED_CDR2_LE";
      EncapsulationKind2[EncapsulationKind2["RTPS_PL_CDR2_BE"] = 10] = "RTPS_PL_CDR2_BE";
      EncapsulationKind2[EncapsulationKind2["RTPS_PL_CDR2_LE"] = 11] = "RTPS_PL_CDR2_LE";
    })(EncapsulationKind || (exports.EncapsulationKind = EncapsulationKind = {}));
  }
});

// node_modules/@foxglove/cdr/dist/getEncapsulationKindInfo.js
var require_getEncapsulationKindInfo = __commonJS({
  "node_modules/@foxglove/cdr/dist/getEncapsulationKindInfo.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.getEncapsulationKindInfo = void 0;
    var EncapsulationKind_1 = require_EncapsulationKind();
    var getEncapsulationKindInfo = (kind) => {
      const isCDR2 = kind > EncapsulationKind_1.EncapsulationKind.PL_CDR_LE;
      const littleEndian = kind === EncapsulationKind_1.EncapsulationKind.CDR_LE || kind === EncapsulationKind_1.EncapsulationKind.PL_CDR_LE || kind === EncapsulationKind_1.EncapsulationKind.CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.PL_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.DELIMITED_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_PL_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_DELIMITED_CDR2_LE;
      const isDelimitedCDR2 = kind === EncapsulationKind_1.EncapsulationKind.DELIMITED_CDR2_BE || kind === EncapsulationKind_1.EncapsulationKind.DELIMITED_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_DELIMITED_CDR2_BE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_DELIMITED_CDR2_LE;
      const isPLCDR2 = kind === EncapsulationKind_1.EncapsulationKind.PL_CDR2_BE || kind === EncapsulationKind_1.EncapsulationKind.PL_CDR2_LE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_PL_CDR2_BE || kind === EncapsulationKind_1.EncapsulationKind.RTPS_PL_CDR2_LE;
      const isPLCDR1 = kind === EncapsulationKind_1.EncapsulationKind.PL_CDR_BE || kind === EncapsulationKind_1.EncapsulationKind.PL_CDR_LE;
      const usesDelimiterHeader = isDelimitedCDR2 || isPLCDR2;
      const usesMemberHeader = isPLCDR2 || isPLCDR1;
      return {
        isCDR2,
        littleEndian,
        usesDelimiterHeader,
        usesMemberHeader
      };
    };
    exports.getEncapsulationKindInfo = getEncapsulationKindInfo;
  }
});

// node_modules/@foxglove/cdr/dist/isBigEndian.js
var require_isBigEndian = __commonJS({
  "node_modules/@foxglove/cdr/dist/isBigEndian.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.isBigEndian = isBigEndian;
    var endianTestArray = new Uint8Array(4);
    var endianTestView = new Uint32Array(endianTestArray.buffer);
    endianTestView[0] = 1;
    function isBigEndian() {
      return endianTestArray[3] === 1;
    }
  }
});

// node_modules/@foxglove/cdr/dist/lengthCodes.js
var require_lengthCodes = __commonJS({
  "node_modules/@foxglove/cdr/dist/lengthCodes.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.lengthCodeToObjectSizes = void 0;
    exports.getLengthCodeForObjectSize = getLengthCodeForObjectSize;
    function getLengthCodeForObjectSize(objectSize) {
      let defaultLengthCode;
      switch (objectSize) {
        case 1:
          defaultLengthCode = 0;
          break;
        case 2:
          defaultLengthCode = 1;
          break;
        case 4:
          defaultLengthCode = 2;
          break;
        case 8:
          defaultLengthCode = 3;
          break;
      }
      if (defaultLengthCode == void 0) {
        if (objectSize > 4294967295) {
          throw Error(`Object size ${objectSize} for EMHEADER too large without specifying length code. Max size is ${4294967295}`);
        }
        defaultLengthCode = 4;
      }
      return defaultLengthCode;
    }
    exports.lengthCodeToObjectSizes = {
      0: 1,
      1: 2,
      2: 4,
      3: 8
    };
  }
});

// node_modules/@foxglove/cdr/dist/reservedPIDs.js
var require_reservedPIDs = __commonJS({
  "node_modules/@foxglove/cdr/dist/reservedPIDs.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.SENTINEL_PID = exports.EXTENDED_PID = void 0;
    exports.EXTENDED_PID = 16129;
    exports.SENTINEL_PID = 16130;
  }
});

// node_modules/@foxglove/cdr/dist/CdrReader.js
var require_CdrReader = __commonJS({
  "node_modules/@foxglove/cdr/dist/CdrReader.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.CdrReader = void 0;
    var getEncapsulationKindInfo_1 = require_getEncapsulationKindInfo();
    var isBigEndian_1 = require_isBigEndian();
    var lengthCodes_1 = require_lengthCodes();
    var reservedPIDs_1 = require_reservedPIDs();
    var textDecoder = new TextDecoder("utf8");
    var CdrReader2 = class _CdrReader {
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
        } else {
          return this.memberHeaderV1();
        }
      }
      /** XCDR1 PL_CDR encapsulation parameter header*/
      memberHeaderV1() {
        this.align(4);
        const idHeader = this.uint16();
        const mustUnderstandFlag = (idHeader & 16384) >> 14 === 1;
        const implementationSpecificFlag = (idHeader & 32768) >> 15 === 1;
        const extendedPIDFlag = (idHeader & 16383) === reservedPIDs_1.EXTENDED_PID;
        const sentinelPIDFlag = (idHeader & 16383) === reservedPIDs_1.SENTINEL_PID;
        if (sentinelPIDFlag) {
          this.uint16();
          return { id: reservedPIDs_1.SENTINEL_PID, objectSize: 0, mustUnderstand: false, readSentinelHeader: true };
        }
        const usesReservedParameterId = (idHeader & 16383) > reservedPIDs_1.SENTINEL_PID;
        if (usesReservedParameterId || implementationSpecificFlag) {
          throw new Error(`Unsupported parameter ID header ${idHeader.toString(16)}`);
        }
        if (extendedPIDFlag) {
          this.uint16();
        }
        const id = extendedPIDFlag ? this.uint32() : idHeader & 16383;
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
          const sentinelPIDFlag = (header & 16383) === reservedPIDs_1.SENTINEL_PID;
          if (!sentinelPIDFlag) {
            return false;
          }
          this.uint16();
          return true;
        } else {
          return false;
        }
      }
      memberHeaderV2() {
        const header = this.uint32();
        const mustUnderstand = Math.abs((header & 2147483648) >> 31) === 1;
        const lengthCode = (header & 1879048192) >> 28;
        const id = header & 268435455;
        const objectSize = this.emHeaderObjectSize(lengthCode);
        return { mustUnderstand, id, objectSize, lengthCode };
      }
      /** Uses the length code to derive the member object size in
       * the EMHEADER, sometimes reading NEXTINT (the next uint32
       * following the header) from the buffer */
      emHeaderObjectSize(lengthCode) {
        switch (lengthCode) {
          case 0:
          case 1:
          case 2:
          case 3:
            return lengthCodes_1.lengthCodeToObjectSizes[lengthCode];
          // LC > 3 -> NEXTINT exists after header
          case 4:
          case 5:
            return this.uint32();
          case 6:
            return 4 * this.uint32();
          case 7:
            return 8 * this.uint32();
          default:
            throw new Error(
              // eslint-disable-next-line @typescript-eslint/restrict-template-expressions
              `Invalid length code ${lengthCode} in EMHEADER at offset ${this.offset - 4}`
            );
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
        const clone = new _CdrReader(this.view);
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
        } else {
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
          return this.typedArraySlow(TypedArrayConstructor, getter, count);
        } else if (totalOffset % TypedArrayConstructor.BYTES_PER_ELEMENT === 0) {
          const array = new TypedArrayConstructor(this.view.buffer, totalOffset, count);
          this.offset += TypedArrayConstructor.BYTES_PER_ELEMENT * count;
          return array;
        } else {
          return this.typedArrayUnaligned(TypedArrayConstructor, getter, count);
        }
      }
      typedArrayUnaligned(TypedArrayConstructor, getter, count) {
        if (count < 10) {
          return this.typedArraySlow(TypedArrayConstructor, getter, count);
        }
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
    };
    exports.CdrReader = CdrReader2;
  }
});

// node_modules/@foxglove/cdr/dist/CdrSizeCalculator.js
var require_CdrSizeCalculator = __commonJS({
  "node_modules/@foxglove/cdr/dist/CdrSizeCalculator.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.CdrSizeCalculator = void 0;
    var CdrSizeCalculator = class {
      constructor() {
        this.offset = 4;
      }
      get size() {
        return this.offset;
      }
      int8() {
        return this.incrementAndReturn(1);
      }
      uint8() {
        return this.incrementAndReturn(1);
      }
      int16() {
        return this.incrementAndReturn(2);
      }
      uint16() {
        return this.incrementAndReturn(2);
      }
      int32() {
        return this.incrementAndReturn(4);
      }
      uint32() {
        return this.incrementAndReturn(4);
      }
      int64() {
        return this.incrementAndReturn(8);
      }
      uint64() {
        return this.incrementAndReturn(8);
      }
      float32() {
        return this.incrementAndReturn(4);
      }
      float64() {
        return this.incrementAndReturn(8);
      }
      string(length) {
        this.uint32();
        this.offset += length + 1;
        return this.offset;
      }
      sequenceLength() {
        return this.uint32();
      }
      // Increments the offset by `byteCount` and any required padding bytes and
      // returns the new offset
      incrementAndReturn(byteCount) {
        const alignment = (this.offset - 4) % byteCount;
        if (alignment > 0) {
          this.offset += byteCount - alignment;
        }
        this.offset += byteCount;
        return this.offset;
      }
    };
    exports.CdrSizeCalculator = CdrSizeCalculator;
  }
});

// node_modules/@foxglove/cdr/dist/CdrWriter.js
var require_CdrWriter = __commonJS({
  "node_modules/@foxglove/cdr/dist/CdrWriter.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.CdrWriter = void 0;
    var EncapsulationKind_1 = require_EncapsulationKind();
    var getEncapsulationKindInfo_1 = require_getEncapsulationKindInfo();
    var isBigEndian_1 = require_isBigEndian();
    var lengthCodes_1 = require_lengthCodes();
    var reservedPIDs_1 = require_reservedPIDs();
    var textEncoder = new TextEncoder();
    function stringLengthUtf8(str) {
      let byteLength = 0;
      const numCodeUnits = str.length;
      for (let i = 0; i < numCodeUnits; i++) {
        const codeUnit = str.charCodeAt(i);
        if (codeUnit <= 127) {
          byteLength += 1;
        } else if (codeUnit <= 2047) {
          byteLength += 2;
        } else if (55296 <= codeUnit && codeUnit <= 56319) {
          const nextCodeUnit = str.charCodeAt(i + 1);
          if (56320 <= nextCodeUnit && nextCodeUnit <= 57343) {
            byteLength += 4;
            i++;
          } else {
            byteLength += 3;
          }
        } else {
          byteLength += 3;
        }
      }
      return byteLength;
    }
    var CdrWriter = class _CdrWriter {
      get data() {
        return new Uint8Array(this.buffer, 0, this.offset);
      }
      get size() {
        return this.offset;
      }
      get kind() {
        return this.view.getUint8(1);
      }
      constructor(options = {}) {
        if (options.buffer != void 0) {
          this.buffer = options.buffer;
        } else if (options.size != void 0) {
          this.buffer = new ArrayBuffer(options.size);
        } else {
          this.buffer = new ArrayBuffer(_CdrWriter.DEFAULT_CAPACITY);
        }
        const kind = options.kind ?? EncapsulationKind_1.EncapsulationKind.CDR_LE;
        const { isCDR2, littleEndian } = (0, getEncapsulationKindInfo_1.getEncapsulationKindInfo)(kind);
        this.isCDR2 = isCDR2;
        this.littleEndian = littleEndian;
        this.hostLittleEndian = !(0, isBigEndian_1.isBigEndian)();
        this.eightByteAlignment = isCDR2 ? 4 : 8;
        this.array = new Uint8Array(this.buffer);
        this.view = new DataView(this.buffer);
        this.resizeIfNeeded(4);
        this.view.setUint8(0, 0);
        this.view.setUint8(1, kind);
        this.view.setUint16(2, 0, false);
        this.offset = 4;
        this.origin = 4;
      }
      int8(value) {
        this.resizeIfNeeded(1);
        this.view.setInt8(this.offset, value);
        this.offset += 1;
        return this;
      }
      uint8(value) {
        this.resizeIfNeeded(1);
        this.view.setUint8(this.offset, value);
        this.offset += 1;
        return this;
      }
      int16(value) {
        this.align(2);
        this.view.setInt16(this.offset, value, this.littleEndian);
        this.offset += 2;
        return this;
      }
      uint16(value) {
        this.align(2);
        this.view.setUint16(this.offset, value, this.littleEndian);
        this.offset += 2;
        return this;
      }
      int32(value) {
        this.align(4);
        this.view.setInt32(this.offset, value, this.littleEndian);
        this.offset += 4;
        return this;
      }
      uint32(value) {
        this.align(4);
        this.view.setUint32(this.offset, value, this.littleEndian);
        this.offset += 4;
        return this;
      }
      int64(value) {
        this.align(this.eightByteAlignment, 8);
        this.view.setBigInt64(this.offset, value, this.littleEndian);
        this.offset += 8;
        return this;
      }
      uint64(value) {
        this.align(this.eightByteAlignment, 8);
        this.view.setBigUint64(this.offset, value, this.littleEndian);
        this.offset += 8;
        return this;
      }
      uint16BE(value) {
        this.align(2);
        this.view.setUint16(this.offset, value, false);
        this.offset += 2;
        return this;
      }
      uint32BE(value) {
        this.align(4);
        this.view.setUint32(this.offset, value, false);
        this.offset += 4;
        return this;
      }
      uint64BE(value) {
        this.align(this.eightByteAlignment, 8);
        this.view.setBigUint64(this.offset, value, false);
        this.offset += 8;
        return this;
      }
      float32(value) {
        this.align(4);
        this.view.setFloat32(this.offset, value, this.littleEndian);
        this.offset += 4;
        return this;
      }
      float64(value) {
        this.align(this.eightByteAlignment, 8);
        this.view.setFloat64(this.offset, value, this.littleEndian);
        this.offset += 8;
        return this;
      }
      // writeLength optional because it could already be included in a header
      string(value, writeLength = true) {
        const strlen = stringLengthUtf8(value);
        if (writeLength) {
          this.uint32(strlen + 1);
        }
        this.resizeIfNeeded(strlen + 1);
        textEncoder.encodeInto(value, new Uint8Array(this.buffer, this.offset, strlen));
        this.view.setUint8(this.offset + strlen, 0);
        this.offset += strlen + 1;
        return this;
      }
      /** Writes the delimiter header using object size
       * NOTE: changing endian-ness with a single CDR message is not supported
       */
      dHeader(objectSize) {
        const header = objectSize;
        this.uint32(header);
        return this;
      }
      /**
       * Writes the member header (EMHEADER)
       * Accomodates for PL_CDR and PL_CDR2 based on the CdrWriter constructor options
       *
       * @param mustUnderstand - Whether the member is required to be understood by the receiver
       * @param id - The member ID
       * @param objectSize - The size of the member in bytes
       * @param lengthCode - Optional length code for CDR2 emHeaders.
       * lengthCode values [5-7] allow the emHeader object size to take the place of the normally encoded member length.
       *
       * NOTE: Dynamically determines default value if not provided that does not affect serialization ie will use lengthCode values [0-4].
       *
       * From Extensible and Dynamic Topic Types in DDS-XTypes v1.3 @ `7.4.3.4.2`:
       * "EMHEADER1 with LC values 5 to 7 also affect the serialization/deserialization virtual machine in that they cause NEXTINT to be
       * reused also as part of the serialized member. This is useful because the serialization of certain members also starts with an
       * integer length, which would take exactly the same value as NEXTINT. Therefore the use of length codes 5 to 7 saves 4 bytes in
       * the serialization."
       * @returns - CdrWriter instance
       */
      emHeader(mustUnderstand, id, objectSize, lengthCode) {
        return this.isCDR2 ? this.memberHeaderV2(mustUnderstand, id, objectSize, lengthCode) : this.memberHeaderV1(mustUnderstand, id, objectSize);
      }
      memberHeaderV1(mustUnderstand, id, objectSize) {
        this.align(4);
        const mustUnderstandFlag = mustUnderstand ? 1 << 14 : 0;
        const shouldUseExtendedPID = id > 16128 || objectSize > 65535;
        if (!shouldUseExtendedPID) {
          const idHeader = mustUnderstandFlag | id;
          this.uint16(idHeader);
          const objectSizeHeader = objectSize & 65535;
          this.uint16(objectSizeHeader);
        } else {
          const extendedHeader = mustUnderstandFlag | reservedPIDs_1.EXTENDED_PID;
          this.uint16(extendedHeader);
          this.uint16(8);
          this.uint32(id);
          this.uint32(objectSize);
        }
        this.resetOrigin();
        return this;
      }
      /** Sets the origin to the offset (DDS-XTypes Spec: `PUSH(ORIGIN = 0)`)*/
      resetOrigin() {
        this.origin = this.offset;
      }
      /** Writes boolean flag for optional members in CDR2
       * @throws Error if called for CDR1.
       */
      presentFlag(value) {
        if (!this.isCDR2) {
          throw new Error("presentFlag is only supported for CDR2");
        }
        this.uint8(value ? 1 : 0);
        return this;
      }
      /** Writes the PID_SENTINEL value if encapsulation supports it*/
      sentinelHeader() {
        if (!this.isCDR2) {
          this.align(4);
          this.uint16(reservedPIDs_1.SENTINEL_PID);
          this.uint16(0);
        }
        return this;
      }
      memberHeaderV2(mustUnderstand, id, objectSize, lengthCode) {
        if (id > 268435455) {
          throw Error(`Member ID ${id} is too large. Max value is ${268435455}`);
        }
        const mustUnderstandFlag = mustUnderstand ? 1 << 31 : 0;
        const finalLengthCode = lengthCode ?? (0, lengthCodes_1.getLengthCodeForObjectSize)(objectSize);
        const header = mustUnderstandFlag | finalLengthCode << 28 | id;
        this.uint32(header);
        switch (finalLengthCode) {
          case 0:
          case 1:
          case 2:
          case 3: {
            const shouldBeSize = lengthCodes_1.lengthCodeToObjectSizes[finalLengthCode];
            if (objectSize !== shouldBeSize) {
              throw new Error(`Cannot write a length code ${finalLengthCode} header with an object size not equal to ${shouldBeSize}`);
            }
            break;
          }
          // When the length code is > 3 the header is 8 bytes because of the NEXTINT value storing the object size
          case 4:
          case 5:
            this.uint32(objectSize);
            break;
          case 6:
            if (objectSize % 4 !== 0) {
              throw new Error("Cannot write a length code 6 header with an object size that is not a multiple of 4");
            }
            this.uint32(objectSize >> 2);
            break;
          case 7:
            if (objectSize % 8 !== 0) {
              throw new Error("Cannot write a length code 7 header with an object size that is not a multiple of 8");
            }
            this.uint32(objectSize >> 3);
            break;
          default:
            throw new Error(`Unexpected length code ${finalLengthCode}`);
        }
        return this;
      }
      sequenceLength(value) {
        return this.uint32(value);
      }
      int8Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        this.resizeIfNeeded(value.length);
        this.array.set(value, this.offset);
        this.offset += value.length;
        return this;
      }
      uint8Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        this.resizeIfNeeded(value.length);
        this.array.set(value, this.offset);
        this.offset += value.length;
        return this;
      }
      int16Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Int16Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.int16(entry);
          }
        }
        return this;
      }
      uint16Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Uint16Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.uint16(entry);
          }
        }
        return this;
      }
      int32Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Int32Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.int32(entry);
          }
        }
        return this;
      }
      uint32Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Uint32Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.uint32(entry);
          }
        }
        return this;
      }
      int64Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof BigInt64Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.int64(BigInt(entry));
          }
        }
        return this;
      }
      uint64Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof BigUint64Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.uint64(BigInt(entry));
          }
        }
        return this;
      }
      float32Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Float32Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.float32(entry);
          }
        }
        return this;
      }
      float64Array(value, writeLength) {
        if (writeLength === true) {
          this.sequenceLength(value.length);
        }
        if (value instanceof Float64Array && this.littleEndian === this.hostLittleEndian && value.length >= _CdrWriter.BUFFER_COPY_THRESHOLD) {
          this.align(value.BYTES_PER_ELEMENT, value.byteLength);
          this.array.set(new Uint8Array(value.buffer, value.byteOffset, value.byteLength), this.offset);
          this.offset += value.byteLength;
        } else {
          for (const entry of value) {
            this.float64(entry);
          }
        }
        return this;
      }
      /**
       * Calculate the capacity needed to hold the given number of aligned bytes,
       * resize if needed, and write padding bytes for alignment
       * @param size Byte width to align to. If the current offset is 1 and `size`
       *   is 4, 3 bytes of padding will be written
       * @param bytesToWrite Optional, total amount of bytes that are intended to be
       *   written directly following the alignment. This can be used to avoid
       *   additional buffer resizes in the case of writing large blocks of aligned
       *   data such as arrays
       */
      align(size, bytesToWrite = size) {
        const alignment = (this.offset - this.origin) % size;
        const padding = alignment > 0 ? size - alignment : 0;
        this.resizeIfNeeded(padding + bytesToWrite);
        this.array.fill(0, this.offset, this.offset + padding);
        this.offset += padding;
      }
      resizeIfNeeded(additionalBytes) {
        const capacity = this.offset + additionalBytes;
        if (this.buffer.byteLength < capacity) {
          const doubled = this.buffer.byteLength * 2;
          const newCapacity = doubled > capacity ? doubled : capacity;
          this.resize(newCapacity);
        }
      }
      resize(capacity) {
        if (this.buffer.byteLength >= capacity) {
          return;
        }
        const buffer = new ArrayBuffer(capacity);
        const array = new Uint8Array(buffer);
        array.set(this.array);
        this.buffer = buffer;
        this.array = array;
        this.view = new DataView(buffer);
      }
    };
    exports.CdrWriter = CdrWriter;
    CdrWriter.DEFAULT_CAPACITY = 16;
    CdrWriter.BUFFER_COPY_THRESHOLD = 10;
  }
});

// node_modules/@foxglove/cdr/dist/index.js
var require_dist2 = __commonJS({
  "node_modules/@foxglove/cdr/dist/index.js"(exports) {
    "use strict";
    var __createBinding = exports && exports.__createBinding || (Object.create ? (function(o, m, k, k2) {
      if (k2 === void 0) k2 = k;
      var desc = Object.getOwnPropertyDescriptor(m, k);
      if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
        desc = { enumerable: true, get: function() {
          return m[k];
        } };
      }
      Object.defineProperty(o, k2, desc);
    }) : (function(o, m, k, k2) {
      if (k2 === void 0) k2 = k;
      o[k2] = m[k];
    }));
    var __exportStar = exports && exports.__exportStar || function(m, exports2) {
      for (var p in m) if (p !== "default" && !Object.prototype.hasOwnProperty.call(exports2, p)) __createBinding(exports2, m, p);
    };
    Object.defineProperty(exports, "__esModule", { value: true });
    __exportStar(require_CdrReader(), exports);
    __exportStar(require_CdrSizeCalculator(), exports);
    __exportStar(require_CdrWriter(), exports);
    __exportStar(require_EncapsulationKind(), exports);
  }
});

// node_modules/@foxglove/rosmsg2-serialization/dist/messageDefinitionHasDataFields.js
var require_messageDefinitionHasDataFields = __commonJS({
  "node_modules/@foxglove/rosmsg2-serialization/dist/messageDefinitionHasDataFields.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.messageDefinitionHasDataFields = messageDefinitionHasDataFields;
    function messageDefinitionHasDataFields(fields) {
      return fields.some((field) => field.isConstant !== true);
    }
  }
});

// node_modules/@foxglove/rosmsg2-serialization/dist/MessageReader.js
var require_MessageReader = __commonJS({
  "node_modules/@foxglove/rosmsg2-serialization/dist/MessageReader.js"(exports) {
    "use strict";
    var __classPrivateFieldGet = exports && exports.__classPrivateFieldGet || function(receiver, state, kind, f) {
      if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a getter");
      if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot read private member from an object whose class did not declare it");
      return kind === "m" ? f : kind === "a" ? f.call(receiver) : f ? f.value : state.get(receiver);
    };
    var __classPrivateFieldSet = exports && exports.__classPrivateFieldSet || function(receiver, state, value, kind, f) {
      if (kind === "m") throw new TypeError("Private method is not writable");
      if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a setter");
      if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot write private member to an object whose class did not declare it");
      return kind === "a" ? f.call(receiver, value) : f ? f.value = value : state.set(receiver, value), value;
    };
    var _MessageReader_instances;
    var _MessageReader_rootDefinition;
    var _MessageReader_definitions;
    var _MessageReader_lastReadByteLength;
    var _MessageReader_lastReadHadTrailingBytes;
    var _MessageReader_useRos1Time;
    var _MessageReader_readComplexType;
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.MessageReader = void 0;
    var cdr_1 = require_dist2();
    var messageDefinitionHasDataFields_1 = require_messageDefinitionHasDataFields();
    var MessageReader2 = class {
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
        const rootDefinition = definitions.find((def) => !isConstantModule(def));
        if (rootDefinition == void 0) {
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
        __classPrivateFieldSet(this, _MessageReader_lastReadHadTrailingBytes, __classPrivateFieldGet(this, _MessageReader_lastReadByteLength, "f") < buffer.byteLength && !isCdrFinalPadding(buffer, __classPrivateFieldGet(this, _MessageReader_lastReadByteLength, "f")), "f");
        return value;
      }
    };
    exports.MessageReader = MessageReader2;
    _MessageReader_rootDefinition = /* @__PURE__ */ new WeakMap(), _MessageReader_definitions = /* @__PURE__ */ new WeakMap(), _MessageReader_lastReadByteLength = /* @__PURE__ */ new WeakMap(), _MessageReader_lastReadHadTrailingBytes = /* @__PURE__ */ new WeakMap(), _MessageReader_useRos1Time = /* @__PURE__ */ new WeakMap(), _MessageReader_instances = /* @__PURE__ */ new WeakSet(), _MessageReader_readComplexType = function _MessageReader_readComplexType2(definition, reader) {
      const msg = {};
      if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
        reader.uint8();
        return msg;
      }
      for (const field of definition) {
        if (field.isConstant === true) {
          continue;
        }
        if (field.isComplex === true) {
          const nestedDefinition = __classPrivateFieldGet(this, _MessageReader_definitions, "f").get(field.type);
          if (nestedDefinition == void 0) {
            throw new Error(`Unrecognized complex type ${field.type}`);
          }
          if (field.isArray === true) {
            const arrayLength = field.arrayLength ?? reader.sequenceLength();
            const array = [];
            for (let i = 0; i < arrayLength; i++) {
              array.push(__classPrivateFieldGet(this, _MessageReader_instances, "m", _MessageReader_readComplexType2).call(this, nestedDefinition, reader));
            }
            msg[field.name] = array;
          } else {
            msg[field.name] = __classPrivateFieldGet(this, _MessageReader_instances, "m", _MessageReader_readComplexType2).call(this, nestedDefinition, reader);
          }
        } else {
          if (field.isArray === true) {
            const deser = (__classPrivateFieldGet(this, _MessageReader_useRos1Time, "f") ? ros1TypedArrayDeserializers : typedArrayDeserializers).get(field.type);
            if (deser == void 0) {
              throw new Error(`Unrecognized primitive array type ${field.type}[]`);
            }
            const arrayLength = field.arrayLength ?? reader.sequenceLength();
            msg[field.name] = deser(reader, arrayLength);
          } else {
            const deser = (__classPrivateFieldGet(this, _MessageReader_useRos1Time, "f") ? ros1TimeDeserializers : deserializers).get(field.type);
            if (deser == void 0) {
              throw new Error(`Unrecognized primitive type ${field.type}`);
            }
            msg[field.name] = deser(reader);
          }
        }
      }
      return msg;
    };
    function isConstantModule(def) {
      return def.definitions.length > 0 && def.definitions.every((field) => field.isConstant);
    }
    function isCdrFinalPadding(buffer, decodedBytes) {
      const trailingByteLength = buffer.byteLength - decodedBytes;
      if (trailingByteLength <= 0) {
        return false;
      }
      const paddingToFourByteBoundary = (4 - decodedBytes % 4) % 4;
      const paddingToEightByteBoundary = (8 - decodedBytes % 8) % 8;
      if (trailingByteLength !== paddingToFourByteBoundary && trailingByteLength !== paddingToEightByteBoundary) {
        return false;
      }
      const trailingBytes = new Uint8Array(buffer.buffer, buffer.byteOffset + decodedBytes, trailingByteLength);
      return trailingBytes.every((byte) => byte === 0);
    }
    var deserializers = /* @__PURE__ */ new Map([
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
      ["duration", (reader) => ({ sec: reader.int32(), nanosec: reader.uint32() })]
    ]);
    var ros1TimeDeserializers = new Map([
      ...deserializers,
      ["time", (reader) => ({ sec: reader.int32(), nsec: reader.uint32() })],
      ["duration", (reader) => ({ sec: reader.int32(), nsec: reader.uint32() })]
    ]);
    var typedArrayDeserializers = /* @__PURE__ */ new Map([
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
      ["duration", readTimeArray]
    ]);
    var ros1TypedArrayDeserializers = new Map([
      ...typedArrayDeserializers,
      ["time", readRos1TimeArray],
      ["duration", readRos1TimeArray]
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
  }
});

// node_modules/@foxglove/rosmsg2-serialization/dist/stringLengthUtf8.js
var require_stringLengthUtf8 = __commonJS({
  "node_modules/@foxglove/rosmsg2-serialization/dist/stringLengthUtf8.js"(exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.stringLengthUtf8 = stringLengthUtf8;
    function stringLengthUtf8(str) {
      let byteLength = 0;
      const numCodeUnits = str.length;
      for (let i = 0; i < numCodeUnits; i++) {
        const codeUnit = str.charCodeAt(i);
        if (codeUnit <= 127) {
          byteLength += 1;
        } else if (codeUnit <= 2047) {
          byteLength += 2;
        } else if (55296 <= codeUnit && codeUnit <= 56319) {
          const nextCodeUnit = str.charCodeAt(i + 1);
          if (56320 <= nextCodeUnit && nextCodeUnit <= 57343) {
            byteLength += 4;
            i++;
          } else {
            byteLength += 3;
          }
        } else {
          byteLength += 3;
        }
      }
      return byteLength;
    }
  }
});

// node_modules/@foxglove/rosmsg2-serialization/dist/MessageWriter.js
var require_MessageWriter = __commonJS({
  "node_modules/@foxglove/rosmsg2-serialization/dist/MessageWriter.js"(exports) {
    "use strict";
    var __classPrivateFieldSet = exports && exports.__classPrivateFieldSet || function(receiver, state, value, kind, f) {
      if (kind === "m") throw new TypeError("Private method is not writable");
      if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a setter");
      if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot write private member to an object whose class did not declare it");
      return kind === "a" ? f.call(receiver, value) : f ? f.value = value : state.set(receiver, value), value;
    };
    var __classPrivateFieldGet = exports && exports.__classPrivateFieldGet || function(receiver, state, kind, f) {
      if (kind === "a" && !f) throw new TypeError("Private accessor was defined without a getter");
      if (typeof state === "function" ? receiver !== state || !f : !state.has(receiver)) throw new TypeError("Cannot read private member from an object whose class did not declare it");
      return kind === "m" ? f : kind === "a" ? f.call(receiver) : f ? f.value : state.get(receiver);
    };
    var _MessageWriter_instances;
    var _MessageWriter_rootDefinition;
    var _MessageWriter_definitions;
    var _MessageWriter_byteSize;
    var _MessageWriter_write;
    var _MessageWriter_getDefinition;
    var _MessageWriter_getPrimitiveSize;
    var _MessageWriter_getPrimitiveWriter;
    var _MessageWriter_getPrimitiveArrayWriter;
    Object.defineProperty(exports, "__esModule", { value: true });
    exports.MessageWriter = void 0;
    var cdr_1 = require_dist2();
    var messageDefinitionHasDataFields_1 = require_messageDefinitionHasDataFields();
    var stringLengthUtf8_1 = require_stringLengthUtf8();
    var PRIMITIVE_SIZES = /* @__PURE__ */ new Map([
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
      ["duration", 8]
    ]);
    var PRIMITIVE_WRITERS = /* @__PURE__ */ new Map([
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
      ["wstring", throwOnWstring]
    ]);
    var PRIMITIVE_ARRAY_WRITERS = /* @__PURE__ */ new Map([
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
      ["wstring", throwOnWstring]
    ]);
    function throwOnWstring() {
      throw new Error("wstring is implementation-defined and therefore not supported");
    }
    var MessageWriter2 = class {
      constructor(definitions) {
        _MessageWriter_instances.add(this);
        _MessageWriter_rootDefinition.set(this, void 0);
        _MessageWriter_definitions.set(this, void 0);
        const rootDefinition = definitions.find((def) => !isConstantModule(def));
        if (rootDefinition == void 0) {
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
          size: output ? void 0 : this.calculateByteSize(message)
        });
        __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write).call(this, __classPrivateFieldGet(this, _MessageWriter_rootDefinition, "f"), message, writer);
        return writer.data;
      }
    };
    exports.MessageWriter = MessageWriter2;
    _MessageWriter_rootDefinition = /* @__PURE__ */ new WeakMap(), _MessageWriter_definitions = /* @__PURE__ */ new WeakMap(), _MessageWriter_instances = /* @__PURE__ */ new WeakSet(), _MessageWriter_byteSize = function _MessageWriter_byteSize2(definition, message, offset) {
      const messageObj = message;
      let newOffset = offset;
      if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
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
          const dataArray = dataIsArray ? nestedMessage : [];
          if (field.arrayLength == void 0) {
            newOffset += padding(newOffset, 4);
            newOffset += 4;
          }
          if (field.isComplex === true) {
            const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
            for (let i = 0; i < arrayLength; i++) {
              const entry = dataArray[i] ?? {};
              newOffset = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_byteSize2).call(this, nestedDefinition, entry, newOffset);
            }
          } else if (field.type === "string") {
            for (let i = 0; i < arrayLength; i++) {
              const entry = dataArray[i] ?? "";
              newOffset += padding(newOffset, 4);
              newOffset += 4 + (0, stringLengthUtf8_1.stringLengthUtf8)(entry) + 1;
            }
          } else {
            const entrySize = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveSize).call(this, field.type);
            const alignment = field.type === "time" || field.type === "duration" ? 4 : entrySize;
            newOffset += padding(newOffset, alignment);
            newOffset += entrySize * arrayLength;
          }
        } else {
          if (field.isComplex === true) {
            const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
            const entry = nestedMessage ?? {};
            newOffset = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_byteSize2).call(this, nestedDefinition, entry, newOffset);
          } else if (field.type === "string") {
            const entry = typeof nestedMessage === "string" ? nestedMessage : "";
            newOffset += padding(newOffset, 4);
            newOffset += 4 + (0, stringLengthUtf8_1.stringLengthUtf8)(entry) + 1;
          } else {
            const entrySize = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveSize).call(this, field.type);
            const alignment = field.type === "time" || field.type === "duration" ? 4 : entrySize;
            newOffset += padding(newOffset, alignment);
            newOffset += entrySize;
          }
        }
      }
      return newOffset;
    }, _MessageWriter_write = function _MessageWriter_write2(definition, message, writer) {
      const messageObj = message;
      if (!(0, messageDefinitionHasDataFields_1.messageDefinitionHasDataFields)(definition)) {
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
          const dataArray = dataIsArray ? nestedMessage : [];
          if (field.arrayLength == void 0) {
            writer.sequenceLength(arrayLength);
          }
          if (field.arrayLength != void 0 && nestedMessage != void 0) {
            const givenFieldLength = fieldLength(nestedMessage);
            if (givenFieldLength !== field.arrayLength) {
              throw new Error(`Expected ${field.arrayLength} items for fixed-length array field ${field.name} but received ${givenFieldLength}`);
            }
          }
          if (field.isComplex === true) {
            const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
            for (let i = 0; i < arrayLength; i++) {
              const entry = dataArray[i] ?? {};
              __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write2).call(this, nestedDefinition, entry, writer);
            }
          } else {
            const arrayWriter = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveArrayWriter).call(this, field.type);
            arrayWriter(nestedMessage, field.defaultValue, writer, field.arrayLength);
          }
        } else {
          if (field.isComplex === true) {
            const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getDefinition).call(this, field.type);
            const entry = nestedMessage ?? {};
            __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_write2).call(this, nestedDefinition, entry, writer);
          } else {
            const primitiveWriter = __classPrivateFieldGet(this, _MessageWriter_instances, "m", _MessageWriter_getPrimitiveWriter).call(this, field.type);
            primitiveWriter(nestedMessage, field.defaultValue, writer);
          }
        }
      }
    }, _MessageWriter_getDefinition = function _MessageWriter_getDefinition2(datatype) {
      const nestedDefinition = __classPrivateFieldGet(this, _MessageWriter_definitions, "f").get(datatype);
      if (nestedDefinition == void 0) {
        throw new Error(`Unrecognized complex type ${datatype}`);
      }
      return nestedDefinition;
    }, _MessageWriter_getPrimitiveSize = function _MessageWriter_getPrimitiveSize2(primitiveType) {
      const size = PRIMITIVE_SIZES.get(primitiveType);
      if (size == void 0) {
        if (primitiveType === "wstring") {
          throwOnWstring();
        }
        throw new Error(`Unrecognized primitive type ${primitiveType}`);
      }
      return size;
    }, _MessageWriter_getPrimitiveWriter = function _MessageWriter_getPrimitiveWriter2(primitiveType) {
      const writer = PRIMITIVE_WRITERS.get(primitiveType);
      if (writer == void 0) {
        throw new Error(`Unrecognized primitive type ${primitiveType}`);
      }
      return writer;
    }, _MessageWriter_getPrimitiveArrayWriter = function _MessageWriter_getPrimitiveArrayWriter2(primitiveType) {
      const writer = PRIMITIVE_ARRAY_WRITERS.get(primitiveType);
      if (writer == void 0) {
        throw new Error(`Unrecognized primitive type ${primitiveType}[]`);
      }
      return writer;
    };
    function isConstantModule(def) {
      return def.definitions.length > 0 && def.definitions.every((field) => field.isConstant);
    }
    function fieldLength(value) {
      const length = value?.length;
      return typeof length === "number" ? length : 0;
    }
    function bool(value, defaultValue, writer) {
      const boolValue = typeof value === "boolean" ? value : defaultValue ?? false;
      writer.int8(boolValue ? 1 : 0);
    }
    function int8(value, defaultValue, writer) {
      writer.int8(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function uint8(value, defaultValue, writer) {
      writer.uint8(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function int16(value, defaultValue, writer) {
      writer.int16(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function uint16(value, defaultValue, writer) {
      writer.uint16(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function int32(value, defaultValue, writer) {
      writer.int32(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function uint32(value, defaultValue, writer) {
      writer.uint32(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function int64(value, defaultValue, writer) {
      if (typeof value === "bigint") {
        writer.int64(value);
      } else if (typeof value === "number") {
        writer.int64(BigInt(value));
      } else {
        writer.int64(defaultValue ?? 0n);
      }
    }
    function uint64(value, defaultValue, writer) {
      if (typeof value === "bigint") {
        writer.uint64(value);
      } else if (typeof value === "number") {
        writer.uint64(BigInt(value));
      } else {
        writer.uint64(defaultValue ?? 0n);
      }
    }
    function float32(value, defaultValue, writer) {
      writer.float32(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function float64(value, defaultValue, writer) {
      writer.float64(typeof value === "number" ? value : defaultValue ?? 0);
    }
    function string(value, defaultValue, writer) {
      writer.string(typeof value === "string" ? value : defaultValue ?? "");
    }
    function time(value, _defaultValue, writer) {
      if (value == void 0) {
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
      } else {
        writer.int8Array(defaultValue ?? new Int8Array(arrayLength ?? 0).fill(0));
      }
    }
    function int8Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Int8Array) {
        writer.int8Array(value);
      } else if (Array.isArray(value)) {
        const array = new Int8Array(value);
        writer.int8Array(array);
      } else {
        writer.int8Array(defaultValue ?? new Int8Array(arrayLength ?? 0).fill(0));
      }
    }
    function uint8Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Uint8Array) {
        writer.uint8Array(value);
      } else if (value instanceof Uint8ClampedArray) {
        writer.uint8Array(new Uint8Array(value));
      } else if (Array.isArray(value)) {
        const array = new Uint8Array(value);
        writer.uint8Array(array);
      } else {
        writer.uint8Array(defaultValue ?? new Uint8Array(arrayLength ?? 0).fill(0));
      }
    }
    function int16Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Int16Array) {
        writer.int16Array(value);
      } else if (Array.isArray(value)) {
        const array = new Int16Array(value);
        writer.int16Array(array);
      } else {
        writer.int16Array(defaultValue ?? new Int16Array(arrayLength ?? 0).fill(0));
      }
    }
    function uint16Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Uint16Array) {
        writer.uint16Array(value);
      } else if (Array.isArray(value)) {
        const array = new Uint16Array(value);
        writer.uint16Array(array);
      } else {
        writer.uint16Array(defaultValue ?? new Uint16Array(arrayLength ?? 0).fill(0));
      }
    }
    function int32Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Int32Array) {
        writer.int32Array(value);
      } else if (Array.isArray(value)) {
        const array = new Int32Array(value);
        writer.int32Array(array);
      } else {
        writer.int32Array(defaultValue ?? new Int32Array(arrayLength ?? 0).fill(0));
      }
    }
    function uint32Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Uint32Array) {
        writer.uint32Array(value);
      } else if (Array.isArray(value)) {
        const array = new Uint32Array(value);
        writer.uint32Array(array);
      } else {
        writer.uint32Array(defaultValue ?? new Uint32Array(arrayLength ?? 0).fill(0));
      }
    }
    function int64Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof BigInt64Array) {
        writer.int64Array(value);
      } else if (Array.isArray(value)) {
        const array = new BigInt64Array(value);
        writer.int64Array(array);
      } else {
        writer.int64Array(defaultValue ?? new BigInt64Array(arrayLength ?? 0).fill(0n));
      }
    }
    function uint64Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof BigUint64Array) {
        writer.uint64Array(value);
      } else if (Array.isArray(value)) {
        const array = new BigUint64Array(value);
        writer.uint64Array(array);
      } else {
        writer.uint64Array(defaultValue ?? new BigUint64Array(arrayLength ?? 0).fill(0n));
      }
    }
    function float32Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Float32Array) {
        writer.float32Array(value);
      } else if (Array.isArray(value)) {
        const array = new Float32Array(value);
        writer.float32Array(array);
      } else {
        writer.float32Array(defaultValue ?? new Float32Array(arrayLength ?? 0).fill(0));
      }
    }
    function float64Array(value, defaultValue, writer, arrayLength) {
      if (value instanceof Float64Array) {
        writer.float64Array(value);
      } else if (Array.isArray(value)) {
        const array = new Float64Array(value);
        writer.float64Array(array);
      } else {
        writer.float64Array(defaultValue ?? new Float64Array(arrayLength ?? 0).fill(0));
      }
    }
    function stringArray(value, defaultValue, writer, arrayLength) {
      if (Array.isArray(value)) {
        for (const item of value) {
          writer.string(typeof item === "string" ? item : "");
        }
      } else {
        const array = defaultValue ?? new Array(arrayLength ?? 0).fill("");
        for (const item of array) {
          writer.string(item);
        }
      }
    }
    function timeArray(value, _defaultValue, writer, arrayLength) {
      if (Array.isArray(value)) {
        for (const item of value) {
          time(item, void 0, writer);
        }
      } else {
        const array = new Array(arrayLength).fill(void 0);
        for (const item of array) {
          time(item, void 0, writer);
        }
      }
    }
    function padding(offset, byteWidth) {
      const alignment = (offset - 4) % byteWidth;
      return alignment > 0 ? byteWidth - alignment : 0;
    }
  }
});

// node_modules/@foxglove/rosmsg2-serialization/dist/index.js
var require_dist3 = __commonJS({
  "node_modules/@foxglove/rosmsg2-serialization/dist/index.js"(exports) {
    "use strict";
    var __createBinding = exports && exports.__createBinding || (Object.create ? (function(o, m, k, k2) {
      if (k2 === void 0) k2 = k;
      var desc = Object.getOwnPropertyDescriptor(m, k);
      if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
        desc = { enumerable: true, get: function() {
          return m[k];
        } };
      }
      Object.defineProperty(o, k2, desc);
    }) : (function(o, m, k, k2) {
      if (k2 === void 0) k2 = k;
      o[k2] = m[k];
    }));
    var __exportStar = exports && exports.__exportStar || function(m, exports2) {
      for (var p in m) if (p !== "default" && !Object.prototype.hasOwnProperty.call(exports2, p)) __createBinding(exports2, m, p);
    };
    Object.defineProperty(exports, "__esModule", { value: true });
    __exportStar(require_MessageReader(), exports);
    __exportStar(require_MessageWriter(), exports);
  }
});

// scripts/vendor_entry.mjs
var import_rosmsg = __toESM(require_dist(), 1);
var import_rosmsg2_serialization = __toESM(require_dist3(), 1);
var import_cdr = __toESM(require_dist2(), 1);
var export_CdrReader = import_cdr.CdrReader;
var export_MessageReader = import_rosmsg2_serialization.MessageReader;
var export_MessageWriter = import_rosmsg2_serialization.MessageWriter;
var export_parse = import_rosmsg.parse;
export {
  export_CdrReader as CdrReader,
  export_MessageReader as MessageReader,
  export_MessageWriter as MessageWriter,
  export_parse as parse
};
