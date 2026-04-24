(map => {
  const mapUrl = document.currentScript.src;
  const resolve = imports => Object.fromEntries(Object.entries(imports).map(([k, v]) => [k, new URL(v, mapUrl).href]));
  document.head.appendChild(Object.assign(document.createElement("script"), {
    type: "importmap",
    innerHTML: JSON.stringify({ imports: resolve(map.imports) })
  }));
})({
  "imports": {
    "roslib": "./dist/RosLib.js",
    "bson": "./vendor/bson/lib/bson.mjs",
    "cbor2": "./vendor/cbor2/lib/index.js",
    "eventemitter3": "./vendor/eventemitter3/dist/eventemitter3.esm.js",
    "fast-png": "./vendor/fast-png/lib/index.js",
    "uuid": "./vendor/uuid/dist/index.js",
    "ws": "./vendor/ws/browser.js",
    "@cto.af/wtf8": "./vendor/@cto.af/wtf8/lib/index.js",
    "fflate": "./vendor/fflate/esm/browser.js",
    "iobuffer": "./vendor/iobuffer/lib/iobuffer.js"
  }
});
