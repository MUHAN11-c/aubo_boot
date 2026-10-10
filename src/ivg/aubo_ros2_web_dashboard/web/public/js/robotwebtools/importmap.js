(map => {
  const mapUrl = document.currentScript.src;
  const resolve = imports =>
    Object.fromEntries(Object.entries(imports).map(([k, v]) => [k, new URL(v, mapUrl).href]));
  document.head.appendChild(
    Object.assign(document.createElement("script"), {
      type: "importmap",
      innerHTML: JSON.stringify({ imports: resolve(map.imports) })
    })
  );
})({
  "imports": {
    "roslib": "./build/roslib/RosLib.js",
    "ros3d": "./build/ros3d/ros3d.esm.js",
    "bson": "./roslib-esm/vendor/bson/lib/bson.mjs",
    "cbor2": "./roslib-esm/vendor/cbor2/lib/index.js",
    "eventemitter3": "./roslib-esm/vendor/eventemitter3/dist/eventemitter3.esm.js",
    "fast-png": "./roslib-esm/vendor/fast-png/lib/index.js",
    "uuid": "./roslib-esm/vendor/uuid/dist/index.js",
    "ws": "./roslib-esm/vendor/ws/browser.js",
    "@cto.af/wtf8": "./roslib-esm/vendor/@cto.af/wtf8/lib/index.js",
    "fflate": "./roslib-esm/vendor/fflate/esm/browser.js",
    "iobuffer": "./roslib-esm/vendor/iobuffer/lib/iobuffer.js"
  }
});
