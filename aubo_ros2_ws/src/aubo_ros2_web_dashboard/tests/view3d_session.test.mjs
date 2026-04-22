import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const sessionScript = fs.readFileSync(
	'/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/view3d/session.js',
	'utf8'
);
const topicsLabHtml = fs.readFileSync(
	'/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/topics_lab.html',
	'utf8'
);
const visionGraspHtml = fs.readFileSync(
	'/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/vision_grasp_panel.html',
	'utf8'
);

function buildSessionRuntime({ coarsePointer = false, devicePixelRatio = 1 } = {}) {
	const addedObjects = [];
	const viewerOptions = [];
	const pixelRatioCalls = [];

	class FakeVector3 {
		constructor(x = 0, y = 0, z = 0) {
			this.x = x;
			this.y = y;
			this.z = z;
		}

		clone() {
			return new FakeVector3(this.x, this.y, this.z);
		}
	}

	class FakeObject3D {
		constructor() {
			this.children = [];
			this.scaleValue = null;
			this.scale = {
				set: (x, y, z) => {
					this.scaleValue = [x, y, z];
				}
			};
		}

		updateMatrixWorld() {}

		getWorldPosition(target) {
			return target;
		}
	}

	class FakeViewer {
		constructor(opts) {
			this.opts = opts;
			viewerOptions.push(opts);
			this.scene = { remove() {} };
			this.camera = {
				position: new FakeVector3(),
				lookAt() {}
			};
			this.cameraControls = {
				center: null,
				update() {}
			};
			this.renderer = {
				setPixelRatio: value => pixelRatioCalls.push(value)
			};
		}

		addObject(obj) {
			addedObjects.push(obj);
		}

		stop() {}
	}

	class FakeAxes extends FakeObject3D {
		constructor(opts) {
			super();
			this.opts = opts;
		}
	}

	class FakeGrid extends FakeObject3D {
		constructor(opts) {
			super();
			this.opts = opts;
		}
	}

	const host = {
		clientWidth: 900,
		clientHeight: 600,
		appendChild(child) {
			this.child = child;
		},
		setAttribute() {},
		innerHTML: ''
	};

	const dom = {
		'view3d-host': host,
		'view3d-show-urdf': { checked: true },
		'view3d-show-axes': { checked: true },
		'view3d-show-grid': { checked: true },
		'pc-topic': { value: '' },
		'scan3-topic': { value: '' },
		'marker3-topic': { value: '' },
		'tf-fixed-frame': { value: 'base_link' },
		'view3d-use-topic-tf-only': { checked: false },
		'pc-max': { value: '32000' },
		'view3d-point-size': { value: '0.05' },
		'pc-throttle-ms': { value: '120' },
		'pc-msg-ratio': { value: '2' },
		'urdf-param': { value: '/robot_state_publisher:robot_description' }
	};

	const context = {
		console,
		location: { origin: 'http://localhost:8090' },
		document: {
			createElement() {
				return {
					id: '',
					appendChild() {},
					setAttribute() {}
				};
			}
		},
		matchMedia() {
			return { matches: coarsePointer };
		},
		devicePixelRatio,
		setTimeout() {
			return 1;
		},
		clearTimeout() {},
		setInterval() {
			return 1;
		},
		clearInterval() {},
		IVGView3DPatches: {
			installIvgThreeSafeAddPatch() {},
			ivgInstallMeshLoaderCasePatch() {},
			installIvgRos3dEmbeddedThreeSafeAddPatch() {},
			ivgRos3dEmbeddedObject3DClass() {
				return FakeObject3D;
			}
		},
		IVGView3DTf: {
			normalizeFrameId(frame) {
				return frame;
			},
			IvgRos3dTfClient: function IvgRos3dTfClient() {
				this.dispose = () => {};
			}
		},
		IVGView3DHints: {
			removeView3dPc2Hint() {},
			showView3dPc2Hint() {},
			removeView3dUrdfHint() {},
			showView3dUrdfHint() {}
		},
		IVGView3DPointCloud: {
			IvgRosPointCloudClient: function IvgRosPointCloudClient() {}
		},
		IVGView3DUrdf: {
			ivgAttachUrdfFromRosParam() {}
		},
		ROS3D: {
			Viewer: FakeViewer,
			Axes: FakeAxes,
			Grid: FakeGrid,
			Urdf: function Urdf() {},
			LaserScan: function LaserScan() {},
			MarkerArrayClient: function MarkerArrayClient() {},
			MarkerClient: function MarkerClient() {}
		},
		ROSLIB: {
			UrdfModel: function UrdfModel() {}
		},
		THREE: {
			Object3D: FakeObject3D,
			Box3: class Box3 {
				setFromObject() {
					return this;
				}

				isEmpty() {
					return true;
				}

				getCenter(target) {
					return target;
				}

				getBoundingSphere() {
					return { radius: 0.25 };
				}
			},
			Sphere: class Sphere {}
		}
	};
	context.window = context;
	context.globalThis = context;

	vm.runInNewContext(sessionScript, context, {
		filename: 'view3d/session.js'
	});

	const SessionCtor = context.IVGView3DSession.IvgRos3dView3dSession;
	const session = new SessionCtor({}, id => dom[id] || null, {});
	session.start();

	return {
		viewerOptions,
		pixelRatioCalls,
		axes: addedObjects.find(obj => obj instanceof FakeAxes),
		grid: addedObjects.find(obj => obj instanceof FakeGrid)
	};
}

test('desktop viewer uses sharper defaults and toned-down guides', () => {
	const runtime = buildSessionRuntime({ coarsePointer: false, devicePixelRatio: 2 });

	assert.equal(runtime.viewerOptions.length, 1);
	assert.equal(runtime.viewerOptions[0].antialias, true);
	assert.deepEqual(runtime.pixelRatioCalls, [1.5]);
	assert.deepEqual(runtime.axes.scaleValue, [0.35, 0.35, 0.35]);
	assert.equal(runtime.grid.opts.num_cells, 10);
	assert.equal(runtime.grid.opts.cellSize, 1);
	assert.equal(runtime.grid.opts.color, '#cbd5e1');
});

test('coarse-pointer devices keep conservative pixel ratio protection', () => {
	const runtime = buildSessionRuntime({ coarsePointer: true, devicePixelRatio: 2 });

	assert.equal(runtime.viewerOptions[0].antialias, false);
	assert.deepEqual(runtime.pixelRatioCalls, [1]);
});

test('3d panels default to axes-only by leaving grid unchecked', () => {
	assert.match(topicsLabHtml, /id="view3d-show-grid"(?![^>]*checked)/);
	assert.match(visionGraspHtml, /id="view3d-show-grid"(?![^>]*checked)/);
});
