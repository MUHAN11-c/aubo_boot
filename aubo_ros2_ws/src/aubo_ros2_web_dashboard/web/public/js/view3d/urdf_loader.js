/**
 * 3D URDF 加载层：
 * - 兼容 `ROSLIB.Param.get()` 与 `/rosapi/get_param` 两条取参路径。
 * - 负责把取到的 URDF XML 变成 `ROSLIB.UrdfModel + ROS3D.Urdf`。
 * - 不负责 viewer 生命周期；由 session 统一安排加载时机与错误提示。
 */
(function (global) {
	'use strict';

	const hints = global.IVGView3DHints;
	if (!hints) throw new Error('IVGView3D hints 未加载');

	const showView3dUrdfHint = hints.showView3dUrdfHint;
	const removeView3dUrdfHint = hints.removeView3dUrdfHint;

	function ivgDecodeRosapiParamString(raw) {
		if (raw == null || raw === '') return '';
		if (typeof raw === 'string') {
			try {
				const v = JSON.parse(raw);
				return typeof v === 'string' ? v : '';
			} catch (e0) {
				return raw;
			}
		}
		return '';
	}

	function ivgExtractRosapiParamValue(resp) {
		if (!resp || typeof resp !== 'object') return '';
		if (Object.prototype.hasOwnProperty.call(resp, 'value')) return resp.value;
		if (resp.values && Object.prototype.hasOwnProperty.call(resp.values, 'value')) return resp.values.value;
		return '';
	}

	function ivgBuildParamNameCandidates(paramFullName) {
		const raw = String(paramFullName || '').trim();
		const out = [];
		function push(v) {
			const name = String(v || '').trim();
			if (!name || out.indexOf(name) !== -1) return;
			out.push(name);
		}
		push(raw);
		push(raw.replace(/^\/+/, ''));
		if (raw.indexOf(':') !== -1) {
			const idx = raw.indexOf(':');
			const node = raw.slice(0, idx).trim().replace(/^\/+/, '');
			const param = raw.slice(idx + 1).trim().replace(/^\/+/, '');
			push(param);
			if (node && param) {
				push(`${node}/${param}`);
				push(`/${node}/${param}`);
			}
		}
		if (raw.indexOf('/') !== -1) push(raw.slice(raw.lastIndexOf('/') + 1));
		return out;
	}

	function ivgRoslibParamValueToUrdfXml(val) {
		if (val == null) return '';
		if (typeof val === 'string') {
			const t = val.trim();
			if (t.indexOf('<robot') !== -1 || t.indexOf('<?xml') !== -1 || t.indexOf('<urdf') !== -1) return t;
			return ivgDecodeRosapiParamString(t).trim();
		}
		if (typeof val === 'object' && val !== null && typeof val.data === 'string') {
			return ivgRoslibParamValueToUrdfXml(val.data);
		}
		return String(val);
	}

	function ivgAttachUrdfFromRosParam(ros, paramFullName, meshBase, tfClient, rootGroup, $, onErr, hintHostEl) {
		function reportErr(msg) {
			onErr(msg);
		}
		if (!ros) {
			reportErr('ROSLIB.Ros 不可用');
			return;
		}
		const serviceTypes = ['rosapi/GetParam', 'rosapi/srv/GetParam'];
		const paramNames = ivgBuildParamNameCandidates(paramFullName);
		function attachFromXml(xml) {
			const looks = xml.indexOf('<robot') !== -1 || xml.indexOf('<?xml') !== -1 || xml.indexOf('<urdf') !== -1;
			if (!looks) {
				reportErr(`返回值不像 URDF XML（开头）：${JSON.stringify(xml.slice(0, 96))}`);
				return;
			}
			try {
				const xmlForModel = xml.replace(/\.(DAE|STL|OBJ)(?=["'\s>])/gi, function (m) {
					return m.toLowerCase();
				});
				const urdfModel = new ROSLIB.UrdfModel({ string: xmlForModel });
				const urdfViz = new ROS3D.Urdf({
					urdfModel,
					path: meshBase,
					tfClient,
					tfPrefix: ''
				});
				rootGroup.add(urdfViz);
				const h = hintHostEl || ($ && $('view3d-host'));
				if (h) {
					showView3dUrdfHint(
						h,
						'<strong>机械臂</strong>：URDF 已解析，网格异步加载中；若仍不可见请看 Network 是否对 <code>/api/ivg/robot-mesh/…</code> 404。'
					);
					setTimeout(() => removeView3dUrdfHint(h), 10000);
				}
			} catch (e2) {
				reportErr(e2 && e2.message ? e2.message : String(e2));
			}
		}

		function tryRoslibParam(nameIdx) {
			if (typeof ROSLIB.Param !== 'function') {
				tryServiceParam(0, 0, '');
				return;
			}
			if (nameIdx >= paramNames.length) {
				tryServiceParam(0, 0, '');
				return;
			}
			const p = new ROSLIB.Param({ ros, name: paramNames[nameIdx] });
			p.get(
				val => {
					const xml = ivgRoslibParamValueToUrdfXml(val);
					const looks = xml.indexOf('<robot') !== -1 || xml.indexOf('<?xml') !== -1 || xml.indexOf('<urdf') !== -1;
					if (!xml || !looks) {
						tryRoslibParam(nameIdx + 1);
						return;
					}
					attachFromXml(xml);
				},
				() => {
					tryRoslibParam(nameIdx + 1);
				}
			);
		}

		function tryServiceParam(typeIdx, nameIdx, lastErr) {
			if (typeof ROSLIB.Service !== 'function') {
				reportErr('ROSLIB.Service 不可用，无法 get_param');
				return;
			}
			if (typeIdx >= serviceTypes.length) {
				reportErr(lastErr || 'get_param 返回空或无法解析为字符串（请核对「URDF 参数」全名）');
				return;
			}
			if (nameIdx >= paramNames.length) {
				tryServiceParam(typeIdx + 1, 0, lastErr);
				return;
			}
			const svc = new ROSLIB.Service({
				ros,
				name: '/rosapi/get_param',
				serviceType: serviceTypes[typeIdx]
			});
			svc.callService(
				{ name: paramNames[nameIdx], default_value: '' },
				resp => {
					const rawVal = ivgExtractRosapiParamValue(resp);
					const xml = ivgDecodeRosapiParamString(rawVal).trim();
					if (!xml) {
						tryServiceParam(typeIdx, nameIdx + 1, `get_param 返回空或无法解析为字符串（已尝试：${paramNames.join(' / ')}）`);
						return;
					}
					attachFromXml(xml);
				},
				err => {
					const msg = err && err.message ? err.message : String(err);
					tryServiceParam(typeIdx, nameIdx + 1, msg);
				}
			);
		}

		tryRoslibParam(0);
	}

	global.IVGView3DUrdf = {
		ivgDecodeRosapiParamString,
		ivgExtractRosapiParamValue,
		ivgBuildParamNameCandidates,
		ivgRoslibParamValueToUrdfXml,
		ivgAttachUrdfFromRosParam
	};
})(typeof window !== 'undefined' ? window : globalThis);
