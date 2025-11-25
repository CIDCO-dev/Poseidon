/**
 * Unit tests for dashboardscript.js helpers.
 * We stub globals (WebSocket, Chart, RadialGauge, DOM) to isolate pure logic.
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

// Minimal DOM stubs
const elements = {
	overlay: { style: { display: 'none' } },
	'overlay-text': { innerHTML: '' },
	attitudeChart: { getContext: () => ({}) },
	depthChart: { getContext: () => ({}) },
	imuz: {},
	gnssStatus: { classList: { remove: jest.fn(), add: jest.fn() } },
	gnssContext: {},
	gpsSatChart: { getContext: () => ({}) },
	imuGauge: {},
	speedGauge: {},
	depthGauge: {}
};

const documentStub = {
	getElementById: (id) => elements[id] || null
};
global.document = documentStub;
global.window = { location: { hostname: 'localhost' } };

// Minimal chart/gauge stubs
global.Chart = function () { return { update: jest.fn() }; };
global.RadialGauge = function () { return { draw: () => ({}) , value: 0}; };

// WebSocket stub
global.WebSocket = function () { return { close: jest.fn() }; };

// jQuery-like stub for class manipulation
global.$ = () => ({
	removeClass: jest.fn().mockReturnThis(),
	addClass: jest.fn().mockReturnThis(),
	text: jest.fn().mockReturnThis(),
	width: jest.fn(() => (global.window && global.window.innerWidth) || 1024),
	on: jest.fn().mockReturnThis(),
	toggleClass: jest.fn().mockReturnThis(),
	ready: function (fn) { if (fn) { fn(); } return this; },
	parent: jest.fn(() => ({
		toggleClass: jest.fn().mockReturnThis(),
		parent: jest.fn(() => ({ toggleClass: jest.fn().mockReturnThis() })),
		css: jest.fn().mockReturnThis()
	})),
	css: jest.fn().mockReturnThis()
});

function loadScript(filename) {
	const code = fs.readFileSync(filename, 'utf8');
	const script = new vm.Script(code, { filename });
	const sandbox = {
		document: documentStub,
		window: global.window || {},
		WebSocket: global.WebSocket,
		Chart: global.Chart,
		RadialGauge: global.RadialGauge,
		$: global.$,
		console,
		setTimeout
	};
	const context = vm.createContext(sandbox);
	script.runInContext(context);
	return context;
}

describe('dashboardscript helpers', () => {
	const scriptPath = path.join(__dirname, '..', 'dashboardscript.js');
	let context;

	beforeAll(() => {
		context = loadScript(scriptPath);
	});

	test('getFixTypeLabel returns known labels and fallback', () => {
		expect(context.getFixTypeLabel(3)).toBe('3D Fix');
		expect(context.getFixTypeLabel(99)).toBe('Unknown (99)');
	});

	test('displayOverlay updates DOM visibility', () => {
		context.displayOverlay();
		expect(elements.overlay.style.display).toBe('block');
		expect(elements['overlay-text'].innerHTML.length).toBeGreaterThan(0);
	});

	test('hideOverlay updates DOM visibility', () => {
		elements.overlay.style.display = 'block';
		context.hideOverlay();
		expect(elements.overlay.style.display).toBe('none');
	});
});
