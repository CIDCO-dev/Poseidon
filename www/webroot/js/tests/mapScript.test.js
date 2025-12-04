/**
 * Tests for mapScript.js
 * Heavy dependencies (Leaflet, WKT) are stubbed to exercise core helpers.
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { runInstrumented } = require('./helpers/coverage');

// Leaflet stubs
const panToMock = jest.fn();
const fitBoundsMock = jest.fn();

class MapStub {
	constructor() {
		this._zoom = 10;
	}
	setView() { return this; }
	getZoom() { return this._zoom; }
	panTo(...args) { panToMock(...args); return this; }
	fitBounds(...args) { fitBoundsMock(...args); return this; }
}

const control = jest.fn(() => ({
	addTo: jest.fn(() => ({})),
	onAdd: null
}));
control.scale = jest.fn(() => ({ addTo: jest.fn(() => ({})) }));

const L = {
	map: () => new MapStub(),
	control,
	polyline: jest.fn(() => ({
		addTo: jest.fn(() => ({})),
		remove: jest.fn(),
		getLatLngs: jest.fn(() => [])
	})),
	hotline: jest.fn(() => ({
		addTo: jest.fn(() => ({})),
		remove: jest.fn()
	})),
	Shapefile: jest.fn(() => ({
		addTo: function () { return this; },
		once: jest.fn()
	})),
	DomUtil: {
		create: () => ({ innerHTML: '' })
	}
};

// WKT stub
let addToCalled = false;
const Wkt = {
	Wkt: function () {
		return {
			read: jest.fn(),
			toObject: () => ({
				addTo: () => { addToCalled = true; },
				getBounds: () => ({})
			})
		};
	}
};

global.L = L;
global.Wkt = Wkt;
global.window = { location: { hostname: 'localhost' } };
global.document = { getElementById: () => ({}) };
global.WebSocket = function () { return { send: jest.fn() }; };
global.console = console;

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'mapScript.js'), 'utf8');
	const sandbox = {
		...global,
		document: global.document,
		window: global.window,
		WebSocket: global.WebSocket,
		console
	};
	const context = vm.createContext(sandbox);
	runInstrumented(code, 'mapScript.js', context);
	return context;
}

describe('mapScript helpers', () => {
	beforeEach(() => {
		panToMock.mockClear();
		fitBoundsMock.mockClear();
		addToCalled = false;
	});

	test('processState pans map when telemetry present', () => {
		const ctx = loadScript();
		const msg = {
			telemetry: {
				vitals: [0, 0, 0, 0, 1],
				position: [10, 20],
				depth: [5]
			}
		};
		ctx.processState(msg);
		expect(panToMock).toHaveBeenCalled();
		expect(ctx.currentpolyline.length).toBeGreaterThan(0);
	});

	test('drawGeofence adds geometry to map', () => {
		const ctx = loadScript();
		ctx.drawGeofence('POLYGON((0 0,1 0,1 1,0 1,0 0))');
		expect(addToCalled).toBe(true);
		expect(fitBoundsMock).toHaveBeenCalled();
	});
});
