/**
 * Tests for statusScript.js
 * Focus on processState behavior with stubs for jQuery elements.
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { runInstrumented } = require('./helpers/coverage');

// Minimal jQuery stub
const elements = {};
function makeElem(id) {
	if (!elements[id]) {
		elements[id] = {
			style: {},
			hidden: false,
			textContent: '',
			widthValue: '',
			classes: new Set(),
			parent: () => ({ hide: () => { elements[id].hidden = true; }, show: () => { elements[id].hidden = false; } }),
			hide() { this.hidden = true; return this; },
			show() { this.hidden = false; return this; },
			text(val) { if (val !== undefined) this.textContent = val; return this; },
			width(val) { if (val !== undefined) this.widthValue = val; return this; },
			addClass(cls) { this.classes.add(cls); return this; },
			removeClass(cls) { this.classes.delete(cls); return this; }
		};
	}
	return elements[id];
}

function $(selector) {
	if (typeof selector === 'string' && selector.startsWith('#')) {
		return makeElem(selector.slice(1));
	}
	return makeElem(selector);
}
$.fn = {};

global.$ = $;
global.window = { location: { hostname: 'localhost' } };
global.WebSocket = function () { return { onmessage: null, send: () => {} }; };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'statusScript.js'), 'utf8');
	const sandbox = {
		...global,
		document: global.document,
		window: global.window,
		WebSocket: global.WebSocket,
		console
	};
	const context = vm.createContext(sandbox);
	runInstrumented(code, 'statusScript.js', context);
	return context;
}

describe('statusScript processState', () => {
	let ctx;

	beforeEach(() => {
		for (const key of Object.keys(elements)) {
			delete elements[key];
		}
		ctx = loadScript();
	});

	test('handles missing telemetry gracefully', () => {
		expect(() => ctx.processState(null)).not.toThrow();
		expect(() => ctx.processState({})).not.toThrow();
	});

	test('updates uptime text', () => {
		const state = { telemetry: { vitals: [70, 10, 5, 10, 3661, 55, 12, 50] } };
		ctx.processState(state);
		expect(elements['uptimeText'].textContent).toBe('0 01:01:01');
	});

	test('hides humidity when sentinel value', () => {
		const state = { telemetry: { vitals: [70, 10, 5, 10, 0, 55, 12, -555] } };
		ctx.processState(state);
		expect(elements['humidity'].hidden).toBe(true);
		expect(elements['humidityText'].hidden).toBe(true);
	});

	test('shows humidity when valid', () => {
		const state = { telemetry: { vitals: [70, 10, 5, 10, 0, 55, 12, 45] } };
		ctx.processState(state);
		expect(elements['humidity'].hidden).toBe(false);
		expect(elements['humidityText'].textContent).toBe('45%');
	});
});
