/**
 * Tests for recordingScript.js
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { runInstrumented } = require('./helpers/coverage');

// jQuery stub
const elems = {};
function makeElem(id) {
	if (!elems[id]) {
		elems[id] = {
			classes: new Set(),
		textContent: '',
		addClass(cls) { this.classes.add(cls); return this; },
		removeClass(cls) { this.classes.delete(cls); return this; },
		hasClass(cls) { return this.classes.has(cls); },
		text(val) { if (val !== undefined) this.textContent = val; return this; },
		ready(fn) { if (fn) { fn(); } return this; }
	};
}
	return elems[id];
}

function $(selector) {
	if (typeof selector === 'string' && selector.startsWith('#')) return makeElem(selector.slice(1));
	return makeElem(selector);
}
$.fn = {};

global.$ = $;
global.window = { location: { hostname: 'localhost' } };

// WebSocket stub
const sent = [];
global.WebSocket = function () { return { send: (msg) => sent.push(msg) }; };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'recordingScript.js'), 'utf8');
	const sandbox = {
		...global,
		document: global.document,
		window: global.window,
		WebSocket: global.WebSocket,
		console
	};
	const context = vm.createContext(sandbox);
	runInstrumented(code, 'recordingScript.js', context);
	return context;
}

describe('recordingScript', () => {
	let ctx;

	beforeEach(() => {
		for (const k of Object.keys(elems)) delete elems[k];
		sent.length = 0;
		ctx = loadScript();
	});

	test('processRecordingInfo sets mode text and hides button for mode 1', () => {
		ctx.processRecordingInfo(true, "1");
		expect(makeElem('modeWidget').textContent).toBe('always ON');
		expect(makeElem('RecIcon').classes.has('d-none')).toBe(true);
	});

	test('toggleRecording sends start/stop based on RecIcon class', () => {
		// If text-success => startLogging
		makeElem('RecIcon').addClass('text-success');
		ctx.toggleRecording();
		expect(sent.pop()).toContain('startLogging');

		// If text-danger => stopLogging
		makeElem('RecIcon').removeClass('text-success').addClass('text-danger');
		ctx.toggleRecording();
		expect(sent.pop()).toContain('stopLogging');
	});
});
