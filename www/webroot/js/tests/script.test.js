/**
 * Tests for script.js (sidebar toggle and active link highlighting).
 * DOM and jQuery are stubbed to keep it self-contained.
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

// Minimal jQuery stub
const stubbedElems = {};
function makeElem(id) {
	return stubbedElems[id] || (stubbedElems[id] = {
		classes: new Set(),
		style: {},
		addClass(cls) { this.classes.add(cls); return this; },
		removeClass(cls) { this.classes.delete(cls); return this; },
		hasClass(cls) { return this.classes.has(cls); },
		on: jest.fn().mockReturnThis(),
		find: () => ({ attr: () => '/' }),
		attr: () => '/',
		css: function (k, v) { this.style[k] = v; return this; },
		width: function () { return global.window ? global.window.innerWidth : 0; },
		each: function (fn) { if (fn) { fn.call(this); } return this; }
	});
}

// Window stub
const windowStub = {
	location: { pathname: '/home' },
	innerWidth: 1024
};
global.window = windowStub;
global.document = {};

const $ = (selector) => {
	if (selector === windowStub) {
		return {
			width: () => (windowStub ? windowStub.innerWidth : 0),
			on: jest.fn().mockReturnThis()
		};
	}
	if (selector && typeof selector === 'object') return selector;
	if (typeof selector === 'string' && selector.startsWith('#')) return makeElem(selector.slice(1));
	if (typeof selector === 'string' && selector.startsWith('.')) return makeElem(selector.slice(1));
	return makeElem(selector);
};
$.fn = {};

global.$ = $;
global.jQuery = $;

function runScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'script.js'), 'utf8');
	const script = new vm.Script(code);
	const sandbox = {
		document: global.document,
		window: windowStub,
		WebSocket: global.WebSocket,
		$: global.$,
		jQuery: global.jQuery,
		console
	};
	const context = vm.createContext(sandbox);
	script.runInContext(context);
	return context;
}

describe('script.js behaviors', () => {
	beforeEach(() => {
		for (const key of Object.keys(stubbedElems)) {
			delete stubbedElems[key];
		}
	});

	test('active page highlighting sets active on matching link', () => {
		// Prepare navbar item with href
		const liElem = makeElem('navbar-nav li');
		liElem.find = () => ({ attr: () => '/home' });
		liElem.each = (fn) => { if (fn) { fn.call(liElem); } return liElem; };
		liElem.addClass = (cls) => { liElem.classes.add(cls); return liElem; };
		windowStub.location.pathname = '/home';
		// Trigger script
		runScript();
		expect(liElem.classes.has('active')).toBe(true);
	});

	test('home icon visible on narrow screens', () => {
		windowStub.innerWidth = 500;
		runScript();
		expect(makeElem('homeIcon').style.display).toBe('block');
	});
});
