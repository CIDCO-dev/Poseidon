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
		css: function (k, v) { this.style[k] = v; return this; }
	});
}

const $ = (selector) => {
	if (selector.startsWith('#')) return makeElem(selector.slice(1));
	if (selector.startsWith('.')) return makeElem(selector.slice(1));
	return makeElem(selector);
};
$.fn = {};

global.$ = $;
global.jQuery = $;

// Window stub
global.window = {
	location: { pathname: '/home' },
	innerWidth: 1024
};
global.document = {};

function runScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'script.js'), 'utf8');
	const script = new vm.Script(code);
	const context = vm.createContext(global);
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
		const liElem = makeElem('navbar-nav');
		liElem.find = () => ({ attr: () => '/home' });
		// Trigger script
		runScript();
		expect(liElem.classes.has('active')).toBe(true);
	});

	test('home icon visible on narrow screens', () => {
		global.window.innerWidth = 500;
		runScript();
		expect(makeElem('homeIcon').style.display).toBe('block');
	});
});
