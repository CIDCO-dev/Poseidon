/**
 * Tests for dataScript.js
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

// jQuery and DOM stubs
const elems = {};
function makeElem(id) {
	if (!elems[id]) {
		elems[id] = {
			id,
			innerHTML: '',
			textContent: '',
			disabled: false,
			children: [],
			classList: { add: jest.fn(), remove: jest.fn(), toggle: jest.fn(), toggleClass: jest.fn() },
			appendChild(node) { this.children.push(node); return node; },
			addEventListener: jest.fn()
		};
	}
	return elems[id];
}

const $ = (selector) => {
	if (selector.startsWith('#')) return makeElem(selector.slice(1));
	if (selector === document) return { on: jest.fn() };
	return {
		val: () => '-1',
		on: jest.fn(),
		append: jest.fn(),
		empty: jest.fn(),
		toggleClass: jest.fn(),
		text: jest.fn(),
		find: () => ({ remove: jest.fn() }),
		each: jest.fn()
	};
};
$.fn = {};

global.$ = $;
global.window = { location: { hostname: 'localhost' }, innerWidth: 1024 };
global.document = {
	getElementById: (id) => makeElem(id),
	addEventListener: jest.fn()
};
global.bootstrap = { Modal: function () { return { show: jest.fn(), hide: jest.fn() }; } };
global.WebSocket = function () { return { send: jest.fn() }; };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'dataScript.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'dataScript.js' });
	const context = vm.createContext(global);
	script.runInContext(context);
	return context;
}

describe('dataScript', () => {
	let ctx;

	beforeEach(() => {
		for (const k of Object.keys(elems)) delete elems[k];
		ctx = loadScript();
		makeElem('publishStatusList');
		makeElem('publishOkButton');
		makeElem('publishModal');
	});

	test('processState populates dataArray and table when files change', () => {
		const state = { fileslist: [['file1', '/a'], ['file2', '/b']] };
		ctx.processState(state);
		expect(ctx.dataArray.length).toBe(2);
		expect(ctx.dataArray[0]).toContain('file1');
	});

	test('startPublish clears list and disables button', () => {
		const list = makeElem('publishStatusList');
		list.innerHTML = 'old';
		const btn = makeElem('publishOkButton');
		ctx.startPublish();
		expect(list.innerHTML).toBe('');
		expect(btn.disabled).toBe(true);
	});

	test('updatePublishStatus appends messages', () => {
		const list = makeElem('publishStatusList');
		ctx.updatePublishStatus('hello');
		expect(list.children.length).toBe(1);
		expect(list.children[0].innerText).toBe('hello');
	});
});
