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
			value: '',
			children: [],
			classList: { add: jest.fn(), remove: jest.fn(), toggle: jest.fn(), toggleClass: jest.fn() },
			addClass(cls) { this.classList.add(cls); return this; },
			removeClass(cls) { this.classList.remove(cls); return this; },
			append: function (node) { this.children.push(node); return this; },
			appendChild(node) { this.children.push(node); return node; },
			find: () => ({ remove: jest.fn() }),
			text: function (v) { if (v !== undefined) { this.textContent = v; } return this; },
			addEventListener: jest.fn(),
			empty: jest.fn().mockReturnThis(),
			parent: function () { return this; },
			on: jest.fn().mockReturnThis(),
			click: jest.fn().mockReturnThis(),
			change: jest.fn().mockReturnThis(),
			toggleClass: jest.fn().mockReturnThis(),
			val(v) { if (v !== undefined) { this.value = v; return this; } return this.value || '-1'; }
		};
	}
	return elems[id];
}

const $ = (selector) => {
	if (typeof selector === 'string' && selector.startsWith('#')) return makeElem(selector.slice(1));
	if (typeof selector === 'string' && selector.startsWith('<')) {
		return {
			addClass: jest.fn().mockReturnThis(),
			insertBefore: jest.fn().mockReturnThis()
		};
	}
	if (selector === document) return { on: jest.fn() };
	return {
		val: () => '-1',
		on: jest.fn(),
		append: jest.fn(),
		empty: jest.fn(),
		toggleClass: jest.fn(),
		text: jest.fn(),
		find: () => ({ remove: jest.fn() }),
		each: jest.fn(),
		width: () => (global.window && global.window.innerWidth) || 0
	};
};
$.fn = {};
$.each = (arr, cb) => {
	if (Array.isArray(arr) && typeof cb === 'function') {
		arr.forEach((item, idx) => cb(idx, item));
	}
};

global.$ = $;
global.window = { location: { hostname: 'localhost' }, innerWidth: 1024 };
const documentStub = {
	getElementById: (id) => makeElem(id),
	addEventListener: jest.fn(),
	createElement: () => ({ className: '', innerText: '', children: [], appendChild: jest.fn() })
};
global.document = documentStub;
global.bootstrap = { Modal: function () { return { show: jest.fn(), hide: jest.fn() }; } };
global.WebSocket = function () { return { send: jest.fn() }; };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'dataScript.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'dataScript.js' });
	const sandbox = {
		document: documentStub,
		window: global.window,
		WebSocket: global.WebSocket,
		$: global.$,
		bootstrap: global.bootstrap,
		console,
		setTimeout,
		setInterval
	};
	const context = vm.createContext(sandbox);
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
