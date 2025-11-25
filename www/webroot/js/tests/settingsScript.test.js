/**
 * Tests for settingsScript.js
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

// DOM stubs
const elements = {};
function makeElem(id) {
	if (!elements[id]) {
		elements[id] = {
			innerHTML: '',
			value: '',
			className: '',
			children: [],
			appendChild: (c) => elements[id].children.push(c)
		};
	}
	return elements[id];
}
const documentStub = {
	getElementById: (id) => makeElem(id),
	getElementsByClassName: (cls) => {
		return Object.values(elements).filter((el) => el.className === cls || (el.classList && el.classList.contains && el.classList.contains(cls)));
	}
};
global.document = documentStub;

// Minimal WebSocket stub
const sentMessages = [];
global.WebSocket = function () { return { send: (msg) => sentMessages.push(msg) }; };
global.window = { location: { hostname: 'localhost' } };

// Helpers to simulate config fields
function addConfigField(id, value, cls = 'configurationField') {
	const el = makeElem(id);
	el.id = id;
	el.value = value;
	el.className = cls;
	return el;
}

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'settingsScript.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'settingsScript.js' });
	const sandbox = {
		document: documentStub,
		window: global.window,
		WebSocket: global.WebSocket,
		console
	};
	const context = vm.createContext(sandbox);
	script.runInContext(context);
	return context;
}

describe('settingsScript', () => {
	let ctx;

	beforeEach(() => {
		for (const k of Object.keys(elements)) {
			delete elements[k];
		}
		sentMessages.length = 0;
		ctx = loadScript();
	});

	test('processConfig renders inputs and select for loggingMode', () => {
		const form = makeElem('formContent');
		const config = [
			{ key: 'apiServer', value: 'localhost' },
			{ key: 'loggingMode', value: '2' }
		];
		ctx.processConfig(config);
		expect(form.innerHTML).toMatch(/apiServer/);
		expect(form.innerHTML).toMatch(/loggingMode/);
		expect(form.innerHTML).toMatch(/manual/);
	});

	test('saveConfig builds payload from configurationField', () => {
		const form = makeElem('formContent');
		form.innerHTML = '';
		addConfigField('foo', 'bar');
		addConfigField('loggingMode', '1');

		ctx.saveConfig();

		expect(sentMessages.length).toBe(1);
		const parsed = JSON.parse(sentMessages[0]);
		expect(parsed.command).toBe('saveConfiguration');
		expect(parsed.configuration).toEqual(
			expect.arrayContaining([
				{ key: 'foo', value: 'bar' },
				{ key: 'loggingMode', value: '1' }
			])
		);
	});
});
