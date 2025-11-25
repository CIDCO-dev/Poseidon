/**
 * Tests for poseidon.js (currently minimal stub).
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'poseidon.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'poseidon.js' });
	const sandbox = {
		...global,
		document: global.document,
		window: global.window,
		WebSocket: global.WebSocket,
		console
	};
	const context = vm.createContext(sandbox);
	script.runInContext(context);
	return context;
}

describe('poseidon.js', () => {
	test('WebSocketInit exists', () => {
		const ctx = loadScript();
		expect(typeof ctx.WebSocketInit).toBe('function');
	});
});
