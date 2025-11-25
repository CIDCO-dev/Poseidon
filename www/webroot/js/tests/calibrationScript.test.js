/**
 * Tests for calibrationScript.js
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const sent = [];
global.WebSocket = function () { return { send: (msg) => sent.push(msg) }; };
global.window = { location: { hostname: 'localhost' } };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'calibrationScript.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'calibrationScript.js' });
	const context = vm.createContext(global);
	script.runInContext(context);
	return context;
}

describe('calibrationScript', () => {
	test('sendZeroImu sends correct command', () => {
		const ctx = loadScript();
		ctx.sendZeroImu();
		expect(sent.pop()).toBe(JSON.stringify({ command: 'zeroImu' }));
	});
});
