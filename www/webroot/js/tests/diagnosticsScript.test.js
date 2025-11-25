/**
 * Tests for diagnosticsScript.js
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const elements = {};
function makeElem(id, tag = 'div') {
	if (!elements[id]) {
		elements[id] = {
			id,
			tag,
			classList: {
				add: jest.fn(),
				remove: jest.fn()
			},
			textContent: '',
			innerHTML: '',
			children: [],
			appendChild(node) { this.children.push(node); return node; },
			remove: jest.fn(),
			rows: [],
			insertRow(index) {
				const row = {
					cells: [],
					insertCell: function () {
						const cell = { innerHTML: '', textContent: '', style: {} };
						this.cells.push(cell);
						return cell;
					}
				};
				this.rows[index] = row;
				return row;
			},
			deleteRow(i) { this.rows.splice(i, 1); }
		};
	}
	return elements[id];
}

global.document = {
	getElementById: (id) => makeElem(id),
	createElement: (tag) => ({ tag, classList: { add: jest.fn() }, appendChild: jest.fn(), insertRow: makeElem('tmp').insertRow })
};
global.window = { location: { hostname: 'localhost' } };
global.WebSocket = function () { return { send: jest.fn() }; };

function loadScript() {
	const code = fs.readFileSync(path.join(__dirname, '..', 'diagnosticsScript.js'), 'utf8');
	const script = new vm.Script(code, { filename: 'diagnosticsScript.js' });
	const context = vm.createContext(global);
	script.runInContext(context);
	return context;
}

describe('diagnosticsScript', () => {
	let ctx;

	beforeEach(() => {
		for (const k of Object.keys(elements)) delete elements[k];
		ctx = loadScript();
		// Prepare tables/spinner/buttons
		makeElem('diagnosticsTable').rows = [{}, {}];
		makeElem('runningNodes');
		makeElem('loading-spinner');
		makeElem('diagnosticsButton');
	});

	test('processDiagnostics populates table', () => {
		const data = [
			{ status: true, name: 'node1', message: 'ok' },
			{ status: false, name: 'node2', message: 'bad' }
		];
		ctx.processDiagnostics(data);
		const table = makeElem('diagnosticsTable');
		expect(table.rows.length).toBeGreaterThan(2);
		expect(table.rows[1].cells[0].innerHTML).toContain('✅');
	});

	test('processRunningNodes builds running nodes table', () => {
		ctx.processRunningNodes(['a', 'b']);
		const container = makeElem('runningNodes');
		expect(container.children.length).toBeGreaterThan(0);
	});
});
