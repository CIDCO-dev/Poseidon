const fs = require('fs');
const path = require('path');
const vm = require('vm');

const JS_ROOT = path.join(__dirname, '..');

function collectJsFiles(dir) {
	const entries = fs.readdirSync(dir, { withFileTypes: true });
	let files = [];
	for (const entry of entries) {
		if (entry.name === 'tests') continue;
		const full = path.join(dir, entry.name);
		if (entry.isDirectory()) {
			files = files.concat(collectJsFiles(full));
		} else if (entry.isFile() && entry.name.endsWith('.js')) {
			files.push(full);
		}
	}
	return files;
}

describe('JavaScript sources compile', () => {
	const files = collectJsFiles(JS_ROOT);

	test.each(files)('compiles without syntax error: %s', (file) => {
		const code = fs.readFileSync(file, 'utf8');
		expect(() => new vm.Script(code, { filename: file })).not.toThrow();
	});
});
