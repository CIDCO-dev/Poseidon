const vm = require('vm');
const { createInstrumenter } = require('istanbul-lib-instrument');

const instrumenter = createInstrumenter({ coverageVariable: '__coverage__' });

function runInstrumented(code, filename, context) {
  let instrumented = code;
  try {
    instrumented = instrumenter.instrumentSync(code, filename);
  } catch (e) {
    // If instrumentation fails, fall back to raw code.
    instrumented = code;
  }
  const script = new vm.Script(instrumented, { filename });
  script.runInContext(context);
  if (context.__coverage__) {
    global.__coverage__ = global.__coverage__ || {};
    Object.assign(global.__coverage__, context.__coverage__);
  }
}

module.exports = { runInstrumented };
