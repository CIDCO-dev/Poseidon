/**
 * Jest setup: provide minimal globals (window, document via jsdom), jQuery, WebSocket stubs.
 */

global.window = global.window || { location: { hostname: 'localhost' }, innerWidth: 1024 };
global.document = global.document || {};

const sentMessages = [];
global.__sentMessages = sentMessages;

global.WebSocket = function () {
	return {
		send: (msg) => sentMessages.push(msg),
		onmessage: null,
		onopen: null
	};
};

function createElem() {
	return {
		classes: new Set(),
		style: {},
		textContent: '',
		innerHTML: '',
		widthValue: 0,
		hidden: false,
		addClass(cls) { this.classes.add(cls); return this; },
		removeClass(cls) { this.classes.delete(cls); return this; },
		hasClass(cls) { return this.classes.has(cls); },
		css: function (k, v) { this.style[k] = v; return this; },
		on: function () { return this; },
		width: function (v) { if (v !== undefined) { this.widthValue = v; return this; } return this.widthValue; },
		find: function () { return { attr: () => '/', text: () => this, }; },
		text: function (v) { if (v !== undefined) { this.textContent = v; } return this; },
		val: function () { return this.value || ''; },
		append: function () { return this; },
		empty: function () { return this; },
		toggleClass: function () { return this; },
		parent: function () { return this; },
		hide: function () { this.hidden = true; return this; },
		show: function () { this.hidden = false; return this; }
	};
}

const elemCache = {};
global.$ = function (selector) {
	if (selector === global.document || selector === document) {
		return { on: () => {}, find: () => ({ attr: () => '/' }) };
	}
	if (typeof selector === 'string' && selector.startsWith('#')) {
		const id = selector.slice(1);
		elemCache[id] = elemCache[id] || createElem();
		return elemCache[id];
	}
	return createElem();
};
global.jQuery = global.$;
