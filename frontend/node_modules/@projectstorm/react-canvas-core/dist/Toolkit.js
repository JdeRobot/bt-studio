export class Toolkit {
    /**
     * Generats a unique ID (thanks Stack overflow :3)
     * @returns {String}
     */
    static UID() {
        if (Toolkit.TESTING) {
            Toolkit.TESTING_UID++;
            return `${Toolkit.TESTING_UID}`;
        }
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
            const r = (Math.random() * 16) | 0;
            const v = c === 'x' ? r : (r & 0x3) | 0x8;
            return v.toString(16);
        });
    }
    static closest(element, selector) {
        if (!Element.prototype.closest) {
            Element.prototype.closest = function (s) {
                var el = this;
                do {
                    if (Element.prototype.matches.call(el, s))
                        return el;
                    el = el.parentElement || el.parentNode;
                } while (el !== null && el.nodeType === 1);
                return null;
            };
        }
        return element.closest(selector);
    }
}
Toolkit.TESTING = false;
Toolkit.TESTING_UID = 0;
//# sourceMappingURL=Toolkit.js.map