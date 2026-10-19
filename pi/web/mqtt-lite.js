// mqtt-lite.js -- a minimal MQTT 3.1.1 client over WebSockets, for the Rider control page.
// Written here (2026-10-19) instead of vendoring MQTT.js: the page only needs connect, subscribe,
// QoS-0 publish, keep-alive and auto-reconnect, and this keeps the page free of third-party code
// and of any download. Spec: MQTT 3.1.1 (OASIS), sections 2-3.
//
// const c = new MqttLite("ws://" + location.host + "/", {clientId, onConnect, onClose, onMessage});
// c.subscribe("rider/status"); c.publish("rider/control/line", JSON.stringify({line: "en 0"}));
"use strict";

class MqttLite {
  constructor(url, opts = {}) {
    this.url = url;
    this.clientId = opts.clientId || ("rider-web-" + Math.random().toString(16).slice(2, 10));
    this.keepAlive = opts.keepAlive || 20;            // seconds
    this.onConnect = opts.onConnect || (() => {});
    this.onClose = opts.onClose || (() => {});
    this.onMessage = opts.onMessage || (() => {});
    this.subs = new Set();
    this.connected = false;
    this.nextId = 1;
    this.rx = new Uint8Array(0);                     // reassembly buffer (frames may split/merge packets)
    this._open();
  }

  // ---- encoding helpers ----
  static _utf8(s) { return new TextEncoder().encode(s); }
  static _str(s) {                                    // 2-byte length + UTF-8
    const b = MqttLite._utf8(s);
    return [b.length >> 8, b.length & 0xff, ...b];
  }
  static _remLen(n) {                                 // variable-length "remaining length"
    const out = [];
    do { let d = n % 128; n = Math.floor(n / 128); if (n > 0) d |= 0x80; out.push(d); } while (n > 0);
    return out;
  }
  _send(type, body) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(new Uint8Array([type, ...MqttLite._remLen(body.length), ...body]));
    return true;
  }

  // ---- connection ----
  _open() {
    this.ws = new WebSocket(this.url, ["mqtt"]);
    this.ws.binaryType = "arraybuffer";
    this.ws.onopen = () => {
      const body = [...MqttLite._str("MQTT"), 4, 0x02,          // protocol level 4 (3.1.1), clean session
                    this.keepAlive >> 8, this.keepAlive & 0xff, ...MqttLite._str(this.clientId)];
      this._send(0x10, body);                                    // CONNECT
    };
    this.ws.onmessage = (ev) => this._rx(new Uint8Array(ev.data));
    this.ws.onclose = () => this._closed();
    this.ws.onerror = () => {};                                  // onclose follows; reconnect there
  }
  _closed() {
    const was = this.connected;
    this.connected = false;
    clearInterval(this.pinger);
    this.rx = new Uint8Array(0);
    if (was) this.onClose();
    setTimeout(() => this._open(), 2000);                        // auto-reconnect
  }

  // ---- receive: split the byte stream into MQTT packets ----
  _rx(chunk) {
    const buf = new Uint8Array(this.rx.length + chunk.length);
    buf.set(this.rx); buf.set(chunk, this.rx.length);
    let p = 0;
    while (p + 2 <= buf.length) {
      let mult = 1, len = 0, i = p + 1, b;
      do {                                                       // decode remaining length
        if (i >= buf.length) { this.rx = buf.slice(p); return; } // header incomplete
        b = buf[i++]; len += (b & 0x7f) * mult; mult *= 128;
      } while (b & 0x80);
      if (i + len > buf.length) break;                           // body incomplete
      this._packet(buf[p], buf.subarray(i, i + len));
      p = i + len;
    }
    this.rx = buf.slice(p);
  }
  _packet(h, body) {
    const type = h >> 4;
    if (type === 2) {                                            // CONNACK
      if (body[1] !== 0) { this.ws.close(); return; }            // refused
      this.connected = true;
      for (const t of this.subs) this._subscribe(t);             // restore subscriptions
      this.pinger = setInterval(() => this._send(0xc0, []), this.keepAlive * 500);  // PINGREQ
      this.onConnect();
    } else if (type === 3) {                                     // PUBLISH
      const qos = (h >> 1) & 3, tl = (body[0] << 8) | body[1];
      const topic = new TextDecoder().decode(body.subarray(2, 2 + tl));
      const start = 2 + tl + (qos > 0 ? 2 : 0);                  // we subscribe at QoS 0; tolerate more
      const payload = new TextDecoder().decode(body.subarray(start));
      try { this.onMessage(topic, payload, (h & 1) === 1); } catch (e) { console.error(e); }
    }
    // SUBACK (9), PINGRESP (13): nothing to do
  }

  // ---- public API ----
  subscribe(topic) {
    this.subs.add(topic);
    if (this.connected) this._subscribe(topic);
  }
  _subscribe(topic) {
    const id = this.nextId++ & 0xffff || 1;
    this._send(0x82, [id >> 8, id & 0xff, ...MqttLite._str(topic), 0]);   // SUBSCRIBE, QoS 0
  }
  publish(topic, payload, retain = false) {
    if (!this.connected) return false;
    return this._send(0x30 | (retain ? 1 : 0), [...MqttLite._str(topic), ...MqttLite._utf8(payload)]);
  }
}
