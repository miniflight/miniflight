export function browserDatagrams(onPacket, ports = [14550, 5600]) {
  const NativeWebSocket = globalThis.WebSocket, sockets = new Map(), allowed = new Set(ports);
  class DatagramSocket extends EventTarget {
    static CONNECTING = 0; static OPEN = 1; static CLOSING = 2; static CLOSED = 3;
    CONNECTING = 0; OPEN = 1; CLOSING = 2; CLOSED = 3;
    readyState = 0; binaryType = 'arraybuffer'; bufferedAmount = 0;
    constructor(url, protocols) {
      super();
      const parsed = new URL(url);
      const local = parsed.hostname === 'localhost' || parsed.hostname === '127.0.0.1';
      const port = Number(parsed.port);
      if (!local || !allowed.has(port)) return new NativeWebSocket(url, protocols);
      this.url = String(url); this.port = port; this.protocol = 'binary'; this.extensions = '';
      if (!sockets.has(port)) sockets.set(port, new Set());
      sockets.get(port).add(this);
      queueMicrotask(() => { if (this.readyState === 0) { this.readyState = 1; this.emit('open', new Event('open')); } });
    }
    emit(name, event) { this.dispatchEvent(event); this['on' + name]?.(event); }
    send(data) {
      if (this.readyState !== 1) throw Error('Datagram channel is not open');
      const bytes = data instanceof Uint8Array ? data.slice() : new Uint8Array(data);
      if (bytes.length === 10 && bytes.slice(0, 8).every((v, i) => v === [255,255,255,255,112,111,114,116][i])) {
        this.sourcePort = bytes[8] * 256 + bytes[9]; return;
      }
      onPacket(bytes, this.port, this);
    }
    receive(bytes) {
      if (this.readyState !== 1) return;
      const data = bytes.slice().buffer;
      queueMicrotask(() => this.emit('message', new MessageEvent('message', {data})));
    }
    close(code = 1000, reason = '') {
      if (this.readyState === 3) return;
      this.readyState = 3; sockets.get(this.port)?.delete(this);
      this.emit('close', new CloseEvent('close', {code, reason, wasClean: true}));
    }
  }
  globalThis.WebSocket = DatagramSocket;
  return {
    send(bytes, port = 14550, peer = null) {
      const targets = peer ? [peer] : [...(sockets.get(port) ?? [])];
      if (!targets.length) throw Error('No simulator datagram peer');
      for (const socket of targets) socket.receive(bytes);
    },
    sockets,
    close() {
      for (const peers of sockets.values()) for (const peer of [...peers]) peer.close();
      if (globalThis.WebSocket === DatagramSocket) globalThis.WebSocket = NativeWebSocket;
    },
  };
}
