import {schema} from './mavlink-schema.mjs';

const methods = {
  char: ['Uint8', 1], uint8_t: ['Uint8', 1], uint8_t_mavlink_version: ['Uint8', 1],
  int8_t: ['Int8', 1], uint16_t: ['Uint16', 2], int16_t: ['Int16', 2],
  uint32_t: ['Uint32', 4], int32_t: ['Int32', 4],
  uint64_t: ['BigUint64', 8], int64_t: ['BigInt64', 8],
  float: ['Float32', 4], double: ['Float64', 8],
};
const names = new Map(Object.entries(schema).map(([id, info]) => [info.name, Number(id)]));

export function checksum(bytes, extra) {
  let crc = 0xffff;
  for (const byte of [...bytes, extra]) {
    let t = byte ^ (crc & 255);
    t ^= (t << 4) & 255;
    crc = ((crc >>> 8) ^ (t << 8) ^ (t << 3) ^ (t >>> 4)) & 65535;
  }
  return crc;
}

export class Mavlink {
  constructor(write = () => {}, system = 255, component = 191) {
    this.write = write;
    this.system = system;
    this.component = component;
    this.sequence = 0;
    this.pending = new Uint8Array();
  }

  send(name, values = {}) {
    const id = names.get(name), info = schema[id];
    if (!info) throw Error('Unknown MAVLink message: ' + name);
    const payload = new Uint8Array(info.length), view = new DataView(payload.buffer);
    for (const [field, type, offset, count] of info.fields) {
      const [method, size] = methods[type];
      const value = values[field] ?? (count > 1 ? [] : 0);
      const bytes = type === 'char' && typeof value === 'string' ? new TextEncoder().encode(value) : value;
      for (let i = 0; i < count; i++) {
        let item = (count === 1 ? bytes : bytes[i]) ?? 0;
        if (size === 8 && type.endsWith('64_t')) item = BigInt(item);
        view['set' + method](offset + i * size, item, true);
      }
    }
    let length = payload.length;
    while (length > 1 && payload[length - 1] === 0) length--;
    const packet = new Uint8Array(12 + length);
    packet.set([253, length, 0, 0, this.sequence++ & 255, this.system, this.component,
      id & 255, (id >>> 8) & 255, id >>> 16]);
    packet.set(payload.subarray(0, length), 10);
    new DataView(packet.buffer).setUint16(10 + length, checksum(packet.subarray(1, 10 + length), info.crc), true);
    this.write(packet);
    return packet;
  }

  receive(bytes) {
    const buffer = new Uint8Array(this.pending.length + bytes.length);
    buffer.set(this.pending); buffer.set(bytes, this.pending.length);
    const messages = [];
    let at = 0;
    while (at < buffer.length) {
      const version = buffer[at];
      if (version !== 253 && version !== 254) { at++; continue; }
      const header = version === 253 ? 10 : 6;
      if (at + header > buffer.length) break;
      const length = buffer[at + 1];
      const signed = version === 253 && (buffer[at + 2] & 1);
      const end = at + header + length + 2 + (signed ? 13 : 0);
      if (end > buffer.length) break;
      const id = version === 253 ? buffer[at + 7] | buffer[at + 8] << 8 | buffer[at + 9] << 16 : buffer[at + 5];
      const info = schema[id];
      const packet = buffer.slice(at, end), frame = new DataView(packet.buffer);
      if (!info || (version === 253 && (packet[2] & ~1)) ||
          frame.getUint16(header + length, true) !== checksum(packet.subarray(1, header + length), info.crc)) {
        at++; continue;
      }
      const payload = new Uint8Array(info.length);
      payload.set(packet.subarray(header, header + Math.min(length, info.length)));
      const view = new DataView(payload.buffer), fields = {};
      for (const [field, type, offset, count] of info.fields) {
        const [method, size] = methods[type], values = [];
        for (let i = 0; i < count; i++) values.push(view['get' + method](offset + i * size, true));
        fields[field] = count === 1 ? values[0] : type === 'char'
          ? new TextDecoder().decode(Uint8Array.from(values).subarray(0, values.indexOf(0) < 0 ? values.length : values.indexOf(0)))
          : values;
      }
      messages.push({name: info.name, id, system: packet[version === 253 ? 5 : 3],
        component: packet[version === 253 ? 6 : 4], fields, packet, signed: !!signed});
      at = end;
    }
    this.pending = buffer.slice(at);
    return messages;
  }
}
