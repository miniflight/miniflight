import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {Mavlink} from '../target/browser/mavlink.mjs';
import {SimulatorClient, GateController} from '../target/browser/client.mjs';
import {browserDatagrams} from '../target/browser/transport.mjs';

const fixture = JSON.parse(await readFile(new URL('./browser_io_fixtures.json',import.meta.url)));
const fromHex = value => Uint8Array.from(Buffer.from(value,'hex'));
const decoder = new Mavlink();
let count = 0;
for(const expected of fixture.incoming) {
  const packet=fromHex(expected.hex),split=Math.floor(packet.length/2);
  assert.equal(decoder.receive(packet.slice(0,split)).length,0);
  const [message] = decoder.receive(packet.slice(split));
  assert.equal(message.name,expected.name);
  for(const [field,value] of Object.entries(expected.fields)) {
    if(field==='mavpackettype') continue;
    const actual=message.fields[field];
    if(typeof actual==='bigint') assert.equal(actual,BigInt(value));
    else if(typeof value==='number') assert.ok(Math.abs(actual-value)<=Math.max(1e-6,Math.abs(value)*1e-6),field);
    else if(Array.isArray(value)) {
      assert.equal(actual.length,value.length);
      value.forEach((item,i)=>assert.ok(Math.abs(actual[i]-item)<=Math.max(1e-6,Math.abs(item)*1e-6),field));
    }
    else assert.equal(actual,value);
  }
  const corrupt=packet.slice();corrupt[10]^=1;
  assert.equal(new Mavlink().receive(corrupt).length,0);
  count++;
}
const encoder = new Mavlink();
for(const expected of fixture.outgoing) {
  const packet=encoder.send(expected.name,expected.fields);
  assert.equal(Buffer.from(packet).toString('hex'),expected.hex);
}
const sent=[],client=new SimulatorClient(bytes=>sent.push(bytes));
for(const packet of fixture.incoming)client.receive(fromHex(packet.hex));
const state=client.read();
assert.deepEqual(state.acceleration,[1,2,3]);assert.deepEqual(state.gyro,[-4,-5,-6]);
assert.deepEqual(state.motion.position,[1,2,3]);assert.equal(state.dt,0);
assert.ok(Math.abs(state.attitude.pitch+.2)<1e-6);assert.ok(Math.abs(state.attitude.yaw+.3)<1e-6);
assert.equal(client.read(),null);assert.equal(client.telemetry.get('ODOMETRY').fields.time_usec,1000000n);
assert.equal(client.gates.length,6);
client.gates.forEach((gate,index)=>gate.center.forEach((value,axis)=>assert.ok(Math.abs(value-fixture.gateCenters[index][axis])<1e-5)));
const controller=new GateController(),target=controller.update(state,0,client.gates);
assert.equal(target.type,'PositionNed');assert.ok(Number.isFinite(target.north));
client.send({type:'BodyRates',roll_rate:.1,pitch_rate:-.2,yaw_rate:.3,thrust:.4});
const [command]=new Mavlink().receive(sent.at(-1));
assert.equal(command.fields.type_mask,144);assert.ok(Math.abs(command.fields.body_roll_rate+.1)<1e-6);
assert.throws(()=>client.send({type:'BodyRates',thrust:2}));
client.stop();assert.equal(new Mavlink().receive(sent.at(-1))[0].fields.param1,0);
globalThis.CloseEvent ??= class extends Event {constructor(type,values){super(type);Object.assign(this,values);}};
globalThis.WebSocket ??= class {constructor(url){this.url=url;}};
const received=[];
const bridge=browserDatagrams((bytes,port,peer)=>received.push({bytes,port,peer}));
const peer=new WebSocket('ws://127.0.0.1:14550/');
await new Promise(resolve=>peer.addEventListener('open',resolve,{once:true}));
peer.send(Uint8Array.from([255,255,255,255,112,111,114,116,57,58]));
assert.equal(peer.sourcePort,14650);assert.equal(received.length,0);
peer.send(Uint8Array.from([79,90]));assert.deepEqual([...received[0].bytes],[79,90]);
const incoming=new Promise(resolve=>peer.addEventListener('message',e=>resolve(new Uint8Array(e.data)),{once:true}));
bridge.send(Uint8Array.from([1,2,3]),14550,peer);
assert.deepEqual([...await incoming],[1,2,3]);bridge.close();
console.log(JSON.stringify({incomingMessages:count,outgoingWireParity:fixture.outgoing.length,trackGates:6,rawIO:true,inTabDatagrams:true,simulatorReadiness:false}));
