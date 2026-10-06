import {Mavlink} from './mavlink.mjs';

const now = () => performance.now() / 1000;
const finite = values => values.every(Number.isFinite);
const frozen = value => Object.freeze(value);

export class SimulatorClient extends EventTarget {
  constructor(write) {
    super();
    this.startedAt = now();
    this.targetIds = null;
    this.peer = null;
    this.telemetry = new Map();
    this.mav = new Mavlink(bytes => write(bytes, this.peer));
    this.race = this.gates = this.frame = null;
    this.lastImu = null;
    this.track = this.camera = null;
  }

  receive(bytes, port = 14550, peer = null) {
    const receivedAt = now();
    if (port === 5600) { this.receiveCamera(bytes, receivedAt); return; }
    if (this.peer && peer && this.peer !== peer) return;
    for (const message of this.mav.receive(bytes)) {
      if (!this.targetIds) {
        if (message.name !== 'HEARTBEAT') continue;
        this.targetIds = [message.system, message.component]; this.peer = peer;
      }
      if (message.system !== this.targetIds[0]) continue;
      const value = message.fields;
      if (message.name === 'HIGHRES_IMU' &&
          (!finite([value.xacc,value.yacc,value.zacc,value.xgyro,value.ygyro,value.zgyro]) ||
           value.time_usec <= (this.telemetry.get('HIGHRES_IMU')?.fields.time_usec ?? -1n))) continue;
      message.receivedAt = receivedAt;
      this.telemetry.set(message.name, message);
      if (message.name === 'TIMESYNC' && value.tc1 === 0n)
        this.mav.send('TIMESYNC', {tc1: BigInt(Math.round(receivedAt * 1e9)), ts1: value.ts1});
      if (message.name === 'DATA_TRANSMISSION_HANDSHAKE') {
        const {size, packets, width} = value;
        if (size >= 2 && size <= 38914 && packets === Math.ceil(size / 250))
          this.track = {id: width, size, packets, chunks: new Map()};
      }
      if (message.name === 'ENCAPSULATED_DATA') {
        const data = Uint8Array.from(value.data), view = new DataView(data.buffer);
        if (data[0] === 1) {
          this.race = frozen({sim_boot_time_ms: Number(view.getBigUint64(1, true)),
            race_start_boot_time_ms: Number(view.getBigInt64(9, true)),
            race_finish_time_ns: view.getBigInt64(17, true), active_gate_index: view.getUint32(25, true),
            last_gate_race_time: view.getBigInt64(29, true), received_at: receivedAt});
          if (this.race.race_start_boot_time_ms < 0 &&
              this.lastRaceBoot > this.race.sim_boot_time_ms) {
            this.telemetry.delete('HIGHRES_IMU'); this.lastImu = null;
          }
          this.lastRaceBoot = this.race.sim_boot_time_ms;
        } else if (data[0] === 2 && this.track && view.getUint16(1, true) === this.track.id) {
          const t = this.track, index = value.seqnr;
          if (index < t.packets) t.chunks.set(index, data.slice(3, 3 + Math.min(250, t.size - index * 250)));
          if (t.chunks.size === t.packets) this.finishTrack();
        }
      }
      this.dispatchEvent(new CustomEvent('packet', {detail: message}));
    }
  }

  finishTrack() {
    const track = this.track; this.track = null;
    const bytes = new Uint8Array(track.size);
    for (const [index, chunk] of track.chunks) bytes.set(chunk, index * 250);
    const view = new DataView(bytes.buffer), count = view.getUint16(0, true), gates = [];
    if (!count || count > 1024 || bytes.length !== 2 + count * 38) return;
    for (let index = 0; index < count; index++) {
      const at = 2 + index * 38, id = view.getUint16(at, true);
      const values = Array.from({length: 9}, (_, i) => view.getFloat32(at + 2 + i * 4, true));
      const [north,east,down,w,x,y,z,width,height] = values;
      const norm = Math.hypot(w,x,y,z);
      if (id !== index || !finite(values) || width <= 0 || height <= 0 || norm < .99 || norm > 1.01) return;
      const q = [w,x,y,z].map(value => value / norm), [qw,qx,qy,qz] = q;
      const h = -height / 2;
      const offset = [2*(qx*qz+qw*qy)*h, 2*(qy*qz-qw*qx)*h, (1-2*(qx*qx+qy*qy))*h];
      const origin = [north,east,down];
      gates.push(frozen({id,center: frozen(origin.map((v,i) => v + offset[i])),
        origin: frozen(origin), orientation: frozen(q), width, height}));
    }
    this.gates = frozen(gates);
  }

  read(timeout = 1) {
    const imu = this.telemetry.get('HIGHRES_IMU');
    if (!imu || imu === this.lastImu || now() - imu.receivedAt > timeout) return null;
    const sample = name => {
      const message = this.telemetry.get(name);
      return message && now() - message.receivedAt <= timeout ? message : null;
    };
    const previous = this.lastImu;
    this.lastImu = imu;
    const f = imu.fields, state = {time: Number(f.time_usec) * 1e-6,
      dt: previous ? Number(f.time_usec - previous.fields.time_usec) * 1e-6 : 0,
      received_at: imu.receivedAt, acceleration: frozen([f.xacc,f.yacc,f.zacc]),
      gyro: frozen([-f.xgyro,-f.ygyro,-f.zgyro]), motion: null, attitude: null, motors: null,
      frame: this.frame && now() - this.frame.received_at <= timeout ? this.frame : null};
    const pose = sample('LOCAL_POSITION_NED');
    if (pose) {
      const p = pose.fields, values = [p.x,p.y,p.z,p.vx,p.vy,p.vz];
      if (finite(values)) state.motion = frozen({time:p.time_boot_ms/1000,received_at:pose.receivedAt,
        position:frozen(values.slice(0,3)),velocity:frozen(values.slice(3))});
    }
    const attitude = sample('ATTITUDE');
    if (attitude && finite([attitude.fields.roll,attitude.fields.pitch,attitude.fields.yaw]))
      state.attitude = frozen({time:attitude.fields.time_boot_ms/1000,received_at:attitude.receivedAt,
        roll:attitude.fields.roll,pitch:-attitude.fields.pitch,yaw:-attitude.fields.yaw});
    const motors = sample('ACTUATOR_OUTPUT_STATUS');
    if (motors) state.motors = frozen({time:Number(motors.fields.time_usec)*1e-6,received_at:motors.receivedAt,
      outputs:frozen(motors.fields.actuator),active:motors.fields.active});
    return frozen(state);
  }

  send(command) {
    if (!this.targetIds) throw Error('No simulator heartbeat');
    const [target_system,target_component] = this.targetIds;
    const time_boot_ms = Math.floor((now() - this.startedAt) * 1000) >>> 0;
    if (command.type === 'BodyRates') {
      const {roll_rate=0,pitch_rate=0,yaw_rate=0,thrust=0} = command;
      if (!finite([roll_rate,pitch_rate,yaw_rate,thrust]) || thrust < 0 || thrust > 1)
        throw Error('Invalid body rates or thrust');
      return this.mav.send('SET_ATTITUDE_TARGET', {time_boot_ms,target_system,target_component,type_mask:144,
        q:[1,0,0,0],body_roll_rate:-roll_rate,body_pitch_rate:-pitch_rate,body_yaw_rate:-yaw_rate,thrust});
    }
    if (!['PositionNed','VelocityNed'].includes(command.type) ||
        !finite([command.north,command.east,command.down])) throw Error('Invalid NED command');
    const position = command.type === 'PositionNed', v = [command.north,command.east,command.down];
    return this.mav.send('SET_POSITION_TARGET_LOCAL_NED', {time_boot_ms,target_system,target_component,
      coordinate_frame:1,type_mask:position?3576:3527,x:position?v[0]:0,y:position?v[1]:0,z:position?v[2]:0,
      vx:position?0:v[0],vy:position?0:v[1],vz:position?0:v[2]});
  }

  heartbeat() { return this.mav.send('HEARTBEAT',{type:6,autopilot:8,system_status:4,mavlink_version:3}); }
  arm(armed = true) {
    if (!this.targetIds) throw Error('No simulator heartbeat');
    return this.mav.send('COMMAND_LONG',{target_system:this.targetIds[0],target_component:this.targetIds[1],
      command:400,param1:Number(armed)});
  }
  stop() {
    if (this.targetIds) { this.send({type:'BodyRates'}); this.arm(false); }
  }

  async receiveCamera(packet, receivedAt) {
    if (packet.length < 24) return;
    const view = new DataView(packet.buffer,packet.byteOffset,packet.length);
    const id=view.getUint32(0,true),index=view.getUint16(4,true),count=view.getUint16(6,true);
    const size=view.getUint32(8,true),length=view.getUint32(12,true),stamp=view.getBigUint64(16,true);
    if (index >= count || count > 4096 || !size || size > 8388608 || length !== packet.length-24 ||
        !length || length > size || stamp <= (this.frame?.time_ns ?? -1n)) return;
    if (!this.camera || stamp > this.camera.stamp || receivedAt-this.camera.startedAt > .5)
      this.camera = {id,count,size,stamp,startedAt:receivedAt,chunks:new Map()};
    const frame = this.camera;
    if (frame.id!==id || frame.count!==count || frame.size!==size || frame.stamp!==stamp) return;
    frame.chunks.set(index,packet.slice(24));
    if ([...frame.chunks.values()].reduce((n,v)=>n+v.length,0)>size) {this.camera=null;return;}
    if (frame.chunks.size!==count) return;
    this.camera=null;
    const bytes = new Uint8Array(size); let at=0;
    for(let i=0;i<count;i++){const chunk=frame.chunks.get(i);bytes.set(chunk,at);at+=chunk.length;}
    if(at!==size) return;
    try {
      const image = await createImageBitmap(new Blob([bytes],{type:'image/jpeg'}));
      if(stamp <= (this.frame?.time_ns ?? -1n)){image.close();return;}
      const canvas = new OffscreenCanvas(image.width,image.height), context=canvas.getContext('2d');
      context.drawImage(image,0,0); const rgba=context.getImageData(0,0,image.width,image.height).data;
      const bgr = new Uint8Array(image.width*image.height*3);
      for(let i=0,j=0;i<rgba.length;i+=4,j+=3){bgr[j]=rgba[i+2];bgr[j+1]=rgba[i+1];bgr[j+2]=rgba[i];}
      image.close();
      this.frame = frozen({id,time_ns:stamp,received_at:receivedAt,bgr,width:canvas.width,height:canvas.height});
      this.dispatchEvent(new CustomEvent('frame',{detail:this.frame}));
    } catch (_) { /* Incomplete or invalid JPEGs do not replace the last camera frame. */ }
  }
}

export class GateController {
  target = null; gate = null;
  update(state,index,gates) {
    if(!state.motion || !gates?.length) return null;
    if(index < gates.length && gates[index] !== this.gate) {
      const center=gates[index].center,position=state.motion.position;
      let direction=center.map((v,i)=>v-position[i]),distance=Math.hypot(...direction);
      if(distance<.1){const previous=index?gates[index-1].center:[0,0,0];direction=center.map((v,i)=>v-previous[i]);distance=Math.hypot(...direction);}
      if(!distance) throw Error('Gate has no approach direction');
      const [north,east,down]=center.map((v,i)=>v+direction[i]/distance);
      this.target=frozen({type:'PositionNed',north,east,down});this.gate=gates[index];
    }
    return this.target;
  }
}
