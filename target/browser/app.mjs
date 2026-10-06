import {GameArchive,selectedFiles} from './assets.mjs';
import {browserDatagrams} from './transport.mjs';
import {SimulatorClient,GateController} from './client.mjs';
import {startRuntime} from './runtime.mjs';

const element=id=>document.getElementById(id),status=text=>{if(text)element('status').textContent=text;};
const log=text=>{element('log').textContent=(element('log').textContent+text+'\n').slice(-32768);};
let archive,restoreFetch,transport,sim,lastState,controller,loop,running=false,booted=false;
element('folder').onchange=async event=>{
  if(booted)return;
  element('start').disabled=true;element('folder').disabled=true;archive=null;
  const worker=new Worker(new URL('./verify-assets.mjs',import.meta.url),{type:'module'});
  try {
    const response=await fetch(new URL('./aigp-vq1-manifest.json',import.meta.url));
    if(!response.ok)throw Error('Cannot read the original package manifest');
    const manifest=await response.json(),files=Array.from(event.target.files);
    status('Checking original files');
    await new Promise((resolve,reject)=>{
      worker.onmessage=({data})=>{
        if(data.progress)status('Checking original files: '+Math.round(data.progress.checked/data.progress.total*100)+'%');
        if(data.result)resolve(data.result);if(data.error)reject(Error(data.error));
      };worker.onerror=reject;worker.postMessage({manifest,files});
    });
    archive=new GameArchive(manifest,selectedFiles(files));
    status('Original files verified. Ready for the browser startup check.');element('start').disabled=false;
  }catch(error){status(String(error));}finally{worker.terminate();element('folder').disabled=false;}
};
function stop() {
  running=false;
  try{sim?.stop();}catch(error){log(String(error));}
  element('stop').disabled=true;
}
element('start').onclick=async()=>{
  element('start').disabled=true; element('folder').disabled=true;
  try {
    const url=new URL('./game.zip',location.href).href;
    restoreFetch=archive.installFetch(url);
    transport=browserDatagrams((bytes,port,peer)=>sim.receive(bytes,port,peer));
    sim=new SimulatorClient((bytes,peer)=>transport.send(bytes,14550,peer));
    globalThis.aigp={sim,archive,transport};
    sim.addEventListener('frame',({detail:frame})=>{
      const canvas=element('camera'),context=canvas.getContext('2d');canvas.width=frame.width;canvas.height=frame.height;
      canvas.style.display='block';
      const rgba=new Uint8ClampedArray(frame.width*frame.height*4);
      for(let i=0,j=0;i<frame.bgr.length;i+=3,j+=4){rgba[j]=frame.bgr[i+2];rgba[j+1]=frame.bgr[i+1];rgba[j+2]=frame.bgr[i];rgba[j+3]=255;}
      context.putImageData(new ImageData(rgba,frame.width,frame.height),0,0);
    });
    await startRuntime(archive,url,log,status);booted=true;
    let heartbeatAt=0;
    loop=setInterval(()=>{
      const now=performance.now();
      if(sim.targetIds&&now-heartbeatAt>=1000){sim.heartbeat();heartbeatAt=now;}
      const state=sim.read();if(state)lastState=state;
      const fresh=lastState&&now/1000-lastState.received_at<=1;
      const race=sim.race,go=race&&race.race_start_boot_time_ms>=0&&race.race_finish_time_ns<0n;
      const ready=go&&fresh&&lastState.motion&&sim.gates?.length;
      element('fly').disabled=running||!ready;
      element('telemetry').textContent=JSON.stringify({nativeHeartbeat:!!sim.targetIds,nativeGo:!!go,
        freshImu:!!fresh,gate:race?.active_gate_index,gates:sim.gates?.length,
        position:lastState?.motion?.position,messages:[...sim.telemetry.keys()],controllerRunning:running},null,2);
      if(running){
        if(!ready){stop();status('Controller stopped: native race or fresh telemetry is unavailable');return;}
        if(state){const command=controller.update(state,race.active_gate_index,sim.gates);if(command)sim.send(command);}
      }
    },20);
  }catch(error){log(String(error));status(String(error));transport?.close();restoreFetch?.();
    if(!globalThis.Module){element('folder').disabled=false;element('start').disabled=false;}}
};
element('fly').onclick=()=>{controller=new GateController();sim.arm(true);running=true;element('stop').disabled=false;status('Controller running on native telemetry');};
element('stop').onclick=()=>{stop();status('Controller stopped');};
addEventListener('pagehide',()=>{stop();clearInterval(loop);transport?.close();restoreFetch?.();});
