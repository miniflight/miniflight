const home='/home/username';
const game=home+'/aigp/FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe';

export async function startRuntime(archive,url,log,status) {
  if(!crossOriginIsolated)throw Error('The page needs cross-origin isolation for the browser runtime');
  const base=new URL('./.runtime/',import.meta.url);
  const response=await fetch(new URL('wine64.zip.manifest.json',base));
  if(!response.ok)throw Error('Browser runtime is missing. Run target/browser/prepare.py first.');
  const wine=await response.json();let left=wine.totalBytes;
  let wineMount='bw64url:'+wine.totalBytes;
  for(const part of wine.parts){const size=Math.min(wine.chunkBytes,left);left-=size;
    wineMount+=';'+new URL(part,base).href+'|'+size;}
  if(left!==0)throw Error('Incomplete Wine asset manifest');
  const names=['glibc-rootfs64.zip','prefix64.zip','unreal-startup.zip'];
  const downloads=Promise.all(names.map(async name=>{
    const response=await fetch(new URL(name,base));
    if(!response.ok)throw Error('Missing browser runtime asset: '+name);
    return new Uint8Array(await response.arrayBuffer());
  }));
  const args=['-root','/root','-zip',names[0],'-zip',wineMount,'-zip',names[1],'-zip',names[2],
    '-zip','bw64url:'+archive.size+';'+url+'|'+archive.size];
  for(const value of ['HOME='+home,'WINEPREFIX='+home+'/.wine','WINESERVER=/usr/lib/wine/wineserver64',
      'WINEDLLPATH=/usr/lib/x86_64-linux-gnu/wine','WINEDLLOVERRIDES=dwmapi=n,b;winegstreamer=',
      'WINE_D3D_CONFIG=csmt=0x0','WINEDEBUG=-all'])args.push('-env',value);
  args.push(home+'/unreal-startup.elf','Z:'+game.replaceAll('/','\\'),
    '/Game/levelsMaster/MAP_anduril_master?game=/Script/DCGame.GameModeRaceBase',
    '-nullrhi','-nosound','-unattended','-stdout','-FullStdOutLogOutput');
  globalThis.Module={arguments:args,canvas:document.querySelector('#canvas'),
    locateFile:name=>new URL(name,base).href,
    print:text=>log(String(text)),printErr:text=>log(String(text)),setStatus:status,
    preRun:[()=>{
      const module=globalThis.Module;
      module.ENV.BW64_GLTRACE='0';module.addRunDependency('aigp-assets');
      downloads.then(bytes=>{
        names.forEach((name,i)=>module.FS.createDataFile('/',name,bytes[i],true,false));
        status('Starting the original AI-GP executable');module.removeRunDependency('aigp-assets');
      }).catch(error=>{status(String(error));log(String(error));});
    }]};
  const script=document.createElement('script');
  script.src=new URL('cpu-fixed-boxedwine64.js',base).href;
  await new Promise((resolve,reject)=>{script.onload=resolve;script.onerror=()=>reject(Error('Cannot load browser runtime'));
    document.body.append(script);});
  return globalThis.Module;
}
