const MAX32 = 0xffffffff, PREFIX = 'home/username/aigp/';
const executable = 'FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe';
const crcTable = Uint32Array.from({length:256}, (_, n) => {
  for (let bit=0;bit<8;bit++) n = n&1 ? 0xedb88320^(n>>>1) : n>>>1;
  return n>>>0;
});

function record(fields, name = new Uint8Array(), extra = new Uint8Array()) {
  const bytes = new Uint8Array(fields.reduce((n,[size])=>n+size,0)+name.length+extra.length);
  const view = new DataView(bytes.buffer); let at=0;
  for(const [size,value] of fields) {
    if(size===8) view.setBigUint64(at,BigInt(value),true);
    else if(size===4) view.setUint32(at,value,true);
    else view.setUint16(at,value,true);
    at+=size;
  }
  bytes.set(name,at);bytes.set(extra,at+name.length);return bytes;
}

export function selectedFiles(files) {
  const result=new Map();
  for(const file of files) {
    const selected=file.webkitRelativePath || file.name, slash=selected.indexOf('/');
    const path=slash<0 ? selected : selected.slice(slash+1);
    if(result.has(path)) throw Error('Duplicate selected file: '+path);
    result.set(path,file);
  }
  return result;
}

export async function verifyFiles(manifest, files, progress = ()=>{}) {
  let checked=0;const total=manifest.files.reduce((n,file)=>n+file.size,0);
  for(const entry of manifest.files) {
    const file=files.get(entry.path);
    if(!file || file.size!==entry.size) throw Error('Missing or different file: '+entry.path);
    let crc=MAX32;
    for(let offset=0;offset<file.size;offset+=1024*1024) {
      const bytes=new Uint8Array(await file.slice(offset,offset+1024*1024).arrayBuffer());
      for(const byte of bytes) crc=crcTable[(crc^byte)&255]^(crc>>>8);
      checked+=bytes.length;progress({path:entry.path,checked,total});
    }
    if(((crc^MAX32)>>>0)!==entry.crc32) throw Error('Content differs: '+entry.path);
  }
  const binary=files.get(executable);
  const hash=await crypto.subtle.digest('SHA-256',await binary.arrayBuffer());
  const digest=Array.from(new Uint8Array(hash),byte=>byte.toString(16).padStart(2,'0')).join('');
  if(digest!==manifest.executable_sha256) throw Error('The simulator executable differs');
  return {files:manifest.files.length,bytes:checked,executable_sha256:digest};
}

// STORE ZIP64 metadata is small. Each payload remains a browser-selected File.
export class GameArchive {
  segments=[];size=0;
  constructor(manifest, files) {
    const directory=[],encoder=new TextEncoder();
    if(manifest.prefix!==PREFIX) throw Error('Unexpected game archive prefix');
    for(const entry of manifest.files) {
      const file=files.get(entry.path),name=encoder.encode(PREFIX+entry.path);
      if(!file || file.size!==entry.size) throw Error('Missing or different file: '+entry.path);
      if(!Number.isSafeInteger(entry.size) || entry.size<0 || name.length>65535 ||
          entry.path.split('/').some(part=>!part || part==='.' || part==='..') || entry.path.includes('\\'))
        throw Error('Invalid game archive entry');
      const offset=this.size,large=entry.size>=MAX32,far=offset>=MAX32;
      const size=large?MAX32:entry.size;
      const extra=large?record([[2,1],[2,16],[8,entry.size],[8,entry.size]]):new Uint8Array();
      if(offset!==entry.local_offset) throw Error('Local ZIP offset differs: '+entry.path);
      this.add(record([[4,0x04034b50],[2,large?45:20],[2,0x800],[2,0],[2,0],[2,33],
        [4,entry.crc32],[4,size],[4,size],[2,name.length],[2,extra.length]],name,extra));
      if(this.size!==entry.data_offset) throw Error('ZIP data offset differs: '+entry.path);
      this.add(file);
      const values=[...(large?[[8,entry.size],[8,entry.size]]:[]),...(far?[[8,offset]]:[])];
      const centralExtra=values.length?record([[2,1],[2,values.length*8],...values]):new Uint8Array();
      const version=values.length?45:20;
      directory.push(record([[4,0x02014b50],[2,(3<<8)|version],[2,version],[2,0x800],
        [2,0],[2,0],[2,33],[4,entry.crc32],[4,size],[4,size],[2,name.length],[2,centralExtra.length],
        [2,0],[2,0],[2,0],[4,0o100644<<16],[4,far?MAX32:offset]],name,centralExtra));
    }
    const start=this.size,length=directory.reduce((n,bytes)=>n+bytes.length,0),count=directory.length;
    for(const bytes of directory)this.add(bytes);
    const zip64=this.size;
    this.add(record([[4,0x06064b50],[8,44],[2,45],[2,45],[4,0],[4,0],[8,count],[8,count],[8,length],[8,start]]));
    this.add(record([[4,0x07064b50],[4,0],[8,zip64],[4,1]]));
    this.add(record([[4,0x06054b50],[2,0],[2,0],[2,Math.min(count,65535)],[2,Math.min(count,65535)],
      [4,Math.min(length,MAX32)],[4,Math.min(start,MAX32)],[2,0]]));
    if(this.size!==manifest.zip_size || !Number.isSafeInteger(this.size)) throw Error('ZIP length differs');
  }
  add(payload) {
    const size=payload.size??payload.length;
    if(size){this.segments.push({start:this.size,size,payload});this.size+=size;}
  }
  slice(start,end) {
    if(!Number.isSafeInteger(start)||!Number.isSafeInteger(end)||start<0||end<start||end>this.size)
      throw RangeError('Invalid ZIP range');
    const parts=[];
    for(const segment of this.segments) {
      if(segment.start>=end)break;
      if(segment.start+segment.size<=start)continue;
      parts.push(segment.payload.slice(Math.max(0,start-segment.start),Math.min(segment.size,end-segment.start)));
    }
    return new Blob(parts,{type:'application/zip'});
  }
  response(request) {
    const headers={'Accept-Ranges':'bytes','Content-Type':'application/zip'};
    if(request.method==='HEAD')return new Response(null,{headers:{...headers,'Content-Length':this.size}});
    if(request.method!=='GET')return new Response(null,{status:405,headers:{Allow:'GET, HEAD'}});
    const range=request.headers.get('Range'),match=/^bytes=(\d*)-(\d*)$/.exec(range??'');
    if(!match || (!match[1]&&!match[2]))return new Response('A byte range is required',{status:416,
      headers:{...headers,'Content-Range':'bytes */'+this.size}});
    const start=match[1]?Number(match[1]):Math.max(0,this.size-Number(match[2]));
    const end=match[1]?(match[2]?Math.min(this.size,Number(match[2])+1):this.size):this.size;
    if(start>=end||!Number.isSafeInteger(start)||!Number.isSafeInteger(end))return new Response(null,{status:416,
      headers:{...headers,'Content-Range':'bytes */'+this.size}});
    return new Response(this.slice(start,end),{status:206,headers:{...headers,
      'Content-Length':end-start,'Content-Range':'bytes '+start+'-'+(end-1)+'/'+this.size}});
  }
  installFetch(url) {
    const native=globalThis.fetch,href=new URL(url,globalThis.location?.href).href;
    const replacement=(input,options)=>{
      const request=new Request(input instanceof Request?input:new URL(input,globalThis.location?.href),options);
      return request.url===href?Promise.resolve(this.response(request)):native(input,options);
    };
    globalThis.fetch=replacement;
    return ()=>{if(globalThis.fetch===replacement)globalThis.fetch=native;};
  }
}
