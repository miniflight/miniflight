import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {GameArchive,selectedFiles,verifyFiles} from '../target/browser/assets.mjs';

const prefix='home/username/aigp/',path='FlightSim/Binaries/Win64/DCGame-Win64-Shipping.exe';
const bytes=new TextEncoder().encode('123456789'),name=prefix+path;
const entry={path,size:9,crc32:0xcbf43926,local_offset:0,data_offset:30+name.length};
const directory=entry.data_offset+9;
const manifest={prefix,files:[entry],zip_size:directory+46+name.length+98,
  executable_sha256:createHash('sha256').update(bytes).digest('hex')};
const file=new File([bytes],'DCGame-Win64-Shipping.exe');
Object.defineProperty(file,'webkitRelativePath',{value:'vq1/'+path});
const files=selectedFiles([file]);
assert.deepEqual(await verifyFiles(manifest,files),{files:1,bytes:9,executable_sha256:manifest.executable_sha256});
await assert.rejects(verifyFiles({...manifest,files:[{...entry,crc32:0}]},files),/Content differs/);
await assert.rejects(verifyFiles({...manifest,executable_sha256:'wrong'},files),/executable differs/);
const archive=new GameArchive(manifest,files);
assert.deepEqual(new Uint8Array(await archive.slice(entry.data_offset,entry.data_offset+9).arrayBuffer()),bytes);
const restore=archive.installFetch('https://assets.test/game.zip');
const head=await fetch('https://assets.test/game.zip',{method:'HEAD'});
assert.equal(Number(head.headers.get('Content-Length')),archive.size);
const range=await fetch('https://assets.test/game.zip',{headers:{Range:'bytes='+entry.data_offset+'-'+(entry.data_offset+8)}});
assert.equal(range.status,206);assert.deepEqual(new Uint8Array(await range.arrayBuffer()),bytes);
const tail=await fetch('https://assets.test/game.zip',{headers:{Range:'bytes=-22'}});
assert.equal(new DataView(await tail.arrayBuffer()).getUint32(0,true),0x06054b50);
for(const value of ['bytes=1-0','bytes=0-1,3-4','bytes=-0','bytes=999999-'])
  assert.equal((await fetch('https://assets.test/game.zip',{headers:{Range:value}})).status,416);
restore();assert.throws(()=>archive.slice(-1,0),/Invalid ZIP range/);

// No multi-gigabyte allocation: only requested bytes of the sparse payload exist.
const largeSize=0x100000000+17,largeName=prefix+'large.pak',smallName=prefix+'after.txt';
const large={path:'large.pak',size:largeSize,crc32:0,local_offset:0,data_offset:30+largeName.length+20};
const after={path:'after.txt',size:9,crc32:entry.crc32,local_offset:large.data_offset+largeSize};
after.data_offset=after.local_offset+30+smallName.length;
const largeDirectory=after.data_offset+9;
const largeManifest={prefix,files:[large,after],zip_size:largeDirectory+46+largeName.length+20+46+smallName.length+12+98};
const sparse={size:largeSize,slice:(start,end)=>new Uint8Array(end-start)};
const largeArchive=new GameArchive(largeManifest,new Map([['large.pak',sparse],['after.txt',new Blob([bytes])]]));
const crossing=new Uint8Array(await largeArchive.slice(after.local_offset-8,after.data_offset+9).arrayBuffer());
assert.deepEqual([...crossing.slice(0,8)],[0,0,0,0,0,0,0,0]);
assert.equal(new DataView(crossing.buffer).getUint32(8,true),0x04034b50);
assert.deepEqual(crossing.slice(-9),bytes);
const ending=new DataView(await largeArchive.slice(largeArchive.size-98,largeArchive.size).arrayBuffer());
assert.equal(ending.getUint32(0,true),0x06064b50);
assert.equal(ending.getBigUint64(48,true),BigInt(largeDirectory));
assert.equal(ending.getUint32(92,true),0xffffffff);

if(process.argv.includes('--zip-fixture')) {
  const segments=await Promise.all(largeArchive.segments.filter(s=>s.payload instanceof Uint8Array||s.payload instanceof Blob)
    .map(async s=>({start:s.start,hex:Buffer.from(s.payload instanceof Blob?await s.payload.arrayBuffer():s.payload).toString('hex')})));
  console.log(JSON.stringify({size:largeArchive.size,segments,directory:largeDirectory,largeSize}));
} else console.log(JSON.stringify({fileChecksums:true,executableSha256:true,rangeFetch:true,zip64:true,largePayloadAllocated:false}));
