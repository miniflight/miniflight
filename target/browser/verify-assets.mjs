import {selectedFiles,verifyFiles} from './assets.mjs';
onmessage=async ({data:{manifest,files}})=>{
  try {
    const result=await verifyFiles(manifest,selectedFiles(files),progress=>postMessage({progress}));
    postMessage({result});
  } catch(error) {postMessage({error:error.message});}
};
