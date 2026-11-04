/** Three's production switch avoids synchronous success-log queries. Keep a
 * real LINK_STATUS check for every new program, and report failed links rather
 * than treating "verbose logs disabled" as "shader errors ignored".
 */
export type ShaderDiagnostics='errors-only'|'verbose';
type GL=Pick<WebGLRenderingContext,'linkProgram'|'getProgramParameter'|'getProgramInfoLog'|'isContextLost'|'LINK_STATUS'>;
const activeContexts=new WeakSet<object>();
export function createShaderChecks(gl:GL,debug:{checkShaderErrors:boolean},requested:ShaderDiagnostics){
  if(activeContexts.has(gl))throw new Error('Shader checks already own this WebGL context');
  const previous=gl.linkProgram,previousDebug=debug.checkShaderErrors;
  const pending=new Set<WebGLProgram>(),failures=new Set<string>();let disposed=false;
  let mode:ShaderDiagnostics=requested;
  const link=(program:WebGLProgram)=>{previous.call(gl,program);if(!disposed)pending.add(program);};
  try{gl.linkProgram=link;if(gl.linkProgram!==link)mode='verbose';}catch{mode='verbose';}
  debug.checkShaderErrors=mode==='verbose';
  activeContexts.add(gl);
  return {
    mode,
    get errors(){return [...failures];},
    // Native loss and restoration can both occur before the next flush. The
    // old context's objects are invalid even when isContextLost() is false now.
    resetPending(){pending.clear();},
    flush(){
      if(disposed||!pending.size)return;
      // Native WebGL returns null while lost. That is not evidence of a
      // failed shader; old handles must not be checked after restoration.
      if(gl.isContextLost()){pending.clear();return;}
      for(const program of pending){
        pending.delete(program);
        try{
          const linked=gl.getProgramParameter(program,gl.LINK_STATUS);
          if(gl.isContextLost()){pending.clear();return;}
          if(linked===false){
            const log=gl.getProgramInfoLog(program)||'No linker log';
            if(gl.isContextLost()){pending.clear();return;}
            if(failures.size<4)failures.add(`着色器链接失败：${String(log).slice(0,2000)}`);
          }else if(linked!==true){
            if(failures.size<4)failures.add('无法验证着色器链接状态');
          }
        }catch{
          if(gl.isContextLost()){pending.clear();return;}
          if(failures.size<4)failures.add('无法验证着色器链接状态');
        }
      }
    },
    dispose(){
      if(disposed)return;disposed=true;pending.clear();failures.clear();
      // A surface can become read-only after installation. Each best-effort
      // restoration must still release the other state and context ownership.
      try{if(gl.linkProgram===link)gl.linkProgram=previous;}catch{}
      try{debug.checkShaderErrors=previousDebug;}catch{}
      activeContexts.delete(gl);
    },
  };
}
