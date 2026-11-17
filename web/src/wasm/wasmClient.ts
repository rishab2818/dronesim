import type { BasicConfig, HistoryRow, PilotCommand, Snapshot, Status, V3, Q4 } from '../types';

type Fn=(...args:number[])=>number;
interface EmscriptenModule {
  HEAPF64:Float64Array; _malloc(n:number):number; _free(p:number):void; UTF8ToString(p:number):string;
  _wasm_sim_create_default:Fn; _wasm_sim_destroy:Fn; _wasm_sim_last_error:Fn; _wasm_sim_reset:Fn;
  _wasm_sim_set_armed:Fn; _wasm_sim_set_mode:Fn; _wasm_sim_set_input:Fn;
  _wasm_sim_step:Fn; _wasm_sim_apply_basic_config:Fn; _wasm_sim_get_snapshot:Fn;
  _wasm_sim_history_count:Fn; _wasm_sim_get_history_row:Fn; _wasm_sim_clear_history:Fn;
}

export class WasmClient {
  private constructor(private m:EmscriptenModule,private handle:number,private snapshotPtr:number,private historyPtr:number){}
  static async create():Promise<WasmClient>{
    const factory=(await import('./generated/drone_sim.js')).default as (options:object)=>Promise<EmscriptenModule>;
    const m=await factory({});
    const handle=(m._wasm_sim_create_default as Fn)();
    if(!handle)throw new Error('C engine could not allocate a simulator.');
    return new WasmClient(m,handle,m._malloc(44*8),m._malloc(27*8));
  }
  destroy(){this.m._free(this.snapshotPtr);this.m._free(this.historyPtr);(this.m._wasm_sim_destroy as Fn)(this.handle);}
  private check(code:number,action:string){if(code!==0){const detail=this.m.UTF8ToString(this.m._wasm_sim_last_error(this.handle));throw new Error(`${action} failed: ${detail} (C error ${code})`);}}
  reset(){this.check((this.m._wasm_sim_reset as Fn)(this.handle),'Reset');}
  arm(value:boolean){this.check((this.m._wasm_sim_set_armed as Fn)(this.handle,value?1:0),'Arm/disarm');}
  setMode(mode:0|1){this.check((this.m._wasm_sim_set_mode as Fn)(this.handle,mode),'Mode change');}
  setInput(p:PilotCommand){this.check((this.m._wasm_sim_set_input as Fn)(this.handle,p.collective,p.roll,p.pitch,p.yaw,p.override?1:0,...p.motors),'Pilot input');}
  step(dt:number){this.check((this.m._wasm_sim_step as Fn)(this.handle,dt),'Physics step');}
  applyConfig(c:BasicConfig){this.check((this.m._wasm_sim_apply_basic_config as Fn)(this.handle,c.mass,c.ixx,c.iyy,c.izz,c.kT,c.kQ,c.maxRpm,c.motorTau,c.gravity,c.drag,c.dragEnabled?1:0),'Configuration');}
  snapshot():Snapshot{
    const count=(this.m._wasm_sim_get_snapshot as Fn)(this.handle,this.snapshotPtr,44);if(count!==44)throw new Error('C snapshot ABI mismatch.');
    const a=this.m.HEAPF64.subarray(this.snapshotPtr/8,this.snapshotPtr/8+44),v=(i:number)=>[a[i],a[i+1],a[i+2]] as V3,q=(i:number)=>[a[i],a[i+1],a[i+2],a[i+3]] as Q4;
    return {time:a[0],step:a[1],position:v(2),velocity:v(5),quaternion:q(8),omega:v(12),acceleration:v(15),force:v(18),moment:v(21),rpm:[a[24],a[25],a[26],a[27]],thrust:[a[28],a[29],a[30],a[31]],motorCommand:[a[32],a[33],a[34],a[35]],collective:a[36],status:a[37] as Status,mode:a[38] as 0|1,armed:!!a[39]};
  }
  history():HistoryRow[]{
    const count=(this.m._wasm_sim_history_count as Fn)(this.handle),rows:HistoryRow[]=[];
    for(let index=0;index<count;index++){const n=(this.m._wasm_sim_get_history_row as Fn)(this.handle,index,this.historyPtr,27);if(n!==27)continue;const a=this.m.HEAPF64.slice(this.historyPtr/8,this.historyPtr/8+27);rows.push({time:a[0],position:[a[1],a[2],a[3]],quaternion:[a[4],a[5],a[6],a[7]],velocity:[a[8],a[9],a[10]],omega:[a[11],a[12],a[13]],rpm:[...a.slice(14,18)],thrust:[...a.slice(18,22)],commands:[...a.slice(22,26)],status:a[26] as Status});}
    return rows;
  }
  clearHistory(){(this.m._wasm_sim_clear_history as Fn)(this.handle);}
}

