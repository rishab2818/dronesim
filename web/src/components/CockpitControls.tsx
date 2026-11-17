import { useSimulationStore } from '../store/useSimulationStore';
import type { PilotCommand } from '../types';

interface Props {
  command:PilotCommand; setCommand:(p:PilotCommand)=>void; hoverCollective:number;
  running:boolean; armed:boolean; mode:0|1; onRun:()=>void; onArm:()=>void;
  onReset:()=>void; onClear:()=>void; onMode:(m:0|1)=>void;
  onPreset:(name:string)=>void; onExport:()=>void; onExportScenario:()=>void;
  onImportScenario:(text:string)=>void;
}

const clamp=(value:number)=>Math.max(0,Math.min(1,value));

export function CockpitControls(p:Props){
  const truth=useSimulationStore(s=>s.snapshot),netZ=truth.force[2],vz=truth.velocity[2];
  const setCollective=(value:number)=>p.setCommand({...p.command,collective:clamp(value)});
  const setAxis=(axis:'roll'|'pitch'|'yaw',value:number)=>()=>p.setCommand({...p.command,[axis]:value});
  const brakeTarget=clamp(p.hoverCollective+(vz>0.15?-.03:vz<-.15?.03:0));
  const forceLabel=netZ>.15?'accelerating up':netZ<-.15?'accelerating down':'force balanced';
  return <section className="cockpit-card pilot-card">
    <div className="pilot-actions"><button className="primary" onClick={p.onRun}>{p.running?'Pause':'Start'}</button><button className={p.armed?'armed':''} onClick={p.onArm}>{p.armed?'Disarm':'Arm'}</button><button onClick={p.onReset}>Reset</button></div>
    <div className="mode-row"><span>Flight mode</span><select value={p.mode} onChange={e=>p.onMode(Number(e.target.value) as 0|1)}><option value={1}>Beginner · smooth speed limits</option><option value={0}>Manual physics · advanced</option></select></div>
    {p.mode===1&&<div className="rate-limits"><b>Dynamic limits</b><span>XY 2.5 m/s</span><span>up 2.0</span><span>down 1.5</span><span>yaw 0.8 rad/s</span></div>}
    <div className="collective-head"><span>COLLECTIVE</span><strong>{(p.command.collective*100).toFixed(1)}%</strong></div>
    <input className="collective-slider" aria-label="Collective throttle" type="range" min="0" max="1" step=".001" value={p.command.collective} onChange={e=>setCollective(Number(e.target.value))}/>
    <div className="trim-grid"><button onClick={()=>setCollective(p.command.collective-.05)}>−5%</button><button onClick={()=>setCollective(p.command.collective-.01)}>−1%</button><button className="hover-button" onClick={()=>setCollective(p.hoverCollective)}>Hover<br/><small>{(p.hoverCollective*100).toFixed(1)}%</small></button><button onClick={()=>setCollective(p.command.collective+.01)}>+1%</button><button onClick={()=>setCollective(p.command.collective+.05)}>+5%</button></div>
    <button className="brake-button" onClick={()=>setCollective(brakeTarget)} disabled={Math.abs(vz)<.15}>{vz>.15?'Brake climb':vz<-.15?'Brake descent':'Vertical speed settled'}<span>{vz>.15?'Sets 3% below hover':vz<-.15?'Sets 3% above hover':'Use Hover to maintain'}</span></button>
    <div className={`motion-guide ${netZ>.15?'up':netZ<-.15?'down':'balanced'}`}><div><span>Vertical speed</span><b>{vz>=0?'+':''}{vz.toFixed(2)} m/s</b></div><div><span>Net vertical force</span><b>{netZ>=0?'+':''}{netZ.toFixed(2)} N</b></div><small>{forceLabel}. Force changes speed; hover does not erase existing motion.</small></div>
    <div className="flight-pad" aria-label="Attitude controls"><span></span><button onPointerDown={setAxis('pitch',1)} onPointerUp={setAxis('pitch',0)}>W<small>forward</small></button><span></span><button onPointerDown={setAxis('roll',-1)} onPointerUp={setAxis('roll',0)}>A<small>left</small></button><button onPointerDown={setAxis('pitch',-1)} onPointerUp={setAxis('pitch',0)}>S<small>back</small></button><button onPointerDown={setAxis('roll',1)} onPointerUp={setAxis('roll',0)}>D<small>right</small></button><button onPointerDown={setAxis('yaw',-1)} onPointerUp={setAxis('yaw',0)}>Q<small>yaw L</small></button><span></span><button onPointerDown={setAxis('yaw',1)} onPointerUp={setAxis('yaw',0)}>E<small>yaw R</small></button></div>
    <div className="keyboard-strip"><b>T / ↑</b> throttle up <i>·</i> <b>G / ↓</b> throttle down <i>·</i> <b>Space</b> pause</div>
    <label className="preset-row">Training flight<select defaultValue="" onChange={e=>{p.onPreset(e.target.value);e.currentTarget.value='';}}><option value="" disabled>Choose a guided demo…</option><option value="hover">Hover balance</option><option value="low">Too little thrust</option><option value="climb">Gentle vertical climb</option><option value="pitch">Pitch forward pulse</option><option value="yaw">Yaw pulse</option><option value="cut">Cut motors</option><option value="crash">Crash test</option></select></label>
    <details className="utility-menu"><summary>Data, trail and scenarios</summary><div className="utility-buttons"><button onClick={p.onClear}>Clear trail</button><button onClick={p.onExport}>Export CSV</button><button onClick={p.onExportScenario}>Save scenario</button><label className="file-button">Load scenario<input type="file" accept="application/json,.json" onChange={async e=>{const file=e.target.files?.[0];if(file)await p.onImportScenario(await file.text());}}/></label></div></details>
  </section>;
}

export function QuickTelemetry(){
  const s=useSimulationStore(x=>x.snapshot),speed=Math.hypot(...s.velocity),avgRpm=s.rpm.reduce((a,b)=>a+b,0)/4;
  const status=['Stopped','Grounded','Flying','Crashed'][s.status];
  return <section className="cockpit-card quick-telemetry"><div className="telemetry-title"><span>SIMULATION TRUTH</span><b className={`flight-state state-${s.status}`}>{status}</b></div><div className="altitude-readout"><span>Altitude</span><strong>{s.position[2].toFixed(2)}</strong><em>m</em></div><div className="quick-grid"><div><span>Vertical speed</span><b>{s.velocity[2].toFixed(2)} m/s</b></div><div><span>Total speed</span><b>{speed.toFixed(2)} m/s</b></div><div><span>Average motor</span><b>{avgRpm.toFixed(0)} RPM</b></div><div><span>Simulation time</span><b>{s.time.toFixed(2)} s</b></div></div></section>;
}
