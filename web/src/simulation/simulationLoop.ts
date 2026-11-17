export interface LoopOptions { fixedDt:number; maxStepsPerFrame:number; onStep:(dt:number)=>void; onFrame:()=>void; onSlow:(slow:boolean)=>void }
export class SimulationLoop {
  private running=false;private accumulator=0;private last=0;private raf=0;
  constructor(private options:LoopOptions){}
  start(){if(this.running)return;this.running=true;this.last=performance.now();this.raf=requestAnimationFrame(this.tick);}
  pause(){this.running=false;cancelAnimationFrame(this.raf);this.accumulator=0;}
  isRunning(){return this.running;}
  private tick=(now:number)=>{if(!this.running)return;this.accumulator+=Math.min((now-this.last)/1000,.1);this.last=now;let steps=0;while(this.accumulator>=this.options.fixedDt&&steps<this.options.maxStepsPerFrame){this.options.onStep(this.options.fixedDt);this.accumulator-=this.options.fixedDt;steps++;}const slow=this.accumulator>=this.options.fixedDt;if(slow)this.accumulator=0;this.options.onSlow(slow);this.options.onFrame();this.raf=requestAnimationFrame(this.tick);};
}

