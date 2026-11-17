import { create } from 'zustand';
import { EMPTY_SNAPSHOT, type HistoryRow, type Snapshot } from '../types';

interface SimStore { snapshot:Snapshot; history:HistoryRow[]; events:string[]; slow:boolean; setSnapshot:(s:Snapshot)=>void; setHistory:(h:HistoryRow[])=>void; log:(m:string)=>void; clearEvents:()=>void; setSlow:(v:boolean)=>void }
export const useSimulationStore=create<SimStore>((set)=>({snapshot:EMPTY_SNAPSHOT,history:[],events:['Ready — load the C/Wasm engine to begin.'],slow:false,setSnapshot:(snapshot)=>set({snapshot}),setHistory:(history)=>set({history}),log:(m)=>set(s=>({events:[`${new Date().toLocaleTimeString()}  ${m}`,...s.events].slice(0,30)})),clearEvents:()=>set({events:[]}),setSlow:(slow)=>set({slow})}));

