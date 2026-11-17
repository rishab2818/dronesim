export type V3 = [number, number, number];
export type Q4 = [number, number, number, number]; // C order: w,x,y,z
export type Status = 0 | 1 | 2 | 3;
export interface Snapshot {
  time: number; step: number; position: V3; velocity: V3; quaternion: Q4;
  omega: V3; acceleration: V3; force: V3; moment: V3;
  rpm: [number, number, number, number]; thrust: [number, number, number, number];
  motorCommand: [number, number, number, number]; collective: number;
  status: Status; mode: 0 | 1; armed: boolean;
}
export interface HistoryRow {
  time: number; position: V3; quaternion: Q4; velocity: V3; omega: V3;
  rpm: number[]; thrust: number[]; commands: number[]; status: Status;
}
export interface PilotCommand { collective:number; roll:number; pitch:number; yaw:number; override:boolean; motors:number[] }
export interface BasicConfig { mass:number; ixx:number; iyy:number; izz:number; kT:number; kQ:number; maxRpm:number; motorTau:number; gravity:number; drag:number; dragEnabled:boolean }
export const DEFAULT_CONFIG: BasicConfig={mass:1,ixx:.018,iyy:.018,izz:.032,kT:1.5e-5,kQ:2.2e-7,maxRpm:10000,motorTau:.08,gravity:9.80665,drag:.12,dragEnabled:true};
export const EMPTY_SNAPSHOT: Snapshot={time:0,step:0,position:[0,0,.08],velocity:[0,0,0],quaternion:[1,0,0,0],omega:[0,0,0],acceleration:[0,0,0],force:[0,0,0],moment:[0,0,0],rpm:[0,0,0,0],thrust:[0,0,0,0],motorCommand:[0,0,0,0],collective:0,status:1,mode:0,armed:false};

