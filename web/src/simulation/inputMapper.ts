export type ControlKey='rollLeft'|'rollRight'|'pitchForward'|'pitchBack'|'yawLeft'|'yawRight'|'throttleUp'|'throttleDown';
/* T/G and arrow keys avoid Windows Sticky Keys, which can be triggered by
   repeated Shift presses. Modifiers are intentionally not flight controls. */
const map:Record<string,ControlKey>={KeyA:'rollLeft',KeyD:'rollRight',KeyW:'pitchForward',KeyS:'pitchBack',KeyQ:'yawLeft',KeyE:'yawRight',KeyT:'throttleUp',ArrowUp:'throttleUp',PageUp:'throttleUp',KeyG:'throttleDown',ArrowDown:'throttleDown',PageDown:'throttleDown'};
export const isEditable=(target:EventTarget|null)=>target instanceof HTMLElement&&(target.isContentEditable||['INPUT','TEXTAREA','SELECT','BUTTON'].includes(target.tagName));
export const controlForCode=(code:string)=>map[code];
export class PressedControls { private keys=new Set<ControlKey>(); set(key:ControlKey,down:boolean){down?this.keys.add(key):this.keys.delete(key);} clear(){this.keys.clear();} has(key:ControlKey){return this.keys.has(key);} axes(){return {roll:(this.has('rollRight')?1:0)-(this.has('rollLeft')?1:0),pitch:(this.has('pitchForward')?1:0)-(this.has('pitchBack')?1:0),yaw:(this.has('yawRight')?1:0)-(this.has('yawLeft')?1:0),throttle:(this.has('throttleUp')?1:0)-(this.has('throttleDown')?1:0)};}}

