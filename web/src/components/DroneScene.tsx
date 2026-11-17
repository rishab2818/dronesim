import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Grid, Line, OrbitControls, Text } from '@react-three/drei';
import { useEffect, useMemo, useRef, useState, type RefObject } from 'react';
import * as THREE from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import { useSimulationStore } from '../store/useSimulationStore';

function Drone(){
  const s=useSimulationStore(x=>x.snapshot),g=useRef<THREE.Group>(null),rotors=useRef<THREE.Mesh[]>([]);
  const displayQ=useMemo(()=>new THREE.Quaternion(),[]),basis=useMemo(()=>new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2),[]);
  useFrame((_,dt)=>{if(!g.current)return;g.current.position.set(s.position[0],s.position[2],-s.position[1]);const q=new THREE.Quaternion(s.quaternion[1],s.quaternion[2],s.quaternion[3],s.quaternion[0]);displayQ.copy(basis).multiply(q).multiply(basis.clone().invert());g.current.quaternion.copy(displayQ);rotors.current.forEach((r,i)=>r.rotation.y+=dt*s.rpm[i]*Math.PI/30);});
  const points:[[number,number,number],string,string][]=[[[.159,.02,-.159],'M1','↺'],[[.159,.02,.159],'M2','↻'],[[-.159,.02,.159],'M3','↺'],[[-.159,.02,-.159],'M4','↻']];
  return <group ref={g}><mesh><boxGeometry args={[.12,.07,.16]}/><meshStandardMaterial color={s.status===3?'#ef4444':s.armed?'#f59e0b':'#64748b'} metalness={.35} roughness={.55}/></mesh>
    <mesh rotation={[0,Math.PI/4,0]}><boxGeometry args={[.5,.025,.035]}/><meshStandardMaterial color="#b8c4d6"/></mesh><mesh rotation={[0,-Math.PI/4,0]}><boxGeometry args={[.5,.025,.035]}/><meshStandardMaterial color="#b8c4d6"/></mesh>
    {points.map(([p,label,spin],i)=><group position={p} key={label}><mesh ref={el=>{if(el)rotors.current[i]=el}}><cylinderGeometry args={[.11,.11,.008,32]}/><meshStandardMaterial color={i%2?'#60a5fa':'#22d3a6'} transparent opacity={.72}/></mesh><Text position={[0,.055,0]} fontSize={.035} color="white" anchorY="bottom">{label} {spin}</Text></group>)}
    <mesh position={[0,.01,-.12]} rotation={[Math.PI/2,0,0]}><coneGeometry args={[.035,.10,4]}/><meshStandardMaterial color="#f8fafc"/></mesh></group>;
}
function Trail(){const h=useSimulationStore(s=>s.history);const points=useMemo(()=>h.map(x=>[x.position[0],x.position[2],-x.position[1]] as [number,number,number]),[h]);return points.length>1?<Line points={points} color="#34d399" lineWidth={2}/>:null;}

function CameraFollower({enabled,resetToken,controlsRef}:{enabled:boolean;resetToken:number;controlsRef:RefObject<OrbitControlsImpl>}){
  const camera=useThree(s=>s.camera),lastTarget=useRef(new THREE.Vector3());
  const currentTarget=()=>{const p=useSimulationStore.getState().snapshot.position;return new THREE.Vector3(p[0],p[2],-p[1]);};
  const recenter=()=>{const target=currentTarget();camera.position.copy(target).add(new THREE.Vector3(2.6,1.8,3.2));controlsRef.current?.target.copy(target);controlsRef.current?.update();lastTarget.current.copy(target);};
  useEffect(()=>{recenter();},[enabled,resetToken]); // eslint-disable-line react-hooks/exhaustive-deps
  useFrame(()=>{const target=currentTarget();if(enabled){const delta=target.clone().sub(lastTarget.current);camera.position.add(delta);controlsRef.current?.target.copy(target);controlsRef.current?.update();}lastTarget.current.copy(target);});
  return null;
}

export function DroneScene(){
  const [follow,setFollow]=useState(true),[resetToken,setResetToken]=useState(0),controls=useRef<OrbitControlsImpl>(null);
  return <div className="viewport"><Canvas camera={{position:[2.6,1.8,3.2],fov:48}}><color attach="background" args={['#08101d']}/><ambientLight intensity={.65}/><directionalLight position={[3,5,2]} intensity={1.5}/><Grid args={[20,20]} cellColor="#27415e" sectionColor="#3b82f6" fadeDistance={18} infiniteGrid/><axesHelper args={[1]}/><Drone/><Trail/><OrbitControls ref={controls} makeDefault target={[0,.3,0]}/><CameraFollower enabled={follow} resetToken={resetToken} controlsRef={controls}/></Canvas><div className="viewport-tools"><label><input type="checkbox" checked={follow} onChange={e=>setFollow(e.target.checked)}/> Follow drone</label><button onClick={()=>setResetToken(x=>x+1)}>Recenter view</button></div><div className="axis-legend">World: <b>+X</b> east · <b>+Y</b> north · <b>+Z</b> up · drag to orbit · wheel to zoom</div></div>;
}

