import { useSimulationStore } from '../store/useSimulationStore';
export function EventLog(){const events=useSimulationStore(s=>s.events);return <section className="panel log"><h2>Event log</h2><ol>{events.map((e,i)=><li key={`${e}-${i}`}>{e}</li>)}</ol></section>}

