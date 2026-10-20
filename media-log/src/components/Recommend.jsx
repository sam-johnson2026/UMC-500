import { useState } from 'react';
import { TYPES } from '../lib/media.js';
import { recommend } from '../lib/api.js';
import { Icon, ICONS } from './Icons.jsx';

const MOODS = ['cozy', 'mind-bending', 'short & binge-able', 'dark and gritty', 'feel-good', 'slow burn'];

export default function Recommend({ onAddPlan }) {
  const [type, setType] = useState('anime');
  const [mood, setMood] = useState('');
  const [picks, setPicks] = useState(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState(null);
  const [added, setAdded] = useState({});

  async function go(e) {
    e.preventDefault();
    setBusy(true); setError(null); setAdded({});
    try { setPicks(await recommend(type, mood.trim())); }
    catch (err) { setError(err.message); setPicks(null); }
    finally { setBusy(false); }
  }

  async function add(p, i) {
    setAdded((a) => ({ ...a, [i]: 'saving' }));
    try { await onAddPlan(type, p); setAdded((a) => ({ ...a, [i]: 'done' })); }
    catch (err) { setError(err.message); setAdded((a) => ({ ...a, [i]: undefined })); }
  }

  return (
    <section>
      <form className="panel rec-form" onSubmit={go}>
        <h2>Recommend</h2>
        <p className="muted">Based on your log. Thumbs up and down count the most.</p>
        <label>What are you in the mood for?
          <select value={type} onChange={(e) => setType(e.target.value)}>{TYPES.map((t) => <option key={t.id} value={t.id}>{t.label}</option>)}</select>
        </label>
        <label>Mood <small>(optional)</small>
          <input value={mood} onChange={(e) => setMood(e.target.value)} maxLength={300} placeholder="e.g. something cozy after a rough week" />
        </label>
        <div className="chips">{MOODS.map((m) => <button type="button" key={m} className={`chip${mood === m ? ' on' : ''}`} onClick={() => setMood(mood === m ? '' : m)}>{m}</button>)}</div>
        <button className="btn primary" disabled={busy}><Icon d={ICONS.spark} size={18} /> {busy ? 'Thinking…' : 'Get 6 picks'}</button>
      </form>

      {error && <p className="error" role="alert">{error}</p>}

      {picks && (
        <ol className="picks">
          {picks.map((p, i) => (
            <li key={`${p.title}-${i}`} className="pick">
              <div className="cover">{p.coverUrl ? <img src={p.coverUrl} alt="" loading="lazy" referrerPolicy="no-referrer" /> : <span className="cover-fallback">{p.title.slice(0, 1)}</span>}</div>
              <div className="pick-body">
                <h3>{p.title} {p.year ? <small>({p.year})</small> : null}</h3>
                <p>{p.reason}</p>
                <button className="btn" disabled={Boolean(added[i])} onClick={() => add(p, i)}>
                  {added[i] === 'done' ? 'Added to Plan to ✓' : added[i] === 'saving' ? 'Adding…' : 'Add to Plan to'}
                </button>
              </div>
            </li>
          ))}
          {!picks.length && <p className="muted">No new picks came back. Try a different mood.</p>}
        </ol>
      )}
    </section>
  );
}
