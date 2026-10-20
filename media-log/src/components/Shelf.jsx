import { useMemo, useState } from 'react';
import { TYPES, STATUSES } from '../lib/media.js';
import Card from './Card.jsx';
import { Icon, ICONS } from './Icons.jsx';

export default function Shelf({ entries, onOpen, onIncrement, onAdd }) {
  const [status, setStatus] = useState('all');
  const [query, setQuery] = useState('');

  const sections = useMemo(() => {
    const q = query.trim().toLowerCase();
    const visible = entries.filter((e) => (status === 'all' || e.status === status) && (!q || e.title.toLowerCase().includes(q) || e.notes.toLowerCase().includes(q)));
    return TYPES.map((t) => ({ ...t, items: visible.filter((e) => e.type === t.id) })).filter((s) => s.items.length);
  }, [entries, status, query]);

  const counts = useMemo(() => {
    const c = { all: entries.length };
    for (const s of STATUSES) c[s.id] = entries.filter((e) => e.status === s.id).length;
    return c;
  }, [entries]);

  return (
    <section>
      <div className="toolbar">
        <label className="search">
          <Icon d={ICONS.search} size={18} />
          <input type="search" placeholder="Search your shelf" value={query} onChange={(e) => setQuery(e.target.value)} aria-label="Search your shelf" />
        </label>
        <div className="chips" role="tablist" aria-label="Filter by status">
          {[{ id: 'all', label: 'All' }, ...STATUSES].map((s) => (
            <button key={s.id} role="tab" aria-selected={status === s.id} className={`chip${status === s.id ? ' on' : ''}`} onClick={() => setStatus(s.id)}>
              {s.label} <span className="count">{counts[s.id]}</span>
            </button>
          ))}
        </div>
      </div>

      {sections.map((s) => (
        <div key={s.id} className="shelf-section">
          <h2>{s.label} <span className="count">{s.items.length}</span></h2>
          <div className="grid">
            {s.items.map((e) => <Card key={e.id} entry={e} onOpen={onOpen} onIncrement={onIncrement} />)}
          </div>
        </div>
      ))}

      {!sections.length && (
        <div className="empty">
          {entries.length ? <p>Nothing matches. Try another filter.</p> : <><p>Your shelf is empty.</p><button className="btn primary" onClick={onAdd}>Add your first title</button></>}
        </div>
      )}
    </section>
  );
}
