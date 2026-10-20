import { useEffect, useRef, useState } from 'react';
import { TYPES, TYPE_BY_ID, STATUSES, UNITS, RATINGS, SEARCHABLE, blankEntry } from '../lib/media.js';
import { searchMedia, mediaDetails } from '../lib/api.js';
import { supabase } from '../lib/supabase.js';
import { Thumb, Icon, ICONS } from './Icons.jsx';

// Add or edit a title. With `entry.id` set it edits; otherwise it adds.
export default function EntryForm({ entry, userId, onSave, onDelete, onClose }) {
  const editing = Boolean(entry?.id);
  const [form, setForm] = useState(() => ({ ...blankEntry(entry?.type ?? 'anime'), ...entry }));
  const [q, setQ] = useState('');
  const [results, setResults] = useState([]);
  const [searching, setSearching] = useState(false);
  const [searchError, setSearchError] = useState(null);
  const [seasons, setSeasons] = useState([]);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState(null);
  const reqId = useRef(0);

  const set = (patch) => setForm((f) => ({ ...f, ...patch }));
  const searchable = SEARCHABLE.has(form.type);

  // Debounced metadata search; stale responses are dropped.
  useEffect(() => {
    setSearchError(null);
    if (!searchable || q.trim().length < 2) { setResults([]); setSearching(false); return; }
    const id = ++reqId.current;
    setSearching(true);
    const t = setTimeout(async () => {
      try {
        const r = await searchMedia(form.type, q.trim());
        if (id === reqId.current) setResults(r);
      } catch (err) {
        if (id === reqId.current) { setResults([]); setSearchError(err.message); }
      } finally {
        if (id === reqId.current) setSearching(false);
      }
    }, 350);
    return () => clearTimeout(t);
  }, [q, form.type, searchable]);

  function changeType(type) {
    const t = TYPE_BY_ID[type];
    setResults([]); setSeasons([]);
    set({ type, unit: t.unit, season: t.season ? form.season ?? 1 : null, total: null, source: null, source_id: null });
  }

  async function pick(r) {
    set({ title: r.title, cover_url: r.coverUrl, year: r.year, total: r.total, unit: r.unit ?? form.unit, source: r.source, source_id: r.id });
    setResults([]); setQ('');
    setSeasons([]);
    if (form.type === 'tv') {
      try {
        const d = await mediaDetails('tv', r.id);
        setSeasons(d.seasons ?? []);
        const s = d.seasons?.find((x) => x.number === (form.season ?? 1)) ?? d.seasons?.[0];
        if (s) set({ season: s.number, total: s.episodes });
      } catch { /* total stays manual */ }
    }
  }

  function changeSeason(n) {
    const s = seasons.find((x) => x.number === n);
    set({ season: n, ...(s ? { total: s.episodes } : {}) });
  }

  function changeStatus(status) {
    const patch = { status };
    if (status === 'finished' && form.total && form.progress < form.total && form.unit !== 'percent') patch.progress = form.total;
    if (status === 'finished' && form.unit === 'percent') patch.progress = 100;
    set(patch);
  }

  async function upload(file) {
    if (!file) return;
    setError(null);
    const path = `${userId}/${crypto.randomUUID()}.${(file.name.split('.').pop() || 'jpg').toLowerCase()}`;
    const { error: err } = await supabase.storage.from('covers').upload(path, file, { contentType: file.type });
    if (err) return setError(`Upload failed: ${err.message}`);
    set({ cover_url: supabase.storage.from('covers').getPublicUrl(path).data.publicUrl });
  }

  async function submit(e) {
    e.preventDefault();
    if (!form.title.trim()) return setError('Give it a title.');
    setSaving(true);
    setError(null);
    try {
      await onSave({
        ...form,
        title: form.title.trim(),
        progress: Number(form.progress) || 0,
        total: form.total === '' || form.total === null ? null : Number(form.total),
        season: TYPE_BY_ID[form.type].season && form.season ? Number(form.season) : null,
        year: form.year ? Number(form.year) : null,
      });
    } catch (err) {
      setError(err.message);
      setSaving(false);
    }
  }

  return (
    <div className="overlay" role="dialog" aria-modal="true" aria-label={editing ? 'Edit title' : 'Add title'} onMouseDown={(e) => e.target === e.currentTarget && onClose()}>
      <form className="sheet" onSubmit={submit}>
        <header className="sheet-head">
          <h2>{editing ? 'Edit title' : 'Add a title'}</h2>
          <button type="button" className="icon-btn" onClick={onClose} aria-label="Close"><Icon d={ICONS.close} /></button>
        </header>

        <label>Type
          <select value={form.type} onChange={(e) => changeType(e.target.value)}>
            {TYPES.map((t) => <option key={t.id} value={t.id}>{t.label}</option>)}
          </select>
        </label>

        {searchable ? (
          <div className="finder">
            <label>Find it
              <div className="search"><Icon d={ICONS.search} size={18} />
                <input value={q} onChange={(e) => setQ(e.target.value)} placeholder={`Search ${TYPE_BY_ID[form.type].label.toLowerCase()}…`} autoFocus={!editing} />
              </div>
            </label>
            {searching && <p className="muted small">Searching…</p>}
            {searchError && <p className="error small">{searchError}</p>}
            {!searching && !searchError && q.trim().length >= 2 && !results.length && <p className="muted small">No matches. You can still fill it in by hand below.</p>}
            {results.length > 0 && (
              <ul className="results">
                {results.map((r) => (
                  <li key={`${r.source}:${r.id}`}>
                    <button type="button" onClick={() => pick(r)}>
                      {r.coverUrl ? <img src={r.coverUrl} alt="" loading="lazy" referrerPolicy="no-referrer" /> : <span className="thumb-fallback" />}
                      <span><strong>{r.title}</strong><small>{[r.year, r.total ? `${r.total} ${r.unit}${r.total > 1 ? 's' : ''}` : null].filter(Boolean).join(' · ')}</small></span>
                    </button>
                  </li>
                ))}
              </ul>
            )}
          </div>
        ) : (
          <p className="muted small">No metadata source for {TYPE_BY_ID[form.type].label.toLowerCase()} — enter the details by hand.</p>
        )}

        <div className="row-cover">
          <div className="cover preview">{form.cover_url ? <img src={form.cover_url} alt="" referrerPolicy="no-referrer" /> : <span className="cover-fallback">{(form.title || '?').slice(0, 1).toUpperCase()}</span>}</div>
          <div className="stack">
            <label>Title<input value={form.title} onChange={(e) => set({ title: e.target.value })} required maxLength={300} /></label>
            <label>Cover URL<input value={form.cover_url ?? ''} onChange={(e) => set({ cover_url: e.target.value || null })} placeholder="https://…" inputMode="url" /></label>
            <label className="file">Or upload a cover<input type="file" accept="image/*" onChange={(e) => upload(e.target.files?.[0])} /></label>
          </div>
        </div>

        <div className="grid2">
          <label>Status
            <select value={form.status} onChange={(e) => changeStatus(e.target.value)}>
              {STATUSES.map((s) => <option key={s.id} value={s.id}>{s.label}</option>)}
            </select>
          </label>
          <label>Year<input type="number" value={form.year ?? ''} onChange={(e) => set({ year: e.target.value })} min="1800" max="2100" /></label>
        </div>

        {TYPE_BY_ID[form.type].season && (
          <label>Season
            {seasons.length ? (
              <select value={form.season ?? 1} onChange={(e) => changeSeason(Number(e.target.value))}>
                {seasons.map((s) => <option key={s.number} value={s.number}>Season {s.number}{s.episodes ? ` (${s.episodes} eps)` : ''}</option>)}
              </select>
            ) : (
              <input type="number" min="1" value={form.season ?? ''} onChange={(e) => set({ season: e.target.value })} />
            )}
          </label>
        )}

        <div className="grid3">
          <label>Progress<input type="number" min="0" step="any" value={form.progress} onChange={(e) => set({ progress: e.target.value })} /></label>
          <label>Unit
            <select value={form.unit} onChange={(e) => set({ unit: e.target.value })}>
              {UNITS.map((u) => <option key={u.id} value={u.id}>{u.label}</option>)}
            </select>
          </label>
          <label>Total <small>(optional)</small><input type="number" min="1" step="any" value={form.total ?? ''} onChange={(e) => set({ total: e.target.value })} disabled={form.unit === 'percent'} /></label>
        </div>

        <fieldset className="rating">
          <legend>Rating</legend>
          {RATINGS.map((r) => (
            <button type="button" key={r.id} className={`rate rate-${r.id}${form.rating === r.id ? ' on' : ''}`} aria-pressed={form.rating === r.id} aria-label={r.label} onClick={() => set({ rating: form.rating === r.id ? null : r.id })}>
              <Thumb kind={r.id} size={22} />
            </button>
          ))}
        </fieldset>

        <label>Notes<textarea rows={3} value={form.notes} onChange={(e) => set({ notes: e.target.value })} maxLength={4000} /></label>

        {error && <p className="error" role="alert">{error}</p>}
        <footer className="sheet-foot">
          {editing && <button type="button" className="btn danger" onClick={() => confirm(`Delete “${form.title}”?`) && onDelete(entry)}>Delete</button>}
          <span className="spacer" />
          <button type="button" className="btn" onClick={onClose}>Cancel</button>
          <button className="btn primary" disabled={saving}>{saving ? 'Saving…' : 'Save'}</button>
        </footer>
      </form>
    </div>
  );
}
