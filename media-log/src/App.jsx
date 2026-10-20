import { useCallback, useEffect, useState } from 'react';
import { configured, supabase } from './lib/supabase.js';
import { TYPE_BY_ID, blankEntry } from './lib/media.js';
import Auth from './components/Auth.jsx';
import Shelf from './components/Shelf.jsx';
import EntryForm from './components/EntryForm.jsx';
import Recommend from './components/Recommend.jsx';
import { Icon, ICONS } from './components/Icons.jsx';

// Columns the form is allowed to write.
const FIELDS = ['type', 'title', 'status', 'season', 'progress', 'unit', 'total', 'rating', 'notes', 'cover_url', 'year', 'source', 'source_id'];
const pickFields = (e) => Object.fromEntries(FIELDS.map((k) => [k, e[k] ?? null]));

function useTheme() {
  const [theme, setTheme] = useState(() => { try { return localStorage.getItem('theme'); } catch { return null; } });
  useEffect(() => {
    if (theme) document.documentElement.dataset.theme = theme; else delete document.documentElement.dataset.theme;
    try { theme ? localStorage.setItem('theme', theme) : localStorage.removeItem('theme'); } catch { /* storage unavailable */ }
  }, [theme]);
  const isDark = theme ? theme === 'dark' : window.matchMedia('(prefers-color-scheme: dark)').matches;
  return [isDark, () => setTheme(isDark ? 'light' : 'dark')];
}

function Setup() {
  return (
    <main className="auth"><div className="panel auth-card">
      <h1>Almost there</h1>
      <p>Set <code>VITE_SUPABASE_URL</code> and <code>VITE_SUPABASE_ANON_KEY</code> (see <code>.env.example</code>) and restart the dev server.</p>
    </div></main>
  );
}

export default function App() {
  const [session, setSession] = useState(undefined);
  const [tab, setTab] = useState('shelf');
  const [entries, setEntries] = useState([]);
  const [loadError, setLoadError] = useState(null);
  const [editing, setEditing] = useState(null);
  const [toast, setToast] = useState(null);
  const [isDark, toggleTheme] = useTheme();

  useEffect(() => {
    if (!configured) return;
    supabase.auth.getSession().then(({ data }) => setSession(data.session));
    const { data } = supabase.auth.onAuthStateChange((_e, s) => setSession(s));
    return () => data.subscription.unsubscribe();
  }, []);

  const userId = session?.user?.id;
  useEffect(() => {
    if (!userId) { setEntries([]); return; }
    supabase.from('entries').select('*').order('updated_at', { ascending: false }).then(({ data, error }) => {
      if (error) setLoadError(error.message); else { setLoadError(null); setEntries(data); }
    });
  }, [userId]);

  const flash = useCallback((text) => { setToast(text); setTimeout(() => setToast(null), 3500); }, []);

  async function save(form) {
    const row = pickFields(form);
    if (form.id) {
      const { data, error } = await supabase.from('entries').update(row).eq('id', form.id).select().single();
      if (error) throw error;
      setEntries((es) => es.map((e) => (e.id === data.id ? data : e)));
    } else {
      const { data, error } = await supabase.from('entries').insert(row).select().single();
      if (error) throw error;
      setEntries((es) => [data, ...es]);
    }
    setEditing(null);
  }

  async function remove(entry) {
    const { error } = await supabase.from('entries').delete().eq('id', entry.id);
    if (error) return flash(error.message);
    setEntries((es) => es.filter((e) => e.id !== entry.id));
    setEditing(null);
  }

  // Optimistic +1; reverts if the write fails. Hitting the total marks it finished.
  async function increment(entry) {
    const progress = Number(entry.progress) + 1;
    const done = entry.total && progress >= entry.total;
    const patch = { progress, ...(done ? { status: 'finished' } : {}) };
    setEntries((es) => es.map((e) => (e.id === entry.id ? { ...e, ...patch } : e)));
    const { data, error } = await supabase.from('entries').update(patch).eq('id', entry.id).select().single();
    if (error) {
      setEntries((es) => es.map((e) => (e.id === entry.id ? entry : e)));
      flash(`Couldn't save: ${error.message}`);
    } else setEntries((es) => es.map((e) => (e.id === data.id ? data : e)));
  }

  async function addPlan(type, p) {
    const base = blankEntry(type);
    await save({ ...base, title: p.title, year: p.year, cover_url: p.coverUrl, total: p.total, unit: p.unit ?? base.unit, source: p.source, source_id: p.sourceId, status: 'plan_to', season: TYPE_BY_ID[type].season ? 1 : null });
    setEditing(null);
  }

  if (!configured) return <Setup />;
  if (session === undefined) return <div className="boot" aria-busy="true">Loading…</div>;
  if (!session) return <Auth />;

  return (
    <>
      <header className="topbar">
        <h1>Media Log</h1>
        <nav className="tabs" aria-label="Sections">
          <button className={tab === 'shelf' ? 'on' : ''} onClick={() => setTab('shelf')}><Icon d={ICONS.shelf} size={18} /> <span className="tab-label">Shelf</span></button>
          <button className={tab === 'rec' ? 'on' : ''} onClick={() => setTab('rec')}><Icon d={ICONS.spark} size={18} /> <span className="tab-label">Recommend</span></button>
        </nav>
        <span className="spacer" />
        <button className="icon-btn" onClick={toggleTheme} aria-label={isDark ? 'Switch to light mode' : 'Switch to dark mode'}><Icon d={isDark ? ICONS.sun : ICONS.moon} /></button>
        <button className="btn small" onClick={() => supabase.auth.signOut()}>Sign out</button>
      </header>

      <main className="page">
        {loadError && <p className="error" role="alert">Couldn't load your log: {loadError}. Did you run supabase/schema.sql?</p>}
        {tab === 'shelf'
          ? <Shelf entries={entries} onOpen={setEditing} onIncrement={increment} onAdd={() => setEditing(blankEntry())} />
          : <Recommend onAddPlan={addPlan} />}
      </main>

      {tab === 'shelf' && <button className="fab" onClick={() => setEditing(blankEntry())} aria-label="Add a title"><Icon d={ICONS.plus} size={26} /></button>}
      {editing && <EntryForm key={editing.id ?? 'new'} entry={editing} userId={userId} onSave={save} onDelete={remove} onClose={() => setEditing(null)} />}
      {toast && <div className="toast" role="status">{toast}</div>}
    </>
  );
}
