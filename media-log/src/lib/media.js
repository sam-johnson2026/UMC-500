export const TYPES = [
  { id: 'anime', label: 'Anime', unit: 'episode', season: true },
  { id: 'tv', label: 'TV', unit: 'episode', season: true },
  { id: 'movie', label: 'Movies', unit: 'percent' },
  { id: 'manga', label: 'Manga', unit: 'chapter' },
  { id: 'manhwa', label: 'Manhwa / Webtoons', unit: 'chapter' },
  { id: 'comic', label: 'Comics', unit: 'issue' },
  { id: 'book', label: 'Books', unit: 'page' },
  { id: 'light_novel', label: 'Light Novels', unit: 'volume' },
  { id: 'podcast', label: 'Podcasts', unit: 'episode' },
  { id: 'game', label: 'Games', unit: 'percent' },
  { id: 'youtube', label: 'YouTube / Web', unit: 'percent' },
  { id: 'other', label: 'Other', unit: 'percent' },
];
export const TYPE_BY_ID = Object.fromEntries(TYPES.map((t) => [t.id, t]));

// Types that have a metadata search behind /api/search.
export const SEARCHABLE = new Set(['anime', 'manga', 'light_novel', 'tv', 'movie', 'book', 'game', 'manhwa']);

export const STATUSES = [
  { id: 'in_progress', label: 'In progress' },
  { id: 'plan_to', label: 'Plan to' },
  { id: 'finished', label: 'Finished' },
  { id: 'on_hold', label: 'On hold' },
  { id: 'dropped', label: 'Dropped' },
];
export const STATUS_LABEL = Object.fromEntries(STATUSES.map((s) => [s.id, s.label]));

export const UNITS = [
  { id: 'episode', label: 'Episode', short: 'Ep' },
  { id: 'chapter', label: 'Chapter', short: 'Ch' },
  { id: 'issue', label: 'Issue', short: '#' },
  { id: 'volume', label: 'Volume', short: 'Vol' },
  { id: 'page', label: 'Page', short: 'p.' },
  { id: 'percent', label: 'Percent', short: '%' },
];
export const UNIT_BY_ID = Object.fromEntries(UNITS.map((u) => [u.id, u]));

export const RATINGS = [
  { id: 'up', label: 'Thumbs up' },
  { id: 'side', label: 'Thumbs sideways' },
  { id: 'down', label: 'Thumbs down' },
];

const num = (n) => (Number.isInteger(Number(n)) ? String(Number(n)) : Number(n).toFixed(1));

// "S2 · Ep 6 / 12", "Ch 120", "p. 88 / 412", "45%"
export function formatProgress(e) {
  const parts = [];
  if (e.season) parts.push(`S${e.season}`);
  const total = e.total ? num(e.total) : null;
  if (e.unit === 'percent') parts.push(`${num(e.progress)}%`);
  else {
    const short = UNIT_BY_ID[e.unit]?.short ?? '';
    const lead = short === '#' ? '#' : `${short} `;
    parts.push(`${lead}${num(e.progress)}${total ? ` / ${total}` : ''}`);
  }
  return parts.join(' · ');
}

export function progressFraction(e) {
  if (e.unit === 'percent') return Math.min(1, Number(e.progress) / 100);
  if (!e.total) return null;
  return Math.min(1, Number(e.progress) / Number(e.total));
}

export const blankEntry = (type = 'anime') => ({
  type,
  title: '',
  status: 'plan_to',
  season: TYPE_BY_ID[type].season ? 1 : null,
  progress: 0,
  unit: TYPE_BY_ID[type].unit,
  total: null,
  rating: null,
  notes: '',
  cover_url: null,
  year: null,
  source: null,
  source_id: null,
});
