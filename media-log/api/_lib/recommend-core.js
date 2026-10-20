// Pure helpers for the recommendation flow (no network), kept separate for testing.

export const TYPE_LABELS = {
  anime: 'anime', tv: 'TV shows', movie: 'movies', manga: 'manga', manhwa: 'manhwa/webtoons',
  comic: 'comics', book: 'books', light_novel: 'light novels', podcast: 'podcasts',
  youtube: 'YouTube/web videos', other: 'things',
};

export const norm = (s) => String(s || '').toLowerCase().replace(/[^\p{L}\p{N}]+/gu, ' ').trim();

// Thumbs up / down are the strongest signals; dropping something is a soft negative.
export function buildTasteProfile(entries) {
  const line = (e) => `${e.title} [${e.type}${e.year ? `, ${e.year}` : ''}]${e.notes ? ` — note: ${e.notes.slice(0, 120)}` : ''}`;
  const loved = [], disliked = [], meh = [], dropped = [], inProgress = [], planned = [], unrated = [];
  for (const e of entries) {
    if (e.rating === 'up') loved.push(e);
    else if (e.rating === 'down') disliked.push(e);
    else if (e.rating === 'side') meh.push(e);
    else if (e.status === 'dropped') dropped.push(e);
    else if (e.status === 'in_progress') inProgress.push(e);
    else if (e.status === 'plan_to') planned.push(e);
    else unrated.push(e);
  }
  const section = (name, list) => (list.length ? `## ${name}\n${list.map((e) => `- ${line(e)}`).join('\n')}` : '');
  return [
    section('LOVED (thumbs up) — strongest positive signal', loved),
    section('DISLIKED (thumbs down) — strongest negative signal', disliked),
    section('MIXED (thumbs sideways)', meh),
    section('DROPPED without a rating — mild negative signal', dropped),
    section('CURRENTLY IN PROGRESS — interest, no verdict yet', inProgress),
    section('FINISHED / ON HOLD, unrated — weak positive', unrated),
    section('ALREADY PLANNED — do not recommend', planned),
  ].filter(Boolean).join('\n\n');
}

export const PICKS_SCHEMA = {
  type: 'object',
  properties: {
    picks: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          title: { type: 'string' },
          year: { type: 'integer' },
          reason: { type: 'string' },
        },
        required: ['title', 'year', 'reason'],
        additionalProperties: false,
      },
    },
  },
  required: ['picks'],
  additionalProperties: false,
};

export const SYSTEM_PROMPT = `You recommend media to one person based on their personal tracking log.
Weight their ratings heavily: lean toward the traits of things they loved, steer away from the traits of things they disliked, and treat unrated drops as a mild negative.
Only recommend real, existing titles, using the title as it is commonly known (the English title where one is widely used). Never recommend anything that already appears in their log, including their plan-to list.
Each reason is one or two sentences, specific to this person's taste (name a title from their log it relates to when that helps) rather than a generic blurb.
If the log is small or empty, lean on the mood and on broadly acclaimed titles, and say so in the reason.`;

export function buildUserPrompt({ type, mood, entries }) {
  const label = TYPE_LABELS[type] ?? 'titles';
  const profile = buildTasteProfile(entries);
  return `${profile || '(Their log is empty.)'}

---
Recommend 8 ${label} I haven't logged yet.${mood ? `\nMood / what I'm after: ${mood}` : ''}`;
}

// Drop anything already in the log (case/punctuation-insensitive), keep at most n.
export function dedupePicks(picks, entries, n = 6) {
  const seen = new Set(entries.map((e) => norm(e.title)));
  const out = [];
  for (const p of picks) {
    const key = norm(p.title);
    if (!key || seen.has(key)) continue;
    seen.add(key);
    out.push(p);
    if (out.length === n) break;
  }
  return out;
}

// Choose the provider result that best matches the recommended title.
export function bestMatch(results, pick) {
  if (!results?.length) return null;
  const want = norm(pick.title);
  return (
    results.find((r) => norm(r.title) === want && (!pick.year || !r.year || Math.abs(r.year - pick.year) <= 1)) ||
    results.find((r) => norm(r.title) === want) ||
    results[0]
  );
}
