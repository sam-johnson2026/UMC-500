// Metadata search adapters. Every adapter returns the same shape:
//   { id, source, title, year, coverUrl, total, unit, overview }
// `fetchImpl` and `env` are injectable so the adapters can be tested offline.

const UA = 'MediaLog/0.1 (personal media tracker)';

const clean = (v) => (v === undefined || v === null || v === '' ? null : v);
const asInt = (v) => {
  const n = parseInt(v, 10);
  return Number.isFinite(n) && n > 0 ? n : null;
};

async function getJson(fetchImpl, url, init = {}) {
  const res = await fetchImpl(url, { ...init, headers: { 'User-Agent': UA, Accept: 'application/json', ...init.headers } });
  if (!res.ok) throw new Error(`${new URL(url).host} responded ${res.status}`);
  return res.json();
}

/* ---------------- AniList: anime, manga, light novels ---------------- */

const ANILIST_QUERY = `
query ($q: String, $type: MediaType, $formats: [MediaFormat]) {
  Page(perPage: 12) {
    media(search: $q, type: $type, format_in: $formats, sort: SEARCH_MATCH) {
      id
      title { romaji english }
      seasonYear
      startDate { year }
      episodes
      chapters
      volumes
      description(asHtml: false)
      coverImage { large medium }
    }
  }
}`;

const ANILIST_SCOPES = {
  anime: { type: 'ANIME', formats: ['TV', 'TV_SHORT', 'MOVIE', 'SPECIAL', 'OVA', 'ONA'], unit: 'episode', total: (m) => m.episodes },
  manga: { type: 'MANGA', formats: ['MANGA', 'ONE_SHOT'], unit: 'chapter', total: (m) => m.chapters },
  light_novel: { type: 'MANGA', formats: ['NOVEL'], unit: 'volume', total: (m) => m.volumes },
};

async function anilist(type, q, { fetchImpl }) {
  const scope = ANILIST_SCOPES[type];
  const data = await getJson(fetchImpl, 'https://graphql.anilist.co', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ query: ANILIST_QUERY, variables: { q, type: scope.type, formats: scope.formats } }),
  });
  if (data.errors?.length) throw new Error(`AniList: ${data.errors[0].message}`);
  return (data.data?.Page?.media ?? []).map((m) => ({
    id: String(m.id),
    source: 'anilist',
    title: m.title.english || m.title.romaji,
    year: m.seasonYear ?? m.startDate?.year ?? null,
    coverUrl: m.coverImage?.large ?? m.coverImage?.medium ?? null,
    total: asInt(scope.total(m)),
    unit: scope.unit,
    overview: clean(m.description?.replace(/<[^>]+>/g, '').slice(0, 240)),
  }));
}

/* ---------------- TMDB: movies and TV ---------------- */

function tmdbRequest(path, params, env) {
  const key = env.TMDB_API_KEY;
  if (!key) throw new Error('TMDB_API_KEY is not configured');
  const url = new URL(`https://api.themoviedb.org/3${path}`);
  Object.entries(params).forEach(([k, v]) => url.searchParams.set(k, v));
  const init = {};
  // v4 read-access tokens are JWTs (long); v3 keys are 32 hex chars.
  if (key.length > 40) init.headers = { Authorization: `Bearer ${key}` };
  else url.searchParams.set('api_key', key);
  return [url.toString(), init];
}

async function tmdb(type, q, { fetchImpl, env }) {
  const kind = type === 'movie' ? 'movie' : 'tv';
  const [url, init] = tmdbRequest(`/search/${kind}`, { query: q, include_adult: 'false' }, env);
  const data = await getJson(fetchImpl, url, init);
  return (data.results ?? []).slice(0, 12).map((r) => ({
    id: String(r.id),
    source: 'tmdb',
    title: r.title || r.name,
    year: asInt((r.release_date || r.first_air_date || '').slice(0, 4)),
    coverUrl: r.poster_path ? `https://image.tmdb.org/t/p/w342${r.poster_path}` : null,
    total: null, // filled in by details() for TV once a season is known
    unit: kind === 'tv' ? 'episode' : 'percent',
    overview: clean(r.overview?.slice(0, 240)),
  }));
}

// Per-season episode counts, so the form can fill the total when a season is chosen.
async function tmdbDetails(id, { fetchImpl, env }) {
  const [url, init] = tmdbRequest(`/tv/${encodeURIComponent(id)}`, {}, env);
  const d = await getJson(fetchImpl, url, init);
  return {
    seasons: (d.seasons ?? [])
      .filter((s) => s.season_number > 0)
      .map((s) => ({ number: s.season_number, episodes: asInt(s.episode_count) })),
  };
}

/* ---------------- Open Library: books ---------------- */

async function openLibrary(_type, q, { fetchImpl }) {
  const url = new URL('https://openlibrary.org/search.json');
  url.searchParams.set('q', q);
  url.searchParams.set('limit', '12');
  url.searchParams.set('fields', 'key,title,author_name,first_publish_year,cover_i,number_of_pages_median');
  const data = await getJson(fetchImpl, url.toString());
  return (data.docs ?? []).map((d) => ({
    id: d.key,
    source: 'openlibrary',
    title: d.title,
    year: d.first_publish_year ?? null,
    coverUrl: d.cover_i ? `https://covers.openlibrary.org/b/id/${d.cover_i}-M.jpg` : null,
    total: asInt(d.number_of_pages_median),
    unit: 'page',
    overview: d.author_name?.length ? `by ${d.author_name.slice(0, 3).join(', ')}` : null,
  }));
}

/* ---------------- MangaDex: manhwa / webtoons ---------------- */

async function mangadex(_type, q, { fetchImpl }) {
  const url = new URL('https://api.mangadex.org/manga');
  url.searchParams.set('title', q);
  url.searchParams.set('limit', '12');
  url.searchParams.append('includes[]', 'cover_art');
  url.searchParams.append('originalLanguage[]', 'ko');
  ['safe', 'suggestive', 'erotica'].forEach((r) => url.searchParams.append('contentRating[]', r));
  url.searchParams.set('order[relevance]', 'desc');
  const data = await getJson(fetchImpl, url.toString());
  return (data.data ?? []).map((m) => {
    const a = m.attributes ?? {};
    const title = a.title?.en || Object.values(a.title ?? {})[0] || (a.altTitles ?? []).map((t) => t.en).find(Boolean) || 'Untitled';
    const cover = m.relationships?.find((r) => r.type === 'cover_art')?.attributes?.fileName;
    return {
      id: m.id,
      source: 'mangadex',
      title,
      year: a.year ?? null,
      coverUrl: cover ? `https://uploads.mangadex.org/covers/${m.id}/${cover}.256.jpg` : null,
      total: asInt(a.lastChapter),
      unit: 'chapter',
      overview: clean(a.description?.en?.slice(0, 240)),
    };
  });
}

/* ---------------- registry ---------------- */

const BY_TYPE = {
  anime: anilist,
  manga: anilist,
  light_novel: anilist,
  movie: tmdb,
  tv: tmdb,
  book: openLibrary,
  manhwa: mangadex,
};

export const hasSearch = (type) => type in BY_TYPE;

export async function search(type, q, { fetchImpl = fetch, env = process.env } = {}) {
  const fn = BY_TYPE[type];
  if (!fn) return [];
  return fn(type, q, { fetchImpl, env });
}

export async function details(type, id, { fetchImpl = fetch, env = process.env } = {}) {
  if (type === 'tv') return tmdbDetails(id, { fetchImpl, env });
  return {};
}
