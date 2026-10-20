import test from 'node:test';
import assert from 'node:assert/strict';
import { search, details } from '../api/_lib/providers.js';
import { createSearchHandler } from '../api/search.js';
import { dedupePicks, bestMatch, buildTasteProfile } from '../api/_lib/recommend-core.js';

const json = (body, ok = true, status = 200) => async () => ({ ok, status, json: async () => body });
const capture = (body) => {
  const calls = [];
  const fn = async (url, init) => { calls.push({ url: String(url), init }); return { ok: true, status: 200, json: async () => body }; };
  fn.calls = calls;
  return fn;
};

test('AniList anime: maps title, year, cover and episode total', async () => {
  const fetchImpl = capture({ data: { Page: { media: [
    { id: 154587, title: { romaji: 'Sousou no Frieren', english: 'Frieren: Beyond Journey\'s End' }, seasonYear: 2023, startDate: { year: 2023 }, episodes: 28, chapters: null, volumes: null, description: 'An elf <br>mage.', coverImage: { large: 'https://s4.anilist.co/x.jpg' } },
    { id: 2, title: { romaji: 'Only Romaji', english: null }, seasonYear: null, startDate: { year: 2001 }, episodes: null, coverImage: { large: null, medium: 'm.jpg' } },
  ] } } });
  const r = await search('anime', 'frieren', { fetchImpl, env: {} });
  assert.deepEqual(r[0], { id: '154587', source: 'anilist', title: "Frieren: Beyond Journey's End", year: 2023, coverUrl: 'https://s4.anilist.co/x.jpg', total: 28, unit: 'episode', overview: 'An elf mage.' });
  assert.equal(r[1].title, 'Only Romaji');
  assert.equal(r[1].year, 2001);
  assert.equal(r[1].total, null);
  assert.equal(r[1].coverUrl, 'm.jpg');
  const sent = JSON.parse(fetchImpl.calls[0].init.body);
  assert.equal(sent.variables.type, 'ANIME');
});

test('AniList light novels use the NOVEL format and volume totals', async () => {
  const fetchImpl = capture({ data: { Page: { media: [{ id: 1, title: { romaji: 'LN' }, volumes: 14, chapters: 99, coverImage: {} }] } } });
  const [ln] = await search('light_novel', 'ln', { fetchImpl, env: {} });
  assert.equal(ln.total, 14);
  assert.equal(ln.unit, 'volume');
  assert.deepEqual(JSON.parse(fetchImpl.calls[0].init.body).variables.formats, ['NOVEL']);
});

test('AniList GraphQL errors surface', async () => {
  await assert.rejects(search('anime', 'x', { fetchImpl: json({ errors: [{ message: 'boom' }] }), env: {} }), /boom/);
});

test('TMDB: requires key, builds poster URL, supports v4 bearer tokens', async () => {
  await assert.rejects(search('movie', 'dune', { fetchImpl: json({}), env: {} }), /TMDB_API_KEY/);
  const body = { results: [{ id: 438631, title: 'Dune', release_date: '2021-09-15', poster_path: '/p.jpg', overview: 'Spice.' }] };
  const f3 = capture(body);
  const [m] = await search('movie', 'dune', { fetchImpl: f3, env: { TMDB_API_KEY: 'abc' } });
  assert.equal(m.coverUrl, 'https://image.tmdb.org/t/p/w342/p.jpg');
  assert.equal(m.year, 2021);
  assert.match(f3.calls[0].url, /api_key=abc/);
  const f4 = capture(body);
  await search('movie', 'dune', { fetchImpl: f4, env: { TMDB_API_KEY: 'x'.repeat(60) } });
  assert.doesNotMatch(f4.calls[0].url, /api_key/);
  assert.match(f4.calls[0].init.headers.Authorization, /^Bearer /);
});

test('TMDB TV details list per-season episode counts', async () => {
  const f = capture({ seasons: [{ season_number: 0, episode_count: 3 }, { season_number: 1, episode_count: 10 }, { season_number: 2, episode_count: 8 }] });
  const d = await details('tv', '1396', { fetchImpl: f, env: { TMDB_API_KEY: 'k' } });
  assert.deepEqual(d.seasons, [{ number: 1, episodes: 10 }, { number: 2, episodes: 8 }]);
});

test('Open Library: cover URL and page total', async () => {
  const [b] = await search('book', 'dune', { fetchImpl: json({ docs: [{ key: '/works/OL1W', title: 'Dune', author_name: ['Frank Herbert'], first_publish_year: 1965, cover_i: 99, number_of_pages_median: 612 }] }), env: {} });
  assert.equal(b.coverUrl, 'https://covers.openlibrary.org/b/id/99-M.jpg');
  assert.equal(b.total, 612);
  assert.equal(b.unit, 'page');
});

test('RAWG: requires key, maps image and year', async () => {
  await assert.rejects(search('game', 'zelda', { fetchImpl: json({}), env: {} }), /RAWG_API_KEY/);
  const [g] = await search('game', 'zelda', { fetchImpl: json({ results: [{ id: 5, name: 'Zelda', released: '2017-03-03', background_image: 'bg.jpg', genres: [{ name: 'Adventure' }] }] }), env: { RAWG_API_KEY: 'k' } });
  assert.deepEqual([g.title, g.year, g.coverUrl, g.unit], ['Zelda', 2017, 'bg.jpg', 'percent']);
});

test('MangaDex: cover from relationship, lastChapter total, Korean origin filter', async () => {
  const f = capture({ data: [{ id: 'abc', attributes: { title: { en: 'Solo Leveling' }, year: 2018, lastChapter: '200', description: { en: 'Hunter.' } }, relationships: [{ type: 'cover_art', attributes: { fileName: 'c.jpg' } }] }, { id: 'd', attributes: { title: { ja: 'X' }, lastChapter: '' }, relationships: [] }] });
  const r = await search('manhwa', 'solo', { fetchImpl: f, env: {} });
  assert.equal(r[0].coverUrl, 'https://uploads.mangadex.org/covers/abc/c.jpg.256.jpg');
  assert.equal(r[0].total, 200);
  assert.equal(r[1].total, null);
  assert.equal(r[1].coverUrl, null);
  assert.match(decodeURIComponent(f.calls[0].url), /originalLanguage\[\]=ko/);
});

test('types without a source return nothing', async () => {
  assert.deepEqual(await search('podcast', 'x', { fetchImpl: json({}), env: {} }), []);
});

test('search handler: requires auth, validates input, never reaches providers when unauthenticated', async () => {
  const mk = (auth) => {
    let hits = 0;
    const handler = createSearchHandler({
      authenticate: auth,
      providers: { hasSearch: (t) => t !== 'podcast', search: async () => { hits++; return [{ title: 'x' }]; }, details: async () => ({}) },
    });
    const call = async (req) => {
      const out = { headers: {} };
      const res = { setHeader: (k, v) => (out.headers[k] = v), status(c) { out.status = c; return this; }, json(b) { out.body = b; return this; } };
      await handler({ method: 'GET', headers: {}, ...req }, res);
      return out;
    };
    return { call, hits: () => hits };
  };
  const denied = mk(async () => { const { HttpError } = await import('../api/_lib/auth.js'); throw new HttpError(401, 'Sign in required'); });
  const r1 = await denied.call({ query: { type: 'anime', q: 'frieren' } });
  assert.equal(r1.status, 401);
  assert.equal(denied.hits(), 0);

  const ok = mk(async () => ({ user: { id: 'u' } }));
  assert.equal((await ok.call({ query: { type: 'anime', q: 'f' } })).body.results.length, 0); // too short
  assert.equal((await ok.call({ query: { type: 'anime', q: 'frieren' } })).body.results.length, 1);
  assert.deepEqual((await ok.call({ query: { type: 'podcast', q: 'abc' } })).body, { results: [], manual: true });
  assert.equal((await ok.call({ query: {}, })).status, 400);
  assert.equal((await ok.call({ method: 'POST', query: {} })).status, 405);
});

test('recommend helpers: dedupe against the log, best match, taste profile weights ratings', () => {
  const log = [
    { title: 'Frieren', type: 'anime', status: 'finished', rating: 'up' },
    { title: 'Boring Show', type: 'anime', status: 'dropped', rating: 'down' },
    { title: 'Planned One', type: 'anime', status: 'plan_to', rating: null },
  ];
  const picks = [{ title: 'frieren!' }, { title: 'Planned One' }, { title: 'A' }, { title: 'A' }, { title: 'B' }];
  assert.deepEqual(dedupePicks(picks, log, 6).map((p) => p.title), ['A', 'B']);
  assert.equal(bestMatch([{ title: 'Other', year: 2000 }, { title: 'A', year: 2020 }], { title: 'A', year: 2020 }).title, 'A');
  const profile = buildTasteProfile(log);
  assert.match(profile, /LOVED[\s\S]*Frieren/);
  assert.match(profile, /DISLIKED[\s\S]*Boring Show/);
  assert.match(profile, /ALREADY PLANNED[\s\S]*Planned One/);
});
