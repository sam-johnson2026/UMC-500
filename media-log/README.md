# Media Log

A personal tracker for anime, TV, movies, manga, manhwa, comics, books, light novels, podcasts, games, YouTube/web and anything else.
React + Vite frontend · Supabase (auth, Postgres, storage) · Vercel hosting with serverless functions in `/api`.

## Setup

1. **Supabase**: create a project, run `supabase/schema.sql` in the SQL editor (table, Row Level Security, and a `covers` storage bucket).
2. **Env**: copy `.env.example` to `.env.local` and fill it in.
   - `VITE_*` values are public by design (the anon key is guarded by RLS).
   - `TMDB_API_KEY` and `ANTHROPIC_API_KEY` are server-only and are read only by `/api/*`.
3. `npm install && npm run dev`. The Vite dev server also serves `/api/*` from `./api`, so the Vercel CLI isn't needed locally.
4. **Deploy**: import the repo in Vercel with this folder as the root directory and add the same env vars (without the `.env.local` file).

## How it fits together

| Piece | Where |
|---|---|
| Metadata search by type | `api/search.js` → `api/_lib/providers.js` |
| Recommendations | `api/recommend.js` (reads your log as you, so RLS applies; calls Anthropic) |
| Auth on the functions | `api/_lib/auth.js`: every call needs your Supabase session token |
| Types, units, "S2 · Ep 6 / 12" formatting | `src/lib/media.js` |

Search sources: AniList (anime, manga, light novels; no key), TMDB (movies, TV; per-season episode counts), Open Library (books), MangaDex (manhwa, Korean-origin). Games, comics, podcasts, YouTube/web and other are entered by hand.

The recommender sends your log grouped by signal (thumbs up and down weigh the most, unrated drops are a mild negative), asks for 8 picks in a JSON schema, removes anything already in your log, keeps 6 and attaches covers from the same metadata source. Model defaults to `claude-opus-5-5`; set `ANTHROPIC_MODEL` to change it.

## Tests

`npm test` covers every provider adapter against sample responses, the auth gate on `/api/search`, and the recommend flow with a stubbed model.
