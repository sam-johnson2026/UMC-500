import Anthropic from '@anthropic-ai/sdk';
import { authenticate, handle, HttpError } from './_lib/auth.js';
import { search, hasSearch } from './_lib/providers.js';
import { PICKS_SCHEMA, SYSTEM_PROMPT, buildUserPrompt, dedupePicks, bestMatch, TYPE_LABELS } from './_lib/recommend-core.js';

const MODEL = () => process.env.ANTHROPIC_MODEL || 'claude-opus-5-5';
const MAX_LOG = 400;

export const createRecommendHandler = (deps = {}) => {
  const auth = deps.authenticate ?? authenticate;
  const providerSearch = deps.search ?? search;
  const getClient = deps.getClient ?? (() => new Anthropic());

  return handle(async (req, res) => {
    if (req.method !== 'POST') throw new HttpError(405, 'POST only');
    if (!deps.getClient && !process.env.ANTHROPIC_API_KEY) throw new HttpError(500, 'ANTHROPIC_API_KEY is not configured');
    const { db } = await auth(req);

    const { type, mood = '' } = req.body ?? {};
    if (!(type in TYPE_LABELS)) throw new HttpError(400, 'Unknown media type');

    // Read the log as the user, so Row Level Security scopes it to them.
    const { data: entries, error } = await db
      .from('entries')
      .select('title,type,status,rating,year,notes,updated_at')
      .order('updated_at', { ascending: false })
      .limit(MAX_LOG);
    if (error) throw new HttpError(500, 'Could not read your log');

    const client = getClient();
    const message = await client.beta.messages.create({
      model: MODEL(),
      max_tokens: 4000,
      betas: ['server-side-fallback-2026-07-01'],
      fallbacks: 'default',
      system: SYSTEM_PROMPT,
      messages: [{ role: 'user', content: buildUserPrompt({ type, mood: String(mood).slice(0, 300), entries }) }],
      output_config: { effort: 'medium', format: { type: 'json_schema', schema: PICKS_SCHEMA } },
    });
    if (message.stop_reason === 'refusal') throw new HttpError(422, 'The model declined this request. Try a different mood.');
    const text = message.content.find((b) => b.type === 'text')?.text;
    let raw;
    try {
      raw = JSON.parse(text).picks;
    } catch {
      throw new HttpError(502, 'Could not read the recommendations');
    }

    const picks = dedupePicks(raw, entries, 6);
    // Attach cover / total from the same metadata source the add form uses (best effort).
    const enriched = await Promise.all(
      picks.map(async (p) => {
        let match = null;
        if (hasSearch(type)) {
          try {
            match = bestMatch(await providerSearch(type, p.title), p);
          } catch {
            /* metadata is optional */
          }
        }
        return {
          title: match?.title && match.title.toLowerCase() === p.title.toLowerCase() ? match.title : p.title,
          year: match?.year ?? p.year,
          reason: p.reason,
          coverUrl: match?.coverUrl ?? null,
          total: match?.total ?? null,
          unit: match?.unit ?? null,
          source: match?.source ?? null,
          sourceId: match?.id ?? null,
        };
      }),
    );
    res.status(200).json({ picks: enriched });
  });
};

export default createRecommendHandler();
