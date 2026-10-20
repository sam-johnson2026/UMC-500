import test from 'node:test';
import assert from 'node:assert/strict';
import { createRecommendHandler } from '../api/recommend.js';

const entries = [
  { title: 'Frieren', type: 'anime', status: 'finished', rating: 'up', year: 2023, notes: '' },
  { title: 'Boring Show', type: 'anime', status: 'dropped', rating: 'down', year: 2010, notes: '' },
];
const db = { from: () => ({ select: () => ({ order: () => ({ limit: async () => ({ data: entries, error: null }) }) }) }) };

function run(body, { picks, method = 'POST' } = {}) {
  let sent;
  const client = { beta: { messages: { create: async (args) => { sent = args; return { stop_reason: 'end_turn', content: [{ type: 'text', text: JSON.stringify({ picks }) }] }; } } } };
  const handler = createRecommendHandler({
    authenticate: async () => ({ db }),
    getClient: () => client,
    search: async (_t, title) => [{ title, year: 2020, coverUrl: 'c.jpg', total: 12, unit: 'episode', source: 'anilist', id: '7' }],
  });
  const out = {};
  const res = { status(c) { out.status = c; return this; }, json(b) { out.body = b; return this; } };
  return handler({ method, headers: {}, body }, res).then(() => ({ out, sent }));
}

test('recommend: sends weighted log, drops titles already logged, enriches with covers', async () => {
  const picks = [{ title: 'Frieren', year: 2023, reason: 'dup' }, ...'ABCDEFG'.split('').map((c) => ({ title: `Show ${c}`, year: 2020, reason: `because ${c}` }))];
  const { out, sent } = await run({ type: 'anime', mood: 'cozy' }, { picks });
  assert.equal(out.status, 200);
  assert.equal(out.body.picks.length, 6);
  assert.ok(!out.body.picks.some((p) => p.title === 'Frieren'));
  assert.deepEqual([out.body.picks[0].coverUrl, out.body.picks[0].total, out.body.picks[0].source], ['c.jpg', 12, 'anilist']);
  const prompt = sent.messages[0].content;
  assert.match(prompt, /LOVED[\s\S]*Frieren/);
  assert.match(prompt, /DISLIKED[\s\S]*Boring Show/);
  assert.match(prompt, /Mood.*cozy/);
  assert.equal(sent.output_config.format.type, 'json_schema');
  assert.equal(sent.fallbacks, 'default');
});

test('recommend: rejects unknown types and non-POST', async () => {
  assert.equal((await run({ type: 'nope' }, { picks: [] })).out.status, 400);
  assert.equal((await run({}, { picks: [], method: 'GET' })).out.status, 405);
});
