import { authenticate, handle, HttpError } from './_lib/auth.js';
import { search, details, hasSearch } from './_lib/providers.js';

// Factory so tests can inject auth + providers.
export const createSearchHandler = (deps = {}) => {
  const auth = deps.authenticate ?? authenticate;
  const providers = deps.providers ?? { search, details, hasSearch };
  return handle(async (req, res) => {
    if (req.method !== 'GET') throw new HttpError(405, 'GET only');
    await auth(req);
    const { type, q, id } = req.query;
    if (!type) throw new HttpError(400, 'type is required');
    if (id) {
      res.setHeader('Cache-Control', 'private, max-age=3600');
      return res.status(200).json(await providers.details(type, id));
    }
    if (!providers.hasSearch(type)) return res.status(200).json({ results: [], manual: true });
    const query = String(q || '').trim();
    if (query.length < 2) return res.status(200).json({ results: [] });
    res.setHeader('Cache-Control', 'private, max-age=300');
    res.status(200).json({ results: await providers.search(type, query.slice(0, 120)) });
  });
};

export default createSearchHandler();
