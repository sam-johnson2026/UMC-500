import { createClient } from '@supabase/supabase-js';

const supaUrl = () => process.env.SUPABASE_URL || process.env.VITE_SUPABASE_URL;
const supaKey = () => process.env.SUPABASE_ANON_KEY || process.env.VITE_SUPABASE_ANON_KEY;

export class HttpError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

// Validates the caller's Supabase access token. Returns { user, token, db } where
// `db` is a Supabase client acting as that user (so Row Level Security applies).
export async function authenticate(req) {
  const token = /^Bearer (.+)$/i.exec(req.headers.authorization || '')?.[1];
  if (!token) throw new HttpError(401, 'Sign in required');
  if (!supaUrl() || !supaKey()) throw new HttpError(500, 'Supabase is not configured on the server');
  const db = createClient(supaUrl(), supaKey(), {
    auth: { persistSession: false, autoRefreshToken: false },
    global: { headers: { Authorization: `Bearer ${token}` } },
  });
  const { data, error } = await db.auth.getUser(token);
  if (error || !data?.user) throw new HttpError(401, 'Invalid or expired session');
  return { user: data.user, token, db };
}

// Wraps a handler so thrown HttpErrors become JSON responses.
export const handle = (fn) => async (req, res) => {
  try {
    await fn(req, res);
  } catch (err) {
    const status = err instanceof HttpError ? err.status : 502;
    if (!(err instanceof HttpError)) console.error(err);
    res.status(status).json({ error: err instanceof HttpError ? err.message : 'Upstream request failed' });
  }
};
