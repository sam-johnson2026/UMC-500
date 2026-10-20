import { supabase } from './supabase.js';

async function request(path, { method = 'GET', body, params } = {}) {
  const { data } = await supabase.auth.getSession();
  const token = data.session?.access_token;
  const qs = params ? `?${new URLSearchParams(params)}` : '';
  const res = await fetch(`/api/${path}${qs}`, {
    method,
    headers: { Authorization: `Bearer ${token}`, ...(body ? { 'Content-Type': 'application/json' } : {}) },
    body: body ? JSON.stringify(body) : undefined,
  });
  const json = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(json.error || `Request failed (${res.status})`);
  return json;
}

export const searchMedia = (type, q) => request('search', { params: { type, q } }).then((r) => r.results);
export const mediaDetails = (type, id) => request('search', { params: { type, id } });
export const recommend = (type, mood) => request('recommend', { method: 'POST', body: { type, mood } }).then((r) => r.picks);
