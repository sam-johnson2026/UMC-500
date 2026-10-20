import { createClient } from '@supabase/supabase-js';

const url = import.meta.env.VITE_SUPABASE_URL;
const key = import.meta.env.VITE_SUPABASE_ANON_KEY;

// The anon key is public by design; Row Level Security (supabase/schema.sql) protects the data.
export const configured = Boolean(url && key);
export const supabase = configured ? createClient(url, key) : null;
