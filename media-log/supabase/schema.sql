-- Run in the Supabase SQL editor (or `supabase db push`).

create table if not exists public.entries (
  id          uuid primary key default gen_random_uuid(),
  user_id     uuid not null default auth.uid() references auth.users (id) on delete cascade,
  type        text not null check (type in ('anime','tv','movie','manga','manhwa','comic','book','light_novel','podcast','game','youtube','other')),
  title       text not null check (char_length(title) between 1 and 300),
  status      text not null default 'plan_to' check (status in ('in_progress','plan_to','finished','on_hold','dropped')),
  season      integer check (season is null or season > 0),
  progress    numeric not null default 0 check (progress >= 0),
  unit        text not null default 'episode' check (unit in ('episode','chapter','issue','volume','page','percent')),
  total       numeric check (total is null or total > 0),
  rating      text check (rating in ('up','side','down')),
  notes       text not null default '' check (char_length(notes) <= 4000),
  cover_url   text,
  year        integer,
  source      text,
  source_id   text,
  created_at  timestamptz not null default now(),
  updated_at  timestamptz not null default now()
);

create index if not exists entries_user_type_idx on public.entries (user_id, type, status);

create or replace function public.touch_updated_at() returns trigger language plpgsql as $$
begin new.updated_at = now(); return new; end $$;

drop trigger if exists entries_touch on public.entries;
create trigger entries_touch before update on public.entries
  for each row execute function public.touch_updated_at();

alter table public.entries enable row level security;

create policy "own entries: select" on public.entries for select using (user_id = auth.uid());
create policy "own entries: insert" on public.entries for insert with check (user_id = auth.uid());
create policy "own entries: update" on public.entries for update using (user_id = auth.uid()) with check (user_id = auth.uid());
create policy "own entries: delete" on public.entries for delete using (user_id = auth.uid());

-- Storage: private-by-folder bucket for custom cover uploads (public read so <img> works).
insert into storage.buckets (id, name, public) values ('covers', 'covers', true)
  on conflict (id) do nothing;

create policy "covers: read" on storage.objects for select using (bucket_id = 'covers');
create policy "covers: upload to own folder" on storage.objects for insert to authenticated
  with check (bucket_id = 'covers' and (storage.foldername(name))[1] = auth.uid()::text);
create policy "covers: replace own" on storage.objects for update to authenticated
  using (bucket_id = 'covers' and (storage.foldername(name))[1] = auth.uid()::text);
create policy "covers: delete own" on storage.objects for delete to authenticated
  using (bucket_id = 'covers' and (storage.foldername(name))[1] = auth.uid()::text);
