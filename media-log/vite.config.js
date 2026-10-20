import { defineConfig, loadEnv } from 'vite';
import react from '@vitejs/plugin-react';
import { pathToFileURL } from 'node:url';
import path from 'node:path';

// Serves /api/* from ./api during `npm run dev`, so Vercel's CLI isn't required locally.
// Server-only vars from .env (no VITE_ prefix) are copied onto process.env here and are
// never bundled into the frontend.
function devApi() {
  return {
    name: 'media-log-dev-api',
    configureServer(server) {
      Object.assign(process.env, loadEnv(server.config.mode, process.cwd(), ''));
      server.middlewares.use('/api', async (req, res, next) => {
        const url = new URL(req.url, 'http://localhost');
        const name = url.pathname.replace(/^\/|\/$/g, '');
        if (!/^[a-z-]+$/.test(name)) return next();
        try {
          const file = pathToFileURL(path.resolve('api', `${name}.js`)).href;
          const mod = await server.ssrLoadModule(file.replace('file://', ''));
          // Mimic the helpers Vercel adds to Node requests/responses.
          req.query = Object.fromEntries(url.searchParams);
          if (req.method === 'POST') {
            const chunks = [];
            for await (const c of req) chunks.push(c);
            try { req.body = JSON.parse(Buffer.concat(chunks).toString() || '{}'); } catch { req.body = {}; }
          }
          res.status = (code) => { res.statusCode = code; return res; };
          res.json = (obj) => { res.setHeader('Content-Type', 'application/json'); res.end(JSON.stringify(obj)); return res; };
          await mod.default(req, res);
        } catch (err) {
          console.error(err);
          res.statusCode = 500;
          res.end(JSON.stringify({ error: 'Dev API error' }));
        }
      });
    },
  };
}

export default defineConfig({ plugins: [react(), devApi()] });
