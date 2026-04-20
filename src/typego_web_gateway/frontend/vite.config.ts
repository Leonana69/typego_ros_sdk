import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Keep asset paths relative so the bundle can be mounted from any prefix
// (e.g. behind a reverse proxy at /hmi/).
export default defineConfig({
  plugins: [react()],
  base: './',
  server: {
    port: 5173,
    proxy: {
      '/api': 'http://localhost:8080',
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    sourcemap: false,
  },
});
