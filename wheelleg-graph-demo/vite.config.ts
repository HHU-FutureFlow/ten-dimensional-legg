import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  server: { proxy: { '/api': 'http://127.0.0.1:' + (process.env.GRAPH_PORT || '4317') } },
  build: {
    outDir: 'dist', sourcemap: true,
    rollupOptions: { output: { manualChunks(moduleId) {
      if (!moduleId.includes('node_modules')) return;
      if (/react-markdown|remark-|rehype-|unified|micromark|mdast-|hast-|unist-|vfile|property-information|parse-entities|decode-named|character-entities|html-url-attributes|space-separated|comma-separated|trim-lines|trough|bail|is-plain-obj|zwitch|escape-string-regexp|markdown-table|devlop|ccount|longest-streak/.test(moduleId)) return 'markdown';
      if (moduleId.includes('@xyflow') || moduleId.includes('d3-') || moduleId.includes('zustand')) return 'graph';
      if (moduleId.includes('ajv') || moduleId.includes('fast-uri') || moduleId.includes('fast-deep-equal') || moduleId.includes('json-schema-traverse') || moduleId.includes('require-from-string')) return 'validation';
      return 'vendor';
    } } },
  },
});
