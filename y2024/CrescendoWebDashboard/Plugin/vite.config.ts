import { defineConfig, Plugin } from 'vite';
import basicSsl from '@vitejs/plugin-basic-ssl';
import { hideBin } from 'yargs/helpers';
import yargs from 'yargs/yargs';
import fs from 'fs';
import path from 'node:path';
import dts from 'vite-plugin-dts';

const argv = yargs(hideBin(process.argv)).argv;

const plugins: Plugin[] = [dts()];

if ((argv as any)._.includes('--useHttps')) {
  plugins.push(basicSsl());
}

function getEntries(folder: string): Record<string, string> {
  const entries: Record<string, string> = {};

  const folderPath = path.resolve(__dirname, `src/${folder}`);
  const fileNames = fs.readdirSync(folderPath, { withFileTypes: true });

  fileNames.forEach((fileName) => {
    if (fileName.isDirectory()) {
      entries[
        `${folder}/${fileName.name}`
      ] = `src/${folder}/${fileName.name}/index.ts`;
    }
  });

  return {
    [folder]: `src/${folder}/index.ts`,
    ...entries,
  };
}

function getAllEntries(): Record<string, string> {
  return {
    ...getEntries('components'),
    fwc: 'src/index.ts',
  };
}

export default defineConfig({
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
      '@dashboard': path.resolve(__dirname, './src/dashboard'),
    },
  },
  build: {
    lib: {
      entry: getAllEntries(),
      formats: ['es'],
      fileName: (format, entryName) => `${entryName}.${format}.js`,
    },
    minify: true,
    sourcemap: true,
  },
  server: {
    open: '/',
  },
  plugins,
});
