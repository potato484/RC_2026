import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import fs from 'node:fs/promises';
import path from 'path';
import type { IncomingMessage, ServerResponse } from 'node:http';

const defaultBehaviorTreeFilePathByPhase = {
  '梅林区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/mf_tree.xml'),
  '武馆区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/mc_tree.xml'),
  '对抗区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/combat_tree.xml'),
} as const;

type SavePhase = keyof typeof defaultBehaviorTreeFilePathByPhase;

const resolveBehaviorTreeFilePathByPhase = (): Record<SavePhase, string> => {
  const saveDirectory = process.env.MERLIN_BT_SAVE_DIR?.trim();
  if (!saveDirectory) {
    return { ...defaultBehaviorTreeFilePathByPhase };
  }

  return {
    '梅林区': path.resolve(saveDirectory, 'mf_tree.xml'),
    '武馆区': path.resolve(saveDirectory, 'mc_tree.xml'),
    '对抗区': path.resolve(saveDirectory, 'combat_tree.xml'),
  };
};

const readRequestBody = async (req: IncomingMessage): Promise<string> => {
  const chunks: Buffer[] = [];
  for await (const chunk of req) {
    chunks.push(typeof chunk === 'string' ? Buffer.from(chunk) : chunk);
  }
  return Buffer.concat(chunks).toString('utf-8');
};

const sendJson = (res: ServerResponse, statusCode: number, payload: unknown) => {
  res.statusCode = statusCode;
  res.setHeader('Content-Type', 'application/json; charset=utf-8');
  res.end(JSON.stringify(payload));
};

const localBehaviorTreeSavePlugin = () => ({
  name: 'local-behavior-tree-save',
  configureServer(server: { middlewares: { use: (path: string, handler: (req: IncomingMessage, res: ServerResponse, next: () => void) => void | Promise<void>) => void } }) {
    server.middlewares.use('/api/editor/save-xml', async (req, res, next) => {
      if (req.method !== 'POST') {
        next();
        return;
      }

      try {
        const behaviorTreeFilePathByPhase = resolveBehaviorTreeFilePathByPhase();
        const rawBody = await readRequestBody(req);
        const payload = JSON.parse(rawBody) as { phase?: SavePhase; xmlContent?: string };
        const phase = payload.phase;
        const xmlContent = payload.xmlContent;

        if (!phase || !(phase in behaviorTreeFilePathByPhase)) {
          sendJson(res, 400, { message: '缺少合法的区域标识' });
          return;
        }

        if (!xmlContent || !xmlContent.trim()) {
          sendJson(res, 400, { message: '缺少可保存的源文件内容' });
          return;
        }

        const targetPath = behaviorTreeFilePathByPhase[phase];
        await fs.writeFile(targetPath, xmlContent, 'utf-8');

        sendJson(res, 200, {
          message: `已写回 ${phase} 对应的源文件`,
          savedPath: targetPath,
        });
      } catch (error) {
        const message = error instanceof Error ? error.message : '保存失败';
        sendJson(res, 500, { message });
      }
    });
  },
});

export default defineConfig({
  plugins: [react(), localBehaviorTreeSavePlugin()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  server: {
    fs: {
      allow: ['..']
    }
  }
});
