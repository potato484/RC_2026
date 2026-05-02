import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import fs from 'node:fs/promises';
import path from 'path';
import type { IncomingMessage, ServerResponse } from 'node:http';

// 区域到行为树 XML 真源的默认映射；开发态保存会按当前区域写回对应文件。
const defaultBehaviorTreeFilePathByPhase = {
  '梅林区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/mf_tree.xml'),
  '武馆区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/mc_tree.xml'),
  '对抗区': path.resolve(__dirname, '../src/rc26_decision/behavior_trees/combat_tree.xml'),
} as const;

// 保存区域枚举直接从默认映射推导，避免区域名和写回表脱节。
type SavePhase = keyof typeof defaultBehaviorTreeFilePathByPhase;

const resolveBehaviorTreeFilePathByPhase = (): Record<SavePhase, string> => {
  // MERLIN_BT_SAVE_DIR 用于测试把写回目标重定向到临时目录，避免污染 ROS2 真源 XML。
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
  // Vite dev middleware 暴露的是 Node HTTP 请求，这里手工收集完整 body。
  const chunks: Buffer[] = [];
  for await (const chunk of req) {
    chunks.push(typeof chunk === 'string' ? Buffer.from(chunk) : chunk);
  }
  return Buffer.concat(chunks).toString('utf-8');
};

const sendJson = (res: ServerResponse, statusCode: number, payload: unknown) => {
  // 统一用 UTF-8 JSON 返回，前端据此展示中文错误或成功提示。
  res.statusCode = statusCode;
  res.setHeader('Content-Type', 'application/json; charset=utf-8');
  res.end(JSON.stringify(payload));
};

const localBehaviorTreeSavePlugin = () => ({
  // 插件名仅用于 Vite 调试输出，不参与机器人运行时契约。
  name: 'local-behavior-tree-save',
  configureServer(server: { middlewares: { use: (path: string, handler: (req: IncomingMessage, res: ServerResponse, next: () => void) => void | Promise<void>) => void } }) {
    // 本地开发保存 API；生产构建不提供机器人在线写回能力。
    server.middlewares.use('/api/editor/save-xml', async (req, res, next) => {
      if (req.method !== 'POST') {
        next();
        return;
      }

      try {
        const behaviorTreeFilePathByPhase = resolveBehaviorTreeFilePathByPhase();
        const rawBody = await readRequestBody(req);
        // phase 决定写回哪个区域 XML，xmlContent 是编辑器序列化后的完整行为树内容。
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
  // React 插件负责 TSX/fast refresh；本地保存插件只在 dev server 中挂载中间件。
  plugins: [react(), localBehaviorTreeSavePlugin()],
  resolve: {
    // @ 指向前端 src，保持组件和工具函数导入路径稳定。
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  server: {
    fs: {
      // 行为树 XML 真源在前端目录上一级的 ROS2 workspace 中，开发态需要允许只读访问。
      allow: ['..']
    }
  }
});
