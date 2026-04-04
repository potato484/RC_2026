import fs from 'node:fs/promises';
import path from 'node:path';
import { expect, test } from '@playwright/test';

const saveDirectory = process.env.E2E_SAVE_DIR;

test.skip(!saveDirectory, '缺少 E2E_SAVE_DIR，无法校验开发态写回链路');

test('开发态保存按钮会把当前区域改动写回临时源文件', async ({ page }) => {
  const merlinSavePath = path.resolve(saveDirectory as string, 'mf_tree.xml');

  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByTestId('editor-node-card').first().click({ force: true });
  await expect(page.getByTestId('editor-toolbar-insert-button')).toBeEnabled();
  await page.getByTestId('editor-toolbar-insert-button').click();

  const visibleInsertMenu = page.locator('[data-testid="editor-insert-menu"]:visible').first();
  await expect(visibleInsertMenu).toBeVisible();
  await visibleInsertMenu.getByTestId('editor-insert-position-after').click();
  await visibleInsertMenu.getByRole('button', { name: /顺序节点/ }).first().click();
  await visibleInsertMenu.getByRole('button', { name: /等待视觉目标/ }).first().click();
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' }).first()).toBeVisible();

  await page.getByTestId('save-source-button').click();
  await expect(page.getByTestId('save-state-banner')).toContainText('已写回 梅林区 对应的源文件');

  const savedXml = await fs.readFile(merlinSavePath, 'utf-8');
  expect(savedXml).toContain('<WaitVisionTarget');
});
