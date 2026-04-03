import fs from 'node:fs/promises';
import path from 'node:path';
import { expect, test } from '@playwright/test';

const saveDirectory = process.env.E2E_SAVE_DIR;

test.skip(!saveDirectory, '缺少 E2E_SAVE_DIR，无法校验开发态写回链路');

test('开发态保存按钮会把当前区域改动写回临时源文件', async ({ page }) => {
  const attributeKey = `测试字段${Date.now()}`;
  const attributeValue = `自动化写回${Date.now()}`;
  const merlinSavePath = path.resolve(saveDirectory as string, 'mf_tree.xml');

  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByTestId('editor-node-card').first().click();
  await expect(page.getByTestId('editor-right-panel')).toBeVisible();

  await page.getByTestId('new-attribute-key').fill(attributeKey);
  await page.getByTestId('new-attribute-value').fill(attributeValue);
  await page.getByTestId('add-attribute-button').click();

  await page.getByTestId('save-source-button').click();
  await expect(page.getByTestId('save-state-banner')).toContainText('已写回 梅林区 对应的源文件');

  const savedXml = await fs.readFile(merlinSavePath, 'utf-8');
  expect(savedXml).toContain(`${attributeKey}="${attributeValue}"`);
});
