import { expect, test } from '@playwright/test';

test('编辑态支持点击连线中点插入节点，并可撤销重做', async ({ page }) => {
  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByTestId('editor-edge-insert-trigger').first().click({ force: true });
  await expect(page.locator('[data-testid="editor-insert-menu"]:visible').first()).toBeVisible();

  await page.getByRole('button', { name: /等待视觉目标/ }).first().click();
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' }).first()).toBeVisible();

  await page.keyboard.press('Control+Z');
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' })).toHaveCount(0);

  await page.keyboard.press('Control+Shift+Z');
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' }).first()).toBeVisible();
});

test('窄屏下可通过抽屉访问决策树和编辑检查器', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByRole('button', { name: '决策树' }).click({ force: true });
  await expect(page.locator('[data-testid="phase-sidebar"]:visible').first()).toBeVisible();

  await page.locator('body').click({ position: { x: 380, y: 40 } });
  await page.getByRole('button', { name: '检查器' }).click({ force: true });
  await expect(page.locator('[data-testid="editor-right-panel"]:visible').first()).toBeVisible();
});
