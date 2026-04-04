import { expect, test } from '@playwright/test';

test('编辑态支持点击连线中点插入节点，并可撤销重做', async ({ page }) => {
  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByTestId('editor-edge-insert-trigger').first().click({ force: true });
  await expect(page.locator('[data-testid="editor-insert-menu"]:visible').first()).toBeVisible();

  await page.getByRole('button', { name: /顺序节点/ }).first().click();
  await page.getByRole('button', { name: /等待视觉目标/ }).first().click();
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' }).first()).toBeVisible();

  await page.keyboard.press('Control+Z');
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' })).toHaveCount(0);

  await page.keyboard.press('Control+Shift+Z');
  await expect(page.getByTestId('editor-node-card').filter({ hasText: '等待视觉目标' }).first()).toBeVisible();
});

test('桌面端可通过节点知识库检索说明，并通过工具栏插入节点', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 });
  await page.goto('/');
  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();

  await page.getByTestId('editor-knowledge-base-button').click();
  await expect(page.getByTestId('editor-knowledge-overlay')).toBeVisible();
  const visibleKnowledgeBase = page.getByTestId('editor-knowledge-base');
  await expect(visibleKnowledgeBase).toBeVisible();
  await visibleKnowledgeBase.getByTestId('editor-knowledge-category-official').click();
  const visibleKnowledgeList = visibleKnowledgeBase.getByTestId('editor-knowledge-list');
  const listMetrics = await visibleKnowledgeList.evaluate((element) => ({
    scrollHeight: element.scrollHeight,
    clientHeight: element.clientHeight,
  }));
  expect(listMetrics.scrollHeight).toBeGreaterThan(listMetrics.clientHeight);
  await visibleKnowledgeList.hover();
  await page.mouse.wheel(0, 960);
  await expect
    .poll(async () => visibleKnowledgeList.evaluate((element) => element.scrollTop))
    .toBeGreaterThan(0);

  await visibleKnowledgeBase.getByTestId('editor-knowledge-category-duel').click();
  await visibleKnowledgeBase.getByTestId('editor-knowledge-base-search').fill('目标俯仰角');
  const visibleKnowledgeDetail = visibleKnowledgeBase.getByTestId('editor-knowledge-detail');
  await expect(visibleKnowledgeDetail).toContainText('云台移动');
  await expect(visibleKnowledgeDetail).toContainText('俯仰角');
  await visibleKnowledgeBase.getByLabel('关闭节点知识库').click();

  await page.getByTestId('editor-node-card').first().click({ force: true });
  await expect(page.getByTestId('editor-toolbar-insert-button')).toBeEnabled();
  await page.getByTestId('editor-toolbar-insert-button').click();
  const visibleInsertMenu = page.locator('[data-testid="editor-insert-menu"]:visible').first();
  await expect(visibleInsertMenu).toBeVisible();

  await visibleInsertMenu.getByTestId('editor-insert-position-after').click();
  await visibleInsertMenu.getByRole('button', { name: /顺序节点/ }).first().click();
  await visibleInsertMenu.getByRole('button', { name: /等待视觉目标/ }).first().click();
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

  await page.locator('body').click({ position: { x: 380, y: 40 } });
  await page.getByTestId('editor-mobile-knowledge-base-button').click({ force: true });
  await expect(page.getByTestId('editor-knowledge-overlay')).toBeVisible();
  const visibleMobileKnowledgeBase = page.getByTestId('editor-knowledge-base');
  await expect(visibleMobileKnowledgeBase).toBeVisible();
  await visibleMobileKnowledgeBase.getByTestId('editor-knowledge-category-official').click();
  await expect(visibleMobileKnowledgeBase.getByTestId('editor-knowledge-active-category')).toContainText('官方节点');
  await visibleMobileKnowledgeBase.getByLabel('关闭节点知识库').click();

  await page.getByTestId('editor-node-card').first().click({ force: true });
  await expect(page.getByTestId('editor-mobile-insert-button')).toBeEnabled();
  await page.getByTestId('editor-mobile-insert-button').click({ force: true });
  await expect(page.locator('[data-testid="editor-insert-menu"]:visible').first()).toBeVisible();
});
