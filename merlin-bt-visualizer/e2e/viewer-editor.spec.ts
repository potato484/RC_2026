import { expect, test } from '@playwright/test';

const disallowedEnglishTokens = [
  'Sequence',
  'Fallback',
  'ReactiveSequence',
  'ReactiveFallback',
  'SubTree',
  'RetryUntilSuccessful',
  'KeepRunningUntilFailure',
  'ForceSuccess',
  'ForceFailure',
  'Inverter',
];

test('查看态和编辑态都随区域切换同步，默认展示不暴露英文节点名', async ({ page }) => {
  await page.goto('/');

  await expect(page.getByTestId('viewer-canvas')).toBeVisible();
  await expect(page.getByTestId('active-phase-label')).toContainText('梅林区');

  const viewerTreeListBefore = (await page.getByTestId('viewer-tree-list').innerText()).trim();
  expect(viewerTreeListBefore).not.toBe('');

  await page.getByTestId('phase-对抗区').click();
  await expect(page.getByTestId('active-phase-label')).toContainText('对抗区');

  const viewerTreeListAfter = (await page.getByTestId('viewer-tree-list').innerText()).trim();
  expect(viewerTreeListAfter).not.toBe('');
  expect(viewerTreeListAfter).not.toBe(viewerTreeListBefore);

  const viewerText = await page.locator('body').innerText();
  for (const token of disallowedEnglishTokens) {
    expect(viewerText).not.toContain(token);
  }

  await page.getByTestId('app-mode-toggle').click();
  await expect(page.getByTestId('editor-canvas')).toBeVisible();
  await expect(page.getByText('编辑模式')).toBeVisible();

  await page.getByTestId('editor-tab-preview').click();
  const editorTreeListBefore = (await page.getByTestId('editor-tree-list').innerText()).trim();
  const structurePreviewBefore = (await page.getByTestId('editor-structure-preview').innerText()).trim();
  expect(editorTreeListBefore).not.toBe('');
  expect(structurePreviewBefore).not.toBe('');

  for (const token of disallowedEnglishTokens) {
    expect(editorTreeListBefore).not.toContain(token);
    expect(structurePreviewBefore).not.toContain(token);
  }

  await page.getByTestId('phase-武馆区').click();
  await expect(page.getByTestId('active-phase-label')).toContainText('武馆区');

  const editorTreeListAfter = (await page.getByTestId('editor-tree-list').innerText()).trim();
  const structurePreviewAfter = (await page.getByTestId('editor-structure-preview').innerText()).trim();

  expect(editorTreeListAfter).not.toBe('');
  expect(editorTreeListAfter).not.toBe(editorTreeListBefore);
  expect(structurePreviewAfter).not.toBe('');
  expect(structurePreviewAfter).not.toBe(structurePreviewBefore);

  for (const token of disallowedEnglishTokens) {
    expect(editorTreeListAfter).not.toContain(token);
    expect(structurePreviewAfter).not.toContain(token);
  }
});
