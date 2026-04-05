/**
 * @vitest-environment jsdom
 */
import { describe, expect, test } from 'vitest';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';
import { xmlToEditorDocument } from '../src/utils/editorParser';
import { buildEditorKnowledgeBaseCatalog } from '../src/utils/editorInsertCatalog';

describe('knowledge-base catalog docs', () => {
  test('provides complete beginner guide sections and port hints for every visible item', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['梅林区']);
    const categories = buildEditorKnowledgeBaseCatalog(document, 'MFAreaTree');
    const items = categories.flatMap((category) => category.items);

    expect(items.length).toBeGreaterThan(0);

    items.forEach((item) => {
      expect(item.description.length).toBeGreaterThan(0);
      expect(item.guideZh.overviewZh.length).toBeGreaterThan(0);
      expect(item.guideZh.whenToUseZh.length).toBeGreaterThan(0);
      expect(item.guideZh.placementZh.length).toBeGreaterThan(0);
      expect(item.guideZh.runningZh.length).toBeGreaterThan(0);
      expect(item.guideZh.successZh.length).toBeGreaterThan(0);
      expect(item.guideZh.failureZh.length).toBeGreaterThan(0);
      expect(item.guideZh.pitfallsZh.length).toBeGreaterThan(0);
      expect(item.guideZh.exampleZh.length).toBeGreaterThan(0);

      item.portSchemas.forEach((port) => {
        expect(port.beginnerHintZh.length).toBeGreaterThan(0);
      });
    });
  });

  test('indexes beginner-friendly guide text and dynamic subtree docs into search tokens', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['梅林区']);
    const categories = buildEditorKnowledgeBaseCatalog(document, 'MFAreaTree');
    const officialSequence = categories
      .flatMap((category) => category.items)
      .find((item) => item.tagName === 'Sequence');
    const subtreeItem = categories
      .find((category) => category.id === 'subtree')
      ?.items.find((item) => item.tagLabel === 'MF_Entry');

    expect(officialSequence).toBeTruthy();
    expect(officialSequence?.searchTokens.some((token) => token.includes('备选方案'))).toBe(true);

    expect(subtreeItem).toBeTruthy();
    expect(subtreeItem?.guideZh.overviewZh).toContain(subtreeItem?.label ?? '');
    expect(subtreeItem?.portSchemas.find((port) => port.name === 'ID')?.exampleValueZh).toBe('MF_Entry');
    expect(subtreeItem?.searchTokens.some((token) => token.includes('自动映射'))).toBe(true);
  });
});
