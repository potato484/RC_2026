/**
 * @vitest-environment jsdom
 */
import { describe, expect, test } from 'vitest';
import { xmlToEditorDocument } from '../src/utils/editorParser';
import {
  getBehaviorTreeAttributeDisplay,
  getBehaviorTreeNodeDisplay
} from '../src/utils/btDisplay';
import { buildEditorTreeList, buildEditorTreePreview } from '../src/utils/editorTreeView';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';

describe('Behavior tree display helpers', () => {
  test('should translate node labels and inline details into Chinese', () => {
    const display = getBehaviorTreeNodeDisplay('SetNavMode', { mode: 'MF_SAFE' });

    expect(display.label).toBe('设置导航模式 (模式: 梅林安全模式)');
    expect(display.desc).toContain('切换导航的安全/穿越/正常模式');
    expect(display.desc).toContain('模式: 梅林安全模式');
  });

  test('should translate subtree names and preserve editor tree hierarchy labels', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['梅林区']);
    const treeList = buildEditorTreeList(document);

    expect(treeList.find((tree) => tree.id === 'MFAreaTree')).toEqual({
      id: 'MFAreaTree',
      name: '梅林区树',
      parentTreeId: undefined,
    });

    expect(treeList.find((tree) => tree.id === 'MF_Entry')).toEqual({
      id: 'MF_Entry',
      name: '梅林进门',
      parentTreeId: 'MFAreaTree',
    });

    expect(getBehaviorTreeNodeDisplay('SubTree', { ID: 'MF_Entry', _autoremap: 'true' }).label).toBe('梅林进门');
  });

  test('should translate attribute labels and values into Chinese display text', () => {
    expect(getBehaviorTreeAttributeDisplay('ID', 'MF_Entry')).toMatchObject({
      label: '节点标识',
      value: '梅林进门',
    });

    expect(getBehaviorTreeAttributeDisplay('_autoremap', 'true')).toMatchObject({
      label: '自动映射',
      value: '是',
    });

    expect(getBehaviorTreeAttributeDisplay('code', "next_action=='GRAB'")).toMatchObject({
      label: '脚本',
      value: '判断是否去抓取',
    });
  });

  test('should build a Chinese-only editor structure preview for current trees', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['武馆区']);
    const preview = buildEditorTreePreview(document, 'MCAreaTree');

    expect(preview).toContain('当前决策树：武馆区树');
    expect(preview).toContain('武馆区主流程');
    expect(preview).toContain('抓取矛头');
    expect(preview).not.toMatch(/[A-Za-z]/);
  });
});
