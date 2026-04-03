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
    const display = getBehaviorTreeNodeDisplay('RetryUntilSuccessful', { num_attempts: '3' });

    expect(display.label).toBe('重试直到成功装饰器（重试次数：3）');
    expect(display.desc).toContain('子节点失败会继续重试');
    expect(display.desc).toContain('重试次数：3');
  });

  test('should translate subtree names and preserve editor tree hierarchy labels', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['梅林区']);
    const treeList = buildEditorTreeList(document);

    expect(treeList.find((tree) => tree.id === 'MFAreaTree')).toEqual({
      id: 'MFAreaTree',
      name: '梅林主任务',
      parentTreeId: undefined,
    });

    expect(treeList.find((tree) => tree.id === 'MF_Entry')).toEqual({
      id: 'MF_Entry',
      name: '梅林进门阶段',
      parentTreeId: 'MFAreaTree',
    });

    expect(getBehaviorTreeNodeDisplay('SubTree', { ID: 'MF_Entry', _autoremap: 'true' }).label).toBe(
      '梅林进门阶段（子树标识：梅林进门阶段，自动映射：是）'
    );
  });

  test('should translate attribute labels and values into Chinese display text', () => {
    expect(getBehaviorTreeAttributeDisplay('ID', 'MF_Entry')).toMatchObject({
      label: '子树标识',
      value: '梅林进门阶段',
    });

    expect(getBehaviorTreeAttributeDisplay('_autoremap', 'true')).toMatchObject({
      label: '自动映射',
      value: '是',
    });

    expect(getBehaviorTreeAttributeDisplay('code', "next_action=='GRAB'")).toMatchObject({
      label: '脚本',
      value: '判断下一步是否抓取',
    });
  });

  test('should build a Chinese-only editor structure preview for current trees', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['武馆区']);
    const preview = buildEditorTreePreview(document, 'MCAreaTree');

    expect(preview).toContain('当前决策树：武馆主任务');
    expect(preview).toContain('武馆顺序流程');
    expect(preview).toContain('取矛头');
    expect(preview).not.toMatch(/[A-Za-z]/);
  });
});
