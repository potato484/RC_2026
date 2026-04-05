/**
 * @vitest-environment jsdom
 */
import { afterEach, beforeEach, describe, expect, test } from 'vitest';
import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { EditorKnowledgeBase } from '../src/components/EditorKnowledgeBase';
import { useEditorStore } from '../src/store/useEditorStore';
import { useStore } from '../src/store/useStore';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';

const resetEditorStore = () => {
  useEditorStore.setState({
    currentPhase: null,
    phaseDrafts: {},
    document: null,
    activeTreeId: null,
    collapsedNodes: new Set(),
    flowNodes: [],
    flowEdges: [],
    selectedNodeId: null,
    canUndo: false,
    canRedo: false,
  });
};

describe('EditorKnowledgeBase', () => {
  beforeEach(() => {
    resetEditorStore();
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);
    useStore.setState({ activePhase: '武馆区' });
  });

  afterEach(() => {
    cleanup();
  });

  test('defaults to the current phase category and searches only inside the chosen domain', () => {
    render(<EditorKnowledgeBase onRequestClose={() => undefined} />);

    expect(screen.getByTestId('editor-knowledge-active-category').textContent).toContain('武馆区模块');

    fireEvent.change(screen.getByTestId('editor-knowledge-base-search'), {
      target: { value: '目标俯仰角' },
    });

    expect(screen.getByTestId('editor-knowledge-empty').textContent).toContain('武馆区模块');

    fireEvent.click(screen.getByTestId('editor-knowledge-category-duel'));
    fireEvent.change(screen.getByTestId('editor-knowledge-base-search'), {
      target: { value: '目标俯仰角' },
    });

    expect(screen.getByRole('button', { name: /云台移动/ })).toBeTruthy();

    const detail = screen.getByTestId('editor-knowledge-detail');
    expect(detail.textContent).toContain('云台移动');
    expect(detail.textContent).toContain('一句话认识');
    expect(detail.textContent).toContain('什么时候用');
    expect(detail.textContent).toContain('运行结果怎么理解');
    expect(detail.textContent).toContain('参数怎么填');
    expect(detail.textContent).toContain('新手提示');
    expect(detail.textContent).toContain('一个典型用法');
    expect(detail.textContent).toContain('俯仰角');
    expect(detail.textContent).toContain('偏航角');
    expect(screen.queryByText('控制包装')).toBeNull();
    expect(screen.queryByText('在当前节点后插入')).toBeNull();
  });

  test('keeps official nodes in one domain and clears stale detail when the current domain has no matches', () => {
    render(<EditorKnowledgeBase onRequestClose={() => undefined} />);

    fireEvent.click(screen.getByTestId('editor-knowledge-category-official'));
    fireEvent.change(screen.getByTestId('editor-knowledge-base-search'), {
      target: { value: '结果反转' },
    });

    expect(screen.getByRole('button', { name: /结果反转/ })).toBeTruthy();
    expect(screen.getByTestId('editor-knowledge-detail').textContent).toContain('官方节点');

    fireEvent.change(screen.getByTestId('editor-knowledge-base-search'), {
      target: { value: '不存在的关键词' },
    });

    expect(screen.getByTestId('editor-knowledge-empty')).toBeTruthy();
    expect(screen.getByTestId('editor-knowledge-detail').textContent).not.toContain('结果反转');
  });
});
