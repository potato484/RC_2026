/**
 * @vitest-environment jsdom
 */
import { afterEach, beforeEach, describe, expect, test } from 'vitest';
import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { EditorRightPanel } from '../src/components/EditorRightPanel';
import { useEditorStore } from '../src/store/useEditorStore';
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

describe('EditorRightPanel', () => {
  beforeEach(() => {
    resetEditorStore();
  });

  afterEach(() => {
    cleanup();
  });

  test('renders preformatted previews and removes arbitrary attribute editing', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);
    const firstChildId = useEditorStore.getState().document!.trees[0].rootNode.children[0].id;
    useEditorStore.getState().setSelectedNode(firstChildId);

    render(<EditorRightPanel />);

    expect(screen.queryByText('添加附加属性')).toBeNull();
    expect(screen.queryByTestId('add-attribute-button')).toBeNull();
    expect(screen.getByText('保留的原始属性')).toBeTruthy();

    fireEvent.click(screen.getByTestId('editor-tab-preview'));
    const preview = screen.getByTestId('editor-structure-preview');
    expect(preview.tagName).toBe('PRE');
    expect(preview.textContent).toContain('\n');

    fireEvent.click(screen.getByText('源文件预览'));
    const sourcePreview = screen.getByTestId('editor-source-preview');
    expect(sourcePreview.tagName).toBe('PRE');
    expect(sourcePreview.textContent).toContain('\n');
    expect(sourcePreview.textContent).toContain('<root');
  });

  test('shows add-branch controls only for eligible control nodes', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootId = useEditorStore.getState().document!.trees[0].rootNode.id;
    useEditorStore.getState().setSelectedNode(rootId);
    const { rerender } = render(<EditorRightPanel />);
    expect(screen.queryByText('新增支线')).toBeTruthy();

    const firstChildId = useEditorStore.getState().document!.trees[0].rootNode.children[0].id;
    useEditorStore.getState().setSelectedNode(firstChildId);
    rerender(<EditorRightPanel />);
    expect(screen.queryByText('新增支线')).toBeNull();
  });

  test('requires choosing a wrapper control for along-branch insertion and hides control templates there', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootId = useEditorStore.getState().document!.trees[0].rootNode.id;
    useEditorStore.getState().setSelectedNode(rootId);

    render(<EditorRightPanel />);

    const insertButton = screen.getByRole('button', { name: '执行同支线插入' });
    expect((insertButton as HTMLButtonElement).disabled).toBe(true);

    const templateSelect = screen.getByTestId('editor-along-branch-template-select') as HTMLSelectElement;
    expect(Array.from(templateSelect.options).some((option) => option.text.includes('顺序节点'))).toBe(false);
    expect(Array.from(templateSelect.options).some((option) => option.text.includes('动作'))).toBe(true);

    fireEvent.change(screen.getByTestId('editor-along-branch-wrapper-select'), {
      target: { value: 'Fallback' },
    });

    expect((insertButton as HTMLButtonElement).disabled).toBe(false);
  });
});
