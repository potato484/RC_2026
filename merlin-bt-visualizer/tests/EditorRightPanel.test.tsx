/**
 * @vitest-environment jsdom
 */
import { afterEach, beforeEach, describe, expect, test } from 'vitest';
import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { EditorRightPanel } from '../src/components/EditorRightPanel';
import { useEditorStore } from '../src/store/useEditorStore';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';
import { xmlToEditorDocument } from '../src/utils/editorParser';

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

const loadCustomDocument = (xmlContent: string) => {
  const document = xmlToEditorDocument(xmlContent);
  const selectedNodeId = document.trees[0]?.rootNode.children[0]?.id ?? document.trees[0]?.rootNode.id ?? null;

  useEditorStore.setState({
    currentPhase: '武馆区',
    phaseDrafts: {},
    document,
    activeTreeId: document.trees[0]?.id ?? null,
    collapsedNodes: new Set(),
    flowNodes: [],
    flowEdges: [],
    selectedNodeId,
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
    expect(screen.queryByText('复合节点切换')).toBeNull();

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

  test('shows numeric literal inputs for numeric-only nodes', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const checkManualRobotId =
      useEditorStore.getState().document!.trees[0].rootNode.children[1].children[0].children[0].id;
    useEditorStore.getState().setSelectedNode(checkManualRobotId);

    render(<EditorRightPanel />);

    expect(screen.getByText('已定义参数')).toBeTruthy();
    expect(screen.getByTestId('editor-port-field-distance_threshold')).toBeTruthy();
    expect(screen.getByTestId('editor-port-field-static_time')).toBeTruthy();
    expect(screen.queryByText('黑板')).toBeNull();
  });

  test('keeps only numeric literal ports and hides declared string and blackboard-capable ports', () => {
    loadCustomDocument(`
      <root BTCPP_format="4">
        <BehaviorTree ID="LegacyTree">
          <Sequence>
            <NavToTaskPose grid_id="@shared_grid" />
          </Sequence>
        </BehaviorTree>
      </root>
    `);

    render(<EditorRightPanel />);

    expect(screen.getByText('已定义参数')).toBeTruthy();
    expect(screen.getByTestId('editor-port-field-timeout_sec')).toBeTruthy();
    expect(screen.queryByTestId('editor-port-field-grid_id')).toBeNull();
    expect(screen.queryByTestId('editor-port-field-task_tag')).toBeNull();
    expect(screen.queryByText('保留的原始属性')).toBeNull();
  });

  test('shows legacy error_code bindings as readonly raw attributes instead of editable declared parameters', () => {
    loadCustomDocument(`
      <root BTCPP_format="4">
        <BehaviorTree ID="LegacyTree">
          <Sequence>
            <GrabTip error_code="{legacy_error}" timeout_sec="10" />
          </Sequence>
        </BehaviorTree>
      </root>
    `);

    render(<EditorRightPanel />);

    expect(screen.queryByText('错误码输出')).toBeNull();
    expect(screen.getByTestId('editor-port-field-timeout_sec')).toBeTruthy();
    expect(screen.getByText('保留的原始属性')).toBeTruthy();
    expect(screen.getByText('错误码')).toBeTruthy();
    expect(screen.getByText('{legacy_error}')).toBeTruthy();
  });

  test('hides string and bool-only SubTree ports instead of moving them to readonly attributes', () => {
    loadCustomDocument(`
      <root BTCPP_format="4">
        <BehaviorTree ID="LegacyTree">
          <Sequence>
            <SubTree ID="MF_Entry" _autoremap="true" />
          </Sequence>
        </BehaviorTree>
      </root>
    `);

    render(<EditorRightPanel />);

    expect(screen.queryByText('已定义参数')).toBeNull();
    expect(screen.queryByText('保留的原始属性')).toBeNull();
    expect(screen.queryByTestId('editor-port-field-ID')).toBeNull();
    expect(screen.queryByTestId('editor-port-field-_autoremap')).toBeNull();
  });
});
