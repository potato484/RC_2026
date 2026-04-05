/**
 * @vitest-environment jsdom
 */
import { afterEach, beforeEach, describe, expect, test } from 'vitest';
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react';
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

  test('scopes source preview to the current active tree while keeping the root wrapper', () => {
    const document = xmlToEditorDocument(`
      <root BTCPP_format="4">
        <include path="shared_nodes.xml" />
        <BehaviorTree ID="TreeA" name="Primary">
          <Sequence>
            <GrabTip name="grab_tip" />
          </Sequence>
        </BehaviorTree>
        <BehaviorTree ID="TreeB" name="Secondary">
          <Fallback>
            <AssembleWeapon name="assemble" />
          </Fallback>
        </BehaviorTree>
      </root>
    `);
    const activeTree = document.trees[1];

    useEditorStore.setState({
      currentPhase: '武馆区',
      phaseDrafts: {},
      document,
      activeTreeId: activeTree.id,
      collapsedNodes: new Set(),
      flowNodes: [],
      flowEdges: [],
      selectedNodeId: activeTree.rootNode.children[0]?.id ?? activeTree.rootNode.id,
      canUndo: false,
      canRedo: false,
    });

    render(<EditorRightPanel />);

    fireEvent.click(screen.getByTestId('editor-tab-preview'));
    expect(screen.getByTestId('editor-structure-preview').textContent).toContain('执行组装');
    expect(screen.getByTestId('editor-structure-preview').textContent).not.toContain('取矛头');

    fireEvent.click(screen.getByText('源文件预览'));
    const sourcePreview = screen.getByTestId('editor-source-preview');
    expect(sourcePreview.textContent).toContain('<root BTCPP_format="4">');
    expect(sourcePreview.textContent).toContain('<include path="shared_nodes.xml"/>');
    expect(sourcePreview.textContent).toContain('<BehaviorTree ID="TreeB" name="Secondary">');
    expect(sourcePreview.textContent).toContain('assemble');
    expect(sourcePreview.textContent).not.toContain('<BehaviorTree ID="TreeA" name="Primary">');
    expect(sourcePreview.textContent).not.toContain('grab_tip');
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

    fireEvent.click(screen.getByTestId('editor-picker-trigger-along-branch-template'));
    const templatePicker = screen.getByTestId('editor-picker-along-branch-template');
    expect(within(templatePicker).queryByText('顺序节点')).toBeNull();
    expect(within(templatePicker).getByText('抓取矛头')).toBeTruthy();
    fireEvent.click(templatePicker);

    fireEvent.click(screen.getByTestId('editor-picker-trigger-along-branch-wrapper'));
    const fallbackButton = within(screen.getByTestId('editor-picker-along-branch-wrapper'))
      .getAllByRole('button')
      .find(
        (button) =>
          button.textContent?.includes('回退节点') &&
          button.textContent?.includes('Fallback') &&
          !button.textContent?.includes('ReactiveFallback')
      );

    expect(fallbackButton).toBeTruthy();
    fireEvent.click(
      fallbackButton as HTMLButtonElement
    );

    expect((insertButton as HTMLButtonElement).disabled).toBe(false);
  });

  test('opens overlay pickers for all structural choices in the right panel', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootId = useEditorStore.getState().document!.trees[0].rootNode.id;
    useEditorStore.getState().setSelectedNode(rootId);

    render(<EditorRightPanel />);

    const pickerCases = [
      {
        triggerId: 'editor-picker-trigger-insert-position',
        pickerId: 'editor-picker-insert-position',
      },
      {
        triggerId: 'editor-picker-trigger-along-branch-wrapper',
        pickerId: 'editor-picker-along-branch-wrapper',
      },
      {
        triggerId: 'editor-picker-trigger-along-branch-template',
        pickerId: 'editor-picker-along-branch-template',
      },
      {
        triggerId: 'editor-picker-trigger-branch-template',
        pickerId: 'editor-picker-branch-template',
      },
      {
        triggerId: 'editor-picker-trigger-wrap-template',
        pickerId: 'editor-picker-wrap-template',
      },
    ];

    pickerCases.forEach(({ triggerId, pickerId }) => {
      fireEvent.click(screen.getByTestId(triggerId));
      expect(screen.getByTestId(pickerId)).toBeTruthy();
      fireEvent.click(screen.getByTestId(pickerId));
      expect(screen.queryByTestId(pickerId)).toBeNull();
    });
  });

  test('renders node template pickers as two-column grids', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootId = useEditorStore.getState().document!.trees[0].rootNode.id;
    useEditorStore.getState().setSelectedNode(rootId);

    render(<EditorRightPanel />);

    fireEvent.click(screen.getByTestId('editor-picker-trigger-along-branch-template'));
    const alongBranchTemplatePicker = screen.getByTestId('editor-picker-along-branch-template');
    const alongBranchTemplateButton = within(alongBranchTemplatePicker)
      .getAllByRole('button')
      .find((button) => button.textContent?.includes('抓取矛头'));

    expect(alongBranchTemplateButton).toBeTruthy();
    expect(alongBranchTemplateButton?.parentElement?.className).toContain('grid-cols-2');
    fireEvent.click(alongBranchTemplatePicker);

    fireEvent.click(screen.getByTestId('editor-picker-trigger-branch-template'));
    const branchTemplatePicker = screen.getByTestId('editor-picker-branch-template');
    const branchTemplateButton = within(branchTemplatePicker)
      .getAllByRole('button')
      .find((button) => button.textContent?.includes('顺序节点'));

    expect(branchTemplateButton).toBeTruthy();
    expect(branchTemplateButton?.parentElement?.className).toContain('grid-cols-2');
  });

  test('selects a wrap template without mutating the tree until execute is clicked', () => {
    loadCustomDocument(`
      <root BTCPP_format="4">
        <BehaviorTree ID="WrapTree">
          <Sequence>
            <GrabTip />
          </Sequence>
        </BehaviorTree>
      </root>
    `);

    render(<EditorRightPanel />);

    fireEvent.click(screen.getByTestId('editor-picker-trigger-wrap-template'));
    const delayButton = within(screen.getByTestId('editor-picker-wrap-template'))
      .getAllByRole('button')
      .find((button) => button.textContent?.includes('Delay'));

    expect(delayButton).toBeTruthy();
    fireEvent.click(
      delayButton as HTMLButtonElement
    );

    expect(screen.queryByTestId('editor-picker-wrap-template')).toBeNull();
    expect(screen.getByTestId('editor-picker-trigger-wrap-template').textContent).toContain('延时装饰器');
    expect(useEditorStore.getState().document!.trees[0].rootNode.children[0].tagName).toBe('GrabTip');

    fireEvent.click(screen.getByRole('button', { name: '执行包裹' }));

    expect(useEditorStore.getState().document!.trees[0].rootNode.children[0].tagName).toBe('Delay');
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
