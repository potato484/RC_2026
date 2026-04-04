/**
 * @vitest-environment jsdom
 */
import { beforeEach, describe, expect, test } from 'vitest';
import { useEditorStore } from '../src/store/useEditorStore';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';

describe('Editor store phase drafts', () => {
  beforeEach(() => {
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
  });

  test('should cache editor drafts per phase and restore them when switching back', () => {
    useEditorStore.getState().ensurePhaseLoaded('梅林区', behaviorTreeXmlByPhase['梅林区']);
    useEditorStore.getState().setActiveTree('MF_Exit');

    const mfExitTree = useEditorStore.getState().document?.trees.find((tree) => tree.id === 'MF_Exit');
    expect(mfExitTree).toBeTruthy();
    useEditorStore.getState().updateNodeAttributes(mfExitTree!.rootNode.id, {
      ...mfExitTree!.rootNode.attributes,
      test_marker: 'kept-in-draft',
    });

    useEditorStore.getState().ensurePhaseLoaded('对抗区', behaviorTreeXmlByPhase['对抗区']);
    expect(useEditorStore.getState().currentPhase).toBe('对抗区');
    expect(useEditorStore.getState().document?.trees[0].id).toBe('CombatAreaTree');

    useEditorStore.getState().ensurePhaseLoaded('梅林区', behaviorTreeXmlByPhase['梅林区']);
    const restoredState = useEditorStore.getState();
    const restoredTree = restoredState.document?.trees.find((tree) => tree.id === 'MF_Exit');

    expect(restoredState.currentPhase).toBe('梅林区');
    expect(restoredState.activeTreeId).toBe('MF_Exit');
    expect(restoredTree?.rootNode.attributes.test_marker).toBe('kept-in-draft');
  });

  test('should project translated display labels for editor flow nodes', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const displayLabels = useEditorStore
      .getState()
      .flowNodes
      .map((node) => String((node.data as { displayLabel?: string }).displayLabel ?? ''));

    expect(displayLabels.some((label) => label.includes('武馆顺序流程'))).toBe(true);
    expect(displayLabels.some((label) => label.includes('取矛头'))).toBe(true);
  });

  test('should insert and wrap nodes while preserving structure', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const initialTree = useEditorStore.getState().document!.trees[0];
    const firstChildId = initialTree.rootNode.children[0].id;

    useEditorStore.getState().insertAlongBranch(firstChildId, {
      position: 'after',
      wrapperTagName: 'Sequence',
      template: { tagName: 'Delay' },
    });
    let rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    expect(rootNode.children.map((child) => child.tagName)).toEqual(['Sequence', 'RetryUntilSuccessful']);
    expect(rootNode.children[0].children.map((child) => child.tagName)).toEqual(['GrabTip', 'Delay']);
    expect(rootNode.children[0].children[1].attributes.delay_msec).toBe('1000');

    useEditorStore.getState().wrapNode(firstChildId, 'Inverter');
    rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    expect(rootNode.children[0].tagName).toBe('Sequence');
    expect(rootNode.children[0].children[0].tagName).toBe('Inverter');
    expect(rootNode.children[0].children[0].children[0].tagName).toBe('GrabTip');
  });

  test('should only update parameters declared in the node registry', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().updateRegisteredAttribute(firstChild.id, 'timeout_sec', '12');
    useEditorStore.getState().updateRegisteredAttribute(firstChild.id, 'temporary_attr', 'blocked');

    const nextFirstChild = useEditorStore.getState().document!.trees[0].rootNode.children[0];
    expect(nextFirstChild.attributes.timeout_sec).toBe('12');
    expect(nextFirstChild.attributes.temporary_attr).toBeUndefined();
  });

  test('should always create an explicit wrapper node when inserting on an existing sequence branch', () => {
    useEditorStore.getState().ensurePhaseLoaded('梅林区', behaviorTreeXmlByPhase['梅林区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().insertAlongBranchOnEdge(rootNode.id, firstChild.id, {
      position: 'before',
      wrapperTagName: 'Sequence',
      template: { tagName: 'WaitVisionTarget' },
    });

    let nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Sequence');
    expect(nextRoot.children[0].children.map((child) => child.tagName)).toEqual([
      'WaitVisionTarget',
      firstChild.tagName,
    ]);
    expect(nextRoot.children).toHaveLength(rootNode.children.length);
  });

  test('should allow choosing a non-sequence wrapper when inserting on a branch edge', () => {
    useEditorStore.getState().ensurePhaseLoaded('对抗区', behaviorTreeXmlByPhase['对抗区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];
    const fallbackChild = firstChild.children[0];

    useEditorStore.getState().insertAlongBranchOnEdge(firstChild.id, fallbackChild.id, {
      position: 'before',
      wrapperTagName: 'Parallel',
      template: { tagName: 'WaitVisionTarget' },
    });

    const nextFallback = useEditorStore.getState().document!.trees[0].rootNode.children[0];
    expect(nextFallback.tagName).toBe('Fallback');
    expect(nextFallback.children[0].tagName).toBe('Parallel');
    expect(nextFallback.children[0].children.map((child) => child.tagName)).toEqual([
      'WaitVisionTarget',
      fallbackChild.tagName,
    ]);
    expect(nextFallback.children[0].attributes.success_count).toBe('-1');
    expect(nextFallback.children[0].attributes.failure_count).toBe('1');
  });

  test('should add a new branch only through the explicit branch insertion action', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    useEditorStore.getState().insertBranch(rootNode.id, 1, 'Delay');

    let nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children.map((child) => child.tagName)).toEqual([
      'GrabTip',
      'Delay',
      'RetryUntilSuccessful',
    ]);

    const retryNodeId = nextRoot.children[2].id;
    useEditorStore.getState().insertBranch(retryNodeId, 0, 'ForceSuccess');
    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[2].tagName).toBe('RetryUntilSuccessful');
    expect(nextRoot.children[2].children).toHaveLength(1);
  });

  test('should undo and redo structural edits within the same phase draft', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().insertAlongBranchOnEdge(rootNode.id, firstChild.id, {
      position: 'before',
      wrapperTagName: 'Fallback',
      template: { tagName: 'WaitVisionTarget' },
    });
    expect(useEditorStore.getState().canUndo).toBe(true);

    let nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Fallback');
    expect(nextRoot.children[0].children[0].tagName).toBe('WaitVisionTarget');

    useEditorStore.getState().undo();
    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe(firstChild.tagName);
    expect(useEditorStore.getState().canRedo).toBe(true);

    useEditorStore.getState().redo();
    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Fallback');
    expect(nextRoot.children[0].children.map((child) => child.tagName)).toEqual([
      'WaitVisionTarget',
      firstChild.tagName,
    ]);
  });
});
