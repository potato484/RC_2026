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

  test('should insert, wrap and replace nodes while preserving structure', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const initialTree = useEditorStore.getState().document!.trees[0];
    const rootId = initialTree.rootNode.id;
    const firstChildId = initialTree.rootNode.children[0].id;

    useEditorStore.getState().insertNode(firstChildId, 'after', 'Delay');
    let rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    expect(rootNode.children.map((child) => child.tagName)).toEqual(['GrabTip', 'Delay', 'RetryUntilSuccessful']);
    expect(rootNode.children[1].attributes.delay_msec).toBe('1000');

    useEditorStore.getState().insertNode(firstChildId, 'wrap', 'Inverter');
    rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    expect(rootNode.children[0].tagName).toBe('Inverter');
    expect(rootNode.children[0].children[0].tagName).toBe('GrabTip');

    useEditorStore.getState().replaceNodeType(rootId, 'Fallback');
    rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    expect(rootNode.tagName).toBe('Fallback');
    expect(rootNode.children[0].tagName).toBe('Inverter');
  });

  test('should cycle composite node types without losing existing children', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const initialTree = useEditorStore.getState().document!.trees[0];
    const rootId = initialTree.rootNode.id;
    const childIds = initialTree.rootNode.children.map((child) => child.id);

    useEditorStore.getState().cycleCompositeType(rootId);

    const cycledRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(cycledRoot.tagName).toBe('SequenceWithMemory');
    expect(cycledRoot.children.map((child) => child.id)).toEqual(childIds);
  });

  test('should insert leaf and subtree templates on an edge with auto sequence bridge', () => {
    useEditorStore.getState().ensurePhaseLoaded('梅林区', behaviorTreeXmlByPhase['梅林区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().insertNodeOnEdge(rootNode.id, firstChild.id, { tagName: 'WaitVisionTarget' });

    let nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Sequence');
    expect(nextRoot.children[0].children.map((child) => child.tagName)).toEqual(['WaitVisionTarget', firstChild.tagName]);

    const bridgedNode = nextRoot.children[0].children[0];
    expect(bridgedNode.tagName).toBe('WaitVisionTarget');

    useEditorStore.getState().insertNodeOnEdge(nextRoot.id, nextRoot.children[1].id, {
      tagName: 'SubTree',
      presetAttributes: { ID: 'MF_Loop', _autoremap: 'true' },
    });

    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[1].tagName).toBe('Sequence');
    expect(nextRoot.children[1].children[0].tagName).toBe('SubTree');
    expect(nextRoot.children[1].children[0].attributes.ID).toBe('MF_Loop');
    expect(nextRoot.children[1].children[0].attributes._autoremap).toBe('true');
  });

  test('should insert control nodes on an edge without creating an extra bridge', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().insertNodeOnEdge(rootNode.id, firstChild.id, { tagName: 'Fallback' });

    const nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Fallback');
    expect(nextRoot.children[0].children).toHaveLength(1);
    expect(nextRoot.children[0].children[0].tagName).toBe(firstChild.tagName);
  });

  test('should undo and redo structural edits within the same phase draft', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootNode = useEditorStore.getState().document!.trees[0].rootNode;
    const firstChild = rootNode.children[0];

    useEditorStore.getState().insertNodeOnEdge(rootNode.id, firstChild.id, { tagName: 'WaitVisionTarget' });
    expect(useEditorStore.getState().canUndo).toBe(true);

    let nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Sequence');

    useEditorStore.getState().undo();
    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe(firstChild.tagName);
    expect(useEditorStore.getState().canRedo).toBe(true);

    useEditorStore.getState().redo();
    nextRoot = useEditorStore.getState().document!.trees[0].rootNode;
    expect(nextRoot.children[0].tagName).toBe('Sequence');
    expect(nextRoot.children[0].children[0].tagName).toBe('WaitVisionTarget');
  });
});
