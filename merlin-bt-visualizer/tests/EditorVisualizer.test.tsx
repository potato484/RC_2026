/**
 * @vitest-environment jsdom
 */
import { beforeEach, describe, expect, test, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';

vi.mock('@xyflow/react', () => ({
  ReactFlow: ({ children }: { children?: any }) => <div data-testid="react-flow">{children}</div>,
  Background: () => null,
  Controls: () => null,
  Edge: () => null,
  Node: () => null,
  MarkerType: {
    ArrowClosed: 'arrowclosed',
  },
}));

import { EditorVisualizer } from '../src/components/EditorVisualizer';
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

const hasNodeId = (nodeId: string | null, node: { id: string; children: Array<{ id: string; children: any[] }> }): boolean => {
  if (!nodeId) {
    return false;
  }
  if (node.id === nodeId) {
    return true;
  }
  return node.children.some((child) => hasNodeId(nodeId, child));
};

describe('EditorVisualizer keyboard shortcuts', () => {
  beforeEach(() => {
    resetEditorStore();
    useStore.setState({ activePhase: '武馆区' });
  });

  test('does not delete the selected node when Backspace is pressed inside a form field', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const firstChildId = useEditorStore.getState().document!.trees[0].rootNode.children[0].id;
    useEditorStore.getState().setSelectedNode(firstChildId);

    render(
      <div>
        <input data-testid="external-editor-input" defaultValue="12" />
        <EditorVisualizer />
      </div>
    );

    const input = screen.getByTestId('external-editor-input');
    input.focus();
    fireEvent.keyDown(input, { key: 'Backspace', bubbles: true });

    expect(
      hasNodeId(firstChildId, useEditorStore.getState().document!.trees[0].rootNode)
    ).toBe(true);
  });

  test('still deletes the selected node when Delete is pressed outside editable fields', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const firstChildId = useEditorStore.getState().document!.trees[0].rootNode.children[0].id;
    useEditorStore.getState().setSelectedNode(firstChildId);

    render(<EditorVisualizer />);

    fireEvent.keyDown(window, { key: 'Delete' });

    expect(
      hasNodeId(firstChildId, useEditorStore.getState().document!.trees[0].rootNode)
    ).toBe(false);
  });

  test('does not switch composite node types when T is pressed anymore', () => {
    useEditorStore.getState().ensurePhaseLoaded('武馆区', behaviorTreeXmlByPhase['武馆区']);

    const rootId = useEditorStore.getState().document!.trees[0].rootNode.id;
    useEditorStore.getState().setSelectedNode(rootId);

    render(<EditorVisualizer />);

    fireEvent.keyDown(window, { key: 't' });

    expect(useEditorStore.getState().document!.trees[0].rootNode.tagName).toBe('Sequence');
  });
});
