/**
 * @vitest-environment jsdom
 */
import { beforeEach, describe, expect, test } from 'vitest';
import { useStore } from '../src/store/useStore';
import { parseBTXml } from '../src/utils/btParser';
import {
  getBehaviorTreeXmlForPhase,
  resetBehaviorTreeXmlSources
} from '../src/utils/behaviorTreeSources';

const resetViewerState = () => {
  resetBehaviorTreeXmlSources();

  useStore.getState().replacePhaseXml('梅林区', getBehaviorTreeXmlForPhase('梅林区'));
  useStore.getState().replacePhaseXml('武馆区', getBehaviorTreeXmlForPhase('武馆区'));
  useStore.getState().replacePhaseXml('对抗区', getBehaviorTreeXmlForPhase('对抗区'));

  const initialArea = parseBTXml(getBehaviorTreeXmlForPhase('梅林区'));
  const initialTreeId = initialArea.mainTreeId || Object.keys(initialArea.trees)[0];
  const initialTree = initialArea.trees[initialTreeId] || { nodes: [], edges: [] };

  useStore.setState({
    appMode: 'viewer',
    isSimulating: true,
    isPlaying: false,
    activePhase: '梅林区',
    activeTreeId: initialTreeId,
    trees: initialArea.trees,
    nodes: initialTree.nodes,
    edges: initialTree.edges,
    activeNodeId: null,
    timeline: [],
    blackboard: [],
  });
};

describe('Viewer store XML refresh', () => {
  beforeEach(() => {
    resetViewerState();
  });

  test('should refresh active phase data after saving updated XML', () => {
    useStore.getState().setActivePhase('武馆区');

    const updatedXml = getBehaviorTreeXmlForPhase('武馆区').replace('num_attempts="10"', 'num_attempts="3"');
    useStore.getState().replacePhaseXml('武馆区', updatedXml);

    const hasUpdatedRetryDisplay = useStore.getState().nodes.some((node) => {
      if (node.label.includes('3次') || node.desc.includes('3次')) return true;
      return node.decorators?.some((decorator) => decorator.label.includes('3次') || decorator.desc.includes('3次')) ?? false;
    });

    expect(getBehaviorTreeXmlForPhase('武馆区')).toContain('num_attempts="3"');
    expect(hasUpdatedRetryDisplay).toBe(true);
  });
});
