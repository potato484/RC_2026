import mfTreeXml from '../../../src/rc26_decision/behavior_trees/mf_tree.xml?raw';
import combatTreeXml from '../../../src/rc26_decision/behavior_trees/combat_tree.xml?raw';
import mcTreeXml from '../../../src/rc26_decision/behavior_trees/mc_tree.xml?raw';

export type BehaviorTreePhase = '武馆区' | '梅林区' | '对抗区';

const initialBehaviorTreeXmlByPhase: Record<BehaviorTreePhase, string> = {
  '梅林区': mfTreeXml,
  '武馆区': mcTreeXml,
  '对抗区': combatTreeXml,
};

export const behaviorTreeXmlByPhase: Record<BehaviorTreePhase, string> = {
  ...initialBehaviorTreeXmlByPhase,
};

export function getBehaviorTreeXmlForPhase(phase: BehaviorTreePhase): string {
  return behaviorTreeXmlByPhase[phase];
}

export function setBehaviorTreeXmlForPhase(phase: BehaviorTreePhase, xmlContent: string): void {
  behaviorTreeXmlByPhase[phase] = xmlContent;
}

export function resetBehaviorTreeXmlSources(): void {
  (Object.keys(initialBehaviorTreeXmlByPhase) as BehaviorTreePhase[]).forEach((phase) => {
    behaviorTreeXmlByPhase[phase] = initialBehaviorTreeXmlByPhase[phase];
  });
}
