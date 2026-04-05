/*
 * @Author: potato484 2220362462@qq.com
 * @Date: 2026-03-29 21:14:09
 * @LastEditors: potato484 2220362462@qq.com
 * @LastEditTime: 2026-03-29 21:18:17
 * @FilePath: /RC_2026/merlin-bt-visualizer/tests/editorParser.test.ts
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @vitest-environment jsdom
 */
import { expect, test, describe } from 'vitest';
import { xmlToEditorDocument } from '../src/utils/editorParser';
import { editorDocumentToActiveTreePreviewXml, editorDocumentToXml } from '../src/utils/editorSerializer';
import * as fs from 'fs';
import * as path from 'path';

describe('Editor Parser and Serializer', () => {
  const mcTreeXml = `
<root BTCPP_format="4">
  <BehaviorTree ID="MCAreaTree">
    <Sequence name="MC_Sequence">
      <GrabTip name="grab_tip"/>
      <RetryUntilSuccessful num_attempts="10">
        <Sequence>
          <CheckManualRobot distance_threshold="0.5" static_time="2.0"/>
          <AssembleWeapon name="assemble"/>
        </Sequence>
      </RetryUntilSuccessful>
    </Sequence>
  </BehaviorTree>
</root>
`.trim();

  test('should parse XML into EditorDocument', () => {
    // Note: DOMParser is only available in browser environments, 
    // so this test assumes it's run in a JSDOM environment by Vitest
    const doc = xmlToEditorDocument(mcTreeXml);
    
    expect(doc.rootAttributes.BTCPP_format).toBe("4");
    expect(doc.trees.length).toBe(1);
    expect(doc.trees[0].id).toBe("MCAreaTree");
    
    const rootNode = doc.trees[0].rootNode;
    expect(rootNode.tagName).toBe("Sequence");
    expect(rootNode.attributes.name).toBe("MC_Sequence");
    expect(rootNode.children.length).toBe(2);
    
    const grabTip = rootNode.children[0];
    expect(grabTip.tagName).toBe("GrabTip");
    expect(grabTip.attributes.name).toBe("grab_tip");
    expect(rootNode.nodeKind).toBe('control');
    expect(rootNode.uiType).toBe('control');
    expect(grabTip.definitionId).toBe('GrabTip');
    expect(grabTip.nodeKind).toBe('action');
    expect(grabTip.source).toBe('robot');
    expect(grabTip.uiType).toBe('leaf');
    expect(grabTip.portBindings.name.bindingValue).toBe('grab_tip');
    expect(grabTip.portBindings.timeout_sec).toBeTruthy();
  });

  test('should serialize EditorDocument back to XML', () => {
    const doc = xmlToEditorDocument(mcTreeXml);
    const serializedXml = editorDocumentToXml(doc);
    
    // We parse it again to verify semantic equivalence since 
    // exact string matching is brittle with formatting
    const doc2 = xmlToEditorDocument(serializedXml);
    
    expect(doc2.rootAttributes.BTCPP_format).toBe("4");
    expect(doc2.trees.length).toBe(1);
    expect(doc2.trees[0].id).toBe("MCAreaTree");
    
    const rootNode = doc2.trees[0].rootNode;
    expect(rootNode.tagName).toBe("Sequence");
    expect(rootNode.attributes.name).toBe("MC_Sequence");
    expect(rootNode.children.length).toBe(2);
  });

  test('should serialize only the active tree for XML preview while keeping the root wrapper', () => {
    const multiTreeXml = `
<root BTCPP_format="4">
  <include path="shared_nodes.xml"/>
  <BehaviorTree ID="TreeA" name="Primary">
    <Sequence>
      <GrabTip name="grab_tip"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="TreeB">
    <Fallback>
      <AssembleWeapon name="assemble"/>
    </Fallback>
  </BehaviorTree>
</root>
`.trim();
    const doc = xmlToEditorDocument(multiTreeXml);

    const previewXml = editorDocumentToActiveTreePreviewXml(doc, 'TreeB');
    expect(previewXml).not.toBeNull();
    if (!previewXml) {
      throw new Error('Expected active tree preview XML to be available');
    }

    expect(previewXml).toContain('<root BTCPP_format="4">');
    expect(previewXml).toContain('<include path="shared_nodes.xml"/>');
    expect(previewXml).toContain('<BehaviorTree ID="TreeB">');
    expect(previewXml).toContain('<Fallback>');
    expect(previewXml).not.toContain('<BehaviorTree ID="TreeA" name="Primary">');
    expect(previewXml).not.toContain('grab_tip');
  });

  test('should return null when previewing a missing active tree', () => {
    const doc = xmlToEditorDocument(mcTreeXml);

    expect(editorDocumentToActiveTreePreviewXml(doc, 'UnknownTree')).toBeNull();
    expect(editorDocumentToActiveTreePreviewXml(doc, null)).toBeNull();
  });

  const testRoundTrip = (xmlContent: string) => {
    const doc = xmlToEditorDocument(xmlContent);
    const serializedXml = editorDocumentToXml(doc);
    const doc2 = xmlToEditorDocument(serializedXml);
    
    // Check root attributes
    expect(doc2.rootAttributes).toEqual(doc.rootAttributes);
    // Check includes
    expect(doc2.includes).toEqual(doc.includes);
    // Check trees count
    expect(doc2.trees.length).toBe(doc.trees.length);
    
    // Recursively check trees
    for (let i = 0; i < doc.trees.length; i++) {
      expect(doc2.trees[i].id).toBe(doc.trees[i].id);
      expect(doc2.trees[i].name).toBe(doc.trees[i].name);
      
      const checkNode = (n1: any, n2: any) => {
        expect(n2.tagName).toBe(n1.tagName);
        expect(n2.attributes).toEqual(n1.attributes);
        expect(n2.children.length).toBe(n1.children.length);
        for (let j = 0; j < n1.children.length; j++) {
          checkNode(n1.children[j], n2.children[j]);
        }
      };
      checkNode(doc.trees[i].rootNode, doc2.trees[i].rootNode);
    }
  };

  test('should round-trip combat_tree.xml correctly', () => {
    const combatTreePath = path.resolve(process.cwd(), '../src/rc26_decision/behavior_trees/combat_tree.xml');
    const combatTreeXml = fs.readFileSync(combatTreePath, 'utf-8');
    testRoundTrip(combatTreeXml);
  });

  test('should round-trip mf_tree.xml correctly', () => {
    const mfTreePath = path.resolve(process.cwd(), '../src/rc26_decision/behavior_trees/mf_tree.xml');
    const mfTreeXml = fs.readFileSync(mfTreePath, 'utf-8');
    testRoundTrip(mfTreeXml);
  });
});
