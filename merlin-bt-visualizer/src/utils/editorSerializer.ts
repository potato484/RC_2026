import { EditorDocument, EditorNode, EditorTree } from '../types/editor';

/**
 * Serializes an EditorDocument model back into an XML string.
 * It strictly preserves structural and semantic details for round-tripping.
 */
export function editorDocumentToXml(doc: EditorDocument): string {
  return serializeEditorDocument(doc, doc.trees);
}

/**
 * Builds a complete XML document preview for the current active tree only.
 * It keeps the root wrapper and include declarations, but narrows BehaviorTree
 * entries to the selected tree so the preview scope matches the structure view.
 */
export function editorDocumentToActiveTreePreviewXml(
  doc: EditorDocument,
  activeTreeId: string | null
): string | null {
  if (!activeTreeId) {
    return null;
  }

  const activeTree = doc.trees.find((tree) => tree.id === activeTreeId);
  if (!activeTree) {
    return null;
  }

  return serializeEditorDocument(doc, [activeTree]);
}

function serializeEditorDocument(doc: EditorDocument, trees: EditorTree[]): string {
  const xmlDoc = document.implementation.createDocument(null, 'root', null);
  const rootElement = xmlDoc.documentElement;

  // Add root attributes
  for (const [key, value] of Object.entries(doc.rootAttributes)) {
    rootElement.setAttribute(key, value);
  }

  // Add includes
  for (const include of doc.includes) {
    const includeElement = xmlDoc.createElement('include');
    includeElement.setAttribute('path', include);
    rootElement.appendChild(includeElement);
  }

  // Add BehaviorTrees
  for (const tree of trees) {
    rootElement.appendChild(serializeEditorTree(tree, xmlDoc));
  }

  // Use XMLSerializer to convert to string
  const serializer = new XMLSerializer();
  let xmlString = serializer.serializeToString(xmlDoc);

  // Apply some basic formatting to match the compact style
  return formatXml(xmlString);
}

function serializeEditorTree(tree: EditorTree, xmlDoc: Document): Element {
  const treeElement = xmlDoc.createElement('BehaviorTree');
  treeElement.setAttribute('ID', tree.id);
  if (tree.name) {
    treeElement.setAttribute('name', tree.name);
  }

  const rootNodeElement = serializeEditorNode(tree.rootNode, xmlDoc);
  treeElement.appendChild(rootNodeElement);

  return treeElement;
}

function serializeEditorNode(node: EditorNode, xmlDoc: Document): Element {
  const element = xmlDoc.createElement(node.tagName);

  for (const [key, value] of Object.entries(node.attributes)) {
    element.setAttribute(key, value);
  }

  for (const childNode of node.children) {
    element.appendChild(serializeEditorNode(childNode, xmlDoc));
  }

  return element;
}

/**
 * Basic XML formatting to add newlines and indentation.
 */
function formatXml(xml: string): string {
  // A simple formatter for the XML output
  let formatted = '';
  let indent = 0;
  const tab = '  ';
  
  // Split by tags
  const tags = xml.split(/(<[^>]+>)/g).filter(tag => tag.trim() !== '');
  
  for (let i = 0; i < tags.length; i++) {
    const tag = tags[i];
    
    if (tag.match(/^<\?xml/)) {
      formatted += tag + '\n';
    } else if (tag.match(/^<!--/)) {
      formatted += tab.repeat(indent) + tag + '\n';
    } else if (tag.match(/^<\/[^>]+>$/)) {
      // Closing tag
      indent = Math.max(0, indent - 1);
      // If previous tag was the opening tag of this element (i.e. empty element), don't indent
      if (i > 0 && tags[i-1].match(/^<[^/][^>]*>$/) && !tags[i-1].match(/\/>$/) && tags[i-1].replace(/<([^ >]+).*/, '$1') === tag.replace(/<\/?([^ >]+).*/, '$1')) {
        // Remove the newline from the previous tag
        formatted = formatted.replace(/\n$/, '');
        formatted += tag + '\n';
      } else {
        formatted += tab.repeat(indent) + tag + '\n';
      }
    } else if (tag.match(/^<[^>]+\/>$/)) {
      // Self-closing tag
      formatted += tab.repeat(indent) + tag + '\n';
    } else if (tag.match(/^<[^>]+>$/)) {
      // Opening tag
      formatted += tab.repeat(indent) + tag + '\n';
      indent++;
    } else {
      // Text node
      formatted += tab.repeat(indent) + tag.trim() + '\n';
    }
  }
  
  return formatted.trim();
}
