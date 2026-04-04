/**
 * @vitest-environment jsdom
 */
import { describe, expect, test, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';
import { EditorInsertMenu } from '../src/components/EditorInsertMenu';
import { xmlToEditorDocument } from '../src/utils/editorParser';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';

describe('EditorInsertMenu', () => {
  test('toolbar mode requires explicitly choosing before or after', () => {
    const onInsert = vi.fn();
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['武馆区']);

    render(
      <EditorInsertMenu
        document={document}
        activeTreeId={document.trees[0]?.id ?? null}
        mode="floating"
        positionMode="choose"
        onInsert={onInsert}
        onClose={() => undefined}
      />
    );

    fireEvent.click(screen.getByRole('button', { name: /^回退节点/ }));
    fireEvent.click(screen.getAllByRole('button', { name: /抓取矛头/ })[0]);
    expect(onInsert).not.toHaveBeenCalled();

    fireEvent.click(screen.getByTestId('editor-insert-position-before'));
    fireEvent.click(screen.getAllByRole('button', { name: /抓取矛头/ })[0]);

    expect(onInsert).toHaveBeenCalledWith({
      position: 'before',
      wrapperTagName: 'Fallback',
      template: { tagName: 'GrabTip' },
    });
  });
});
