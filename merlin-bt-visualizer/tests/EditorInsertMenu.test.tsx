/**
 * @vitest-environment jsdom
 */
import { afterEach, describe, expect, test, vi } from 'vitest';
import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { EditorInsertMenu } from '../src/components/EditorInsertMenu';
import { xmlToEditorDocument } from '../src/utils/editorParser';
import { behaviorTreeXmlByPhase } from '../src/utils/behaviorTreeSources';

describe('EditorInsertMenu', () => {
  afterEach(() => {
    cleanup();
  });

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

  test('renders template cards in a two-column scrollable panel', () => {
    const document = xmlToEditorDocument(behaviorTreeXmlByPhase['武馆区']);

    render(
      <EditorInsertMenu
        document={document}
        activeTreeId={document.trees[0]?.id ?? null}
        mode="floating"
        positionMode="choose"
        onInsert={() => undefined}
        onClose={() => undefined}
      />
    );

    const commonSectionItems = screen.getByTestId('editor-insert-section-items-common');
    expect(commonSectionItems.className).toContain('grid');
    expect(commonSectionItems.className).toContain('grid-cols-2');

    const templateButton = screen.getAllByRole('button', { name: /抓取矛头/ })[0];
    expect(templateButton.parentElement?.className).toContain('grid-cols-2');

    expect(screen.getByTestId('editor-insert-menu').className).toContain('h-[min(560px,72vh)]');
    expect(screen.getByTestId('editor-insert-menu-body').className).toContain('overflow-y-auto');
  });
});
