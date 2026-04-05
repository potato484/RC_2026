/**
 * @vitest-environment jsdom
 */
import { describe, expect, test, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';
import { EditorOverlayPanel } from '../src/components/EditorOverlayPanel';

describe('EditorOverlayPanel', () => {
  test('renders layered header, search box and close button', () => {
    const onClose = vi.fn();
    const onSearchChange = vi.fn();

    render(
      <EditorOverlayPanel
        title="选择控制包装"
        hint="这里先定执行关系，真正插入仍由下方按钮触发。"
        closeLabel="关闭选择控制包装"
        onClose={onClose}
        search={{
          value: '',
          placeholder: '搜索节点',
          onChange: onSearchChange,
        }}
      >
        <div>body</div>
      </EditorOverlayPanel>
    );

    expect(screen.getByText('选择控制包装')).toBeTruthy();
    expect(screen.getByText('这里先定执行关系，真正插入仍由下方按钮触发。')).toBeTruthy();
    expect(screen.getByPlaceholderText('搜索节点')).toBeTruthy();

    fireEvent.change(screen.getByPlaceholderText('搜索节点'), {
      target: { value: 'fallback' },
    });
    expect(onSearchChange).toHaveBeenCalledWith('fallback');

    fireEvent.click(screen.getByRole('button', { name: '关闭选择控制包装' }));
    expect(onClose).toHaveBeenCalled();
  });
});
