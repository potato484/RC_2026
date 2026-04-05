/**
 * @vitest-environment jsdom
 */
import { describe, expect, test, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';
import { EditorOptionPicker } from '../src/components/EditorOptionPicker';

describe('EditorOptionPicker', () => {
  test('filters searchable options and closes from the backdrop', () => {
    const onSelect = vi.fn();
    const onClose = vi.fn();

    render(
      <EditorOptionPicker
        title="选择节点模板"
        description="测试用选择器"
        searchPlaceholder="搜索节点"
        dataTestId="editor-option-picker-test"
        selectedId="grab"
        sections={[
          {
            id: 'actions',
            title: '动作节点',
            items: [
              {
                id: 'grab',
                label: '抓取矛头',
                description: '执行一次抓取动作。',
                searchTokens: ['GrabTip', '抓取'],
              },
              {
                id: 'wait',
                label: '等待识别',
                description: '等待视觉结果返回。',
                searchTokens: ['WaitVision', '识别'],
              },
            ],
          },
        ]}
        onSelect={onSelect}
        onClose={onClose}
      />
    );

    fireEvent.change(screen.getByPlaceholderText('搜索节点'), {
      target: { value: 'vision' },
    });

    expect(screen.queryByText('抓取矛头')).toBeNull();
    expect(screen.getByText('等待识别')).toBeTruthy();

    fireEvent.click(screen.getByRole('button', { name: /等待识别/ }));
    expect(onSelect).toHaveBeenCalledWith('wait');

    fireEvent.click(screen.getByTestId('editor-option-picker-test'));
    expect(onClose).toHaveBeenCalled();
  });
});
