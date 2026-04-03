import { MouseEvent } from 'react';
import { getCompositeSwitchCandidates } from '../utils/btRegistry';

interface EditorContextMenuProps {
  nodeId: string;
  tagName: string;
  x: number;
  y: number;
  onClose: () => void;
  onToggleCollapse: (nodeId: string) => void;
  onDelete: (nodeId: string) => void;
  onReplace: (nodeId: string, tagName: string) => void;
  onWrapInverter: (nodeId: string) => void;
  onWrapRetry: (nodeId: string) => void;
}

export const EditorContextMenu = ({
  nodeId,
  tagName,
  x,
  y,
  onClose,
  onToggleCollapse,
  onDelete,
  onReplace,
  onWrapInverter,
  onWrapRetry,
}: EditorContextMenuProps) => {
  const switchCandidates = getCompositeSwitchCandidates(tagName);

  const handleClick = (event: MouseEvent<HTMLButtonElement>, callback: () => void) => {
    event.stopPropagation();
    callback();
    onClose();
  };

  return (
    <div
      className="absolute z-30 min-w-[220px] rounded-2xl border border-slate-200 bg-white/95 p-2 shadow-2xl backdrop-blur"
      style={{ left: x, top: y }}
      onContextMenu={(event) => event.preventDefault()}
    >
      <button
        type="button"
        onClick={(event) => handleClick(event, () => onToggleCollapse(nodeId))}
        className="w-full rounded-xl px-3 py-2 text-left text-sm font-medium text-slate-700 transition hover:bg-slate-100"
      >
        折叠 / 展开
      </button>

      {switchCandidates.map((candidate) => (
        <button
          key={candidate.tagName}
          type="button"
          onClick={(event) => handleClick(event, () => onReplace(nodeId, candidate.tagName))}
          className="w-full rounded-xl px-3 py-2 text-left text-sm font-medium text-slate-700 transition hover:bg-slate-100"
        >
          切换为 {candidate.labelZh}
        </button>
      ))}

      <button
        type="button"
        onClick={(event) => handleClick(event, () => onWrapInverter(nodeId))}
        className="w-full rounded-xl px-3 py-2 text-left text-sm font-medium text-slate-700 transition hover:bg-slate-100"
      >
        包裹为结果反转
      </button>
      <button
        type="button"
        onClick={(event) => handleClick(event, () => onWrapRetry(nodeId))}
        className="w-full rounded-xl px-3 py-2 text-left text-sm font-medium text-slate-700 transition hover:bg-slate-100"
      >
        包裹为重试直到成功
      </button>
      <button
        type="button"
        onClick={(event) => handleClick(event, () => onDelete(nodeId))}
        className="w-full rounded-xl px-3 py-2 text-left text-sm font-medium text-rose-600 transition hover:bg-rose-50"
      >
        删除节点
      </button>
    </div>
  );
};
