import { MouseEvent } from 'react';
import { BaseEdge, Edge, EdgeLabelRenderer, EdgeProps, getSmoothStepPath } from '@xyflow/react';
import { Plus } from 'lucide-react';
import { useEditorStore } from '../store/useEditorStore';
import { EditorAlongBranchInsertRequest } from '../types/editor';
import { EditorFlowEdgeData } from '../utils/editorProjection';
import { EditorInsertMenu } from './EditorInsertMenu';

interface EditorInsertEdgeRuntimeData extends EditorFlowEdgeData {
  isMenuOpen?: boolean;
  onToggleMenu?: (edgeId: string | null) => void;
  onInsertTemplate?: (
    parentNodeId: string,
    childNodeId: string,
    request: EditorAlongBranchInsertRequest
  ) => void;
}

type EditorInsertEdgeType = Edge<EditorInsertEdgeRuntimeData, 'editorInsertEdge'>;

export const EditorInsertEdge = ({
  id,
  source,
  target,
  data,
  sourceX,
  sourceY,
  targetX,
  targetY,
  sourcePosition,
  targetPosition,
  markerEnd,
  style,
}: EdgeProps<EditorInsertEdgeType>) => {
  const document = useEditorStore((state) => state.document);
  const activeTreeId = useEditorStore((state) => state.activeTreeId);
  const [edgePath, labelX, labelY] = getSmoothStepPath({
    sourceX,
    sourceY,
    targetX,
    targetY,
    sourcePosition,
    targetPosition,
  });

  const handleOpen = (event: MouseEvent) => {
    event.stopPropagation();
    data?.onToggleMenu?.(id);
  };

  const handleInsert = (request: EditorAlongBranchInsertRequest) => {
    data?.onInsertTemplate?.(source, target, request);
    data?.onToggleMenu?.(null);
  };

  return (
    <>
      <BaseEdge
        id={id}
        path={edgePath}
        markerEnd={markerEnd}
        style={style}
        interactionWidth={28}
        onClick={(event) => {
          event.stopPropagation();
          data?.onToggleMenu?.(id);
        }}
      />

      <EdgeLabelRenderer>
        <div
          className="pointer-events-none absolute inset-0 z-40"
          data-edge-source={source}
          data-edge-target={target}
          style={{
            transform: `translate(${labelX}px, ${labelY}px)`,
          }}
        >
          <button
            type="button"
            onClick={handleOpen}
            className="nodrag nopan pointer-events-auto absolute left-1/2 top-1/2 z-50 flex h-7 w-7 -translate-x-1/2 -translate-y-1/2 items-center justify-center rounded-full border border-slate-200/90 bg-white/88 text-slate-500 opacity-25 shadow-md transition hover:border-sky-300 hover:text-sky-700 hover:opacity-100 focus:opacity-100"
            aria-label="沿当前支线插入节点"
            data-testid="editor-edge-insert-trigger"
          >
            <Plus className="h-4 w-4" />
          </button>

          {data?.isMenuOpen && (
            <>
              <div className="nodrag nopan pointer-events-auto absolute left-1/2 top-1/2 z-40 hidden -translate-x-1/2 lg:block">
                <div className="translate-y-6">
                  <EditorInsertMenu
                    document={document}
                    activeTreeId={activeTreeId}
                    mode="floating"
                    position="before"
                    onInsert={handleInsert}
                    onClose={() => data.onToggleMenu?.(null)}
                  />
                </div>
              </div>

              <div className="nodrag nopan pointer-events-auto fixed inset-x-3 bottom-3 z-50 lg:hidden">
                <EditorInsertMenu
                  document={document}
                  activeTreeId={activeTreeId}
                  mode="sheet"
                  position="before"
                  onInsert={handleInsert}
                  onClose={() => data.onToggleMenu?.(null)}
                />
              </div>
            </>
          )}
        </div>
      </EdgeLabelRenderer>
    </>
  );
};
