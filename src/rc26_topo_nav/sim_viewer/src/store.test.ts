import { describe, expect, it } from 'vitest';

import { useSimStore } from './store';

describe('useSimStore', () => {
  it('toggles layers and resets run state', () => {
    useSimStore.getState().toggleLayer('graph');
    expect(useSimStore.getState().layers.graph).toBe(false);

    useSimStore.setState({
      runId: 'abc123',
      runState: 'playing',
      frameCount: 10,
      cursor: 4,
    });
    useSimStore.getState().resetRun();

    expect(useSimStore.getState().runId).toBeNull();
    expect(useSimStore.getState().frameCount).toBe(0);
    expect(useSimStore.getState().cursor).toBe(0);
  });
});
