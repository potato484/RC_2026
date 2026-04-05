import { create } from 'zustand';

export type AppLocale = 'zh-CN' | 'en-US';

interface LocaleState {
  locale: AppLocale;
  setLocale: (locale: AppLocale) => void;
}

export const useLocaleStore = create<LocaleState>((set) => ({
  locale: 'zh-CN',
  setLocale: (locale) => set({ locale }),
}));
